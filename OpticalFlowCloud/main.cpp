#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\objdetect\objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <time.h>
#include <ctime>

#include <vector>
#include <stdio.h>
#include <Windows.h>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

string configFile;

string inputType;
string inputPath;
string outputType;
string outputPath;
int outputWidth;
int outputHeight;
int magnitudeMax;
int linesStep;
float fb_pyr_scale;
int fb_levels;
int fb_winsize;
int fb_iterations;
int fb_poly_n;
float fb_poly_sigma;
byte *IMGBUFFER = NULL;

void loadConfig() {
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(configFile, pt);
	inputType = pt.get<std::string>("inputType");
	inputPath = pt.get<std::string>("inputPath");
	outputType = pt.get<std::string>("outputType");
	outputPath = pt.get<std::string>("outputPath");
	outputWidth = pt.get<int>("outputWidth");
	outputHeight = pt.get<int>("outputHeight");
	magnitudeMax = pt.get<int>("magnitudeMax");
	linesStep = pt.get<int>("linesStep");
	fb_pyr_scale = pt.get<float>("fb_pyr_scale");
	fb_levels = pt.get<int>("fb_levels");
	fb_winsize = pt.get<int>("fb_winsize");
	fb_iterations = pt.get<int>("fb_iterations");
	fb_poly_n = pt.get<int>("fb_poly_n");
	fb_poly_sigma = pt.get<float>("fb_poly_sigma");

}

vector<string> splitString(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

vector<string> splitString(const string &s, char delim) {
	vector<string> elems;
	splitString(s, delim, elems);
	return elems;
}

//sort variables
int prefixSize, totaltextSize;

bool frameSort(cv::String a, cv::String b) 
{
	vector<string> aTokens = splitString(a, '\\');
	vector<string> bTokens = splitString(b, '\\');
	string aTrimmed = aTokens[aTokens.size() - 1];
	string bTrimmed = bTokens[bTokens.size() - 1];
	
	aTrimmed = aTrimmed.substr(prefixSize, aTrimmed.size()- totaltextSize);
	bTrimmed = bTrimmed.substr(prefixSize, bTrimmed.size() - totaltextSize);


	size_t posa = aTrimmed.find(",");
	size_t posb = bTrimmed.find(",");
	string seca = aTrimmed.substr(0, posa);
	string secb = bTrimmed.substr(0, posb);

	int ai = stoi(seca);
	int bi = stoi(secb);
	if (ai == bi) {
		string framea = aTrimmed.substr(posa+1, aTrimmed.size()-(posa+1));
		string frameb = bTrimmed.substr(posb+1, bTrimmed.size()-(posb + 1));
		ai = stoi(framea);
		bi = stoi(frameb);
	}
	return ai < bi;
}

void readDepthFile(string path, short *depthData, int width, int height) {
	std::wstring stempb = std::wstring(path.begin(), path.end());
	LPCWSTR swb = stempb.c_str();
	HANDLE _inFile = CreateFileW(swb, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	DWORD bytesRead;
	ReadFile(_inFile, depthData, width*height * sizeof(short), &bytesRead, NULL);
	CloseHandle(_inFile);
}

HRESULT SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR path)
{
	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

	BITMAPINFOHEADER bmpInfoHeader = { 0 };

	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);  // Size of the header
	bmpInfoHeader.biBitCount = wBitsPerPixel;             // Bit count
	bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
	bmpInfoHeader.biWidth = lWidth;                    // Width in pixels
	bmpInfoHeader.biHeight = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
	bmpInfoHeader.biPlanes = 1;                         // Default
	bmpInfoHeader.biSizeImage = dwByteCount;               // Image size in bytes

	BITMAPFILEHEADER bfh = { 0 };

	bfh.bfType = 0x4D42;                                           // 'M''B', indicates bitmap
	bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
	bfh.bfSize = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers


																   // Create the file on disk to write to
	HANDLE hFile = CreateFileW(path, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	// Return if error opening file
	if (NULL == hFile)
	{
		return E_ACCESSDENIED;
	}

	DWORD dwBytesWritten = 0;

	// Write the bitmap file header
	if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the bitmap info header
	if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the RGB Data
	if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Close the file
	CloseHandle(hFile);
	return S_OK;
}


Mat display_lines(Mat original, Mat flow, string pathout) {
	//By y += 5, x += 5 you can specify the grid 
	for (int y = 0; y < original.rows; y += linesStep) {
		for (int x = 0; x < original.cols; x += linesStep)
		{
			// get the flow from y, x position * 10 for better visibility
			const Point2f flowatxy = flow.at<Point2f>(y, x);
			// draw line at flow direction
			line(original, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255, 0, 0));
			//draw initial point
			Vec2f magnitude;	

			circle(original, Point(x, y), 0.2, Scalar(0, flowatxy.y*10, flowatxy.x * 10), -1);
							
		}
	}

	// draw the results
	cv::namedWindow("lines", WINDOW_AUTOSIZE);
	cv::imshow("lines", original);

	cv::Mat temp;
	original.convertTo(temp, CV_8UC3, 255);
	Mat continuousRGBA(original.size(), CV_8UC4, IMGBUFFER);
	cv::cvtColor(temp, continuousRGBA, CV_BGR2BGRA, 4);


	std::wstring stemp = std::wstring(pathout.begin(), pathout.end());
	SaveBitmapToFile(IMGBUFFER, 512, 424, 32, stemp.c_str());
	return original;
}

Mat display_direction2(Mat original, Mat flow, string pathout) {

	cv::Mat xy[2]; //X,Y
	cv::split(flow, xy);

	//calculate angle and magnitude
	cv::Mat magnitude, angle;
	cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

	//translate magnitude to range [0;1]
	double mag_max;
	cv::minMaxLoc(magnitude, 0, &mag_max);
	magnitude.convertTo(magnitude, -1, 0.4 / magnitudeMax);

	//build hsv image
	cv::Mat _hsv[3], hsv;
	_hsv[0] = angle;
	_hsv[1] = magnitude; 
	_hsv[2] = cv::Mat::ones(angle.size(), CV_32F);
	cv::merge(_hsv, 3, hsv);

	//convert to BGR and show
	cv::Mat bgr;//CV_32FC3 matrix
	cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
	cv::namedWindow("optical flow", WINDOW_AUTOSIZE);
	cv::imshow("optical flow", bgr);
	cv::Mat temp;
	bgr.convertTo(temp, CV_8UC3, 255);
	Mat continuousRGBA(bgr.size(), CV_8UC4, IMGBUFFER);
	cv::cvtColor(temp, continuousRGBA, CV_BGR2BGRA, 4);

	
	std::wstring stemp = std::wstring(pathout.begin(), pathout.end());
	SaveBitmapToFile(IMGBUFFER, 512, 424, 32, stemp.c_str());
	return bgr;
}


Mat display_direction(Mat original, Mat flow) {

	cv::Mat xy[2]; //X,Y
	cv::split(flow, xy);

	//calculate angle and magnitude
	cv::Mat magnitude, angle;
	cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

	//translate magnitude to range [0;1]
	double mag_max;
	cv::minMaxLoc(magnitude, 0, &mag_max);
	magnitude.convertTo(magnitude, -1, 1.0 / magnitudeMax);

	//build hsv image
	cv::Mat _hsv[3], hsv;
	_hsv[0] = angle;
	_hsv[1] = magnitude;
	_hsv[2] = cv::Mat::ones(angle.size(), CV_32F);
	cv::merge(_hsv, 3, hsv);

	//convert to BGR and show
	cv::Mat bgr;//CV_32FC3 matrix
	cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
	cv::namedWindow("optical flow", WINDOW_AUTOSIZE);
	cv::imshow("optical flow", bgr);

	return bgr;
}

Mat display_magnitude(Mat original, Mat flow) {

	cv::Mat xy[2]; //X,Y
	cv::split(flow, xy);

	//calculate angle and magnitude
	cv::Mat magnitude, angle;
	cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);
	
	cv::Mat sats = cv::Mat::zeros(angle.size(), CV_32F);

	//translate magnitude to range [0;1]
	double mag_max;
	cv::minMaxLoc(magnitude, 0, &mag_max);
	magnitude.convertTo(magnitude, -1, 1.0 / magnitudeMax);
	cv::Mat saturation = cv::Mat::zeros(magnitude.size(), CV_32F);

	/*for (int x = 0; x < magnitude.cols; x++)
		for (int y = 0; y < magnitude.rows; y++) {

			if (magnitude.at<float>(y, x) < 0.01) {
				saturation.at<float>(y, x) = 0;
			}
			else {
				saturation.at<float>(y, x) = 1;

			}
		}*/

	//build hsv image
	cv::Mat _hsv[3], hsv;
	_hsv[0] = (260 - (magnitude * 260));
	_hsv[1] =magnitude;
	_hsv[2] = cv::Mat::ones(angle.size(), CV_32F);
	cv::merge(_hsv, 3, hsv);

	//convert to BGR and show
	cv::Mat bgr;//CV_32FC3 matrix
	cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
	cv::namedWindow("optical flow", WINDOW_AUTOSIZE);
	cv::imshow("optical flow", bgr);
	return bgr;

}

Mat display_zdirection(Mat original, Mat flow, Mat zflow, string pathout) {

	cv::Mat xy[2]; //X,Y
	cv::Mat zr;
	cv::Mat zg;

	cv::split(flow, xy);
	//cv::minMaxLoc(zflow, 0, &depthMax);

	zflow.convertTo(zr, xy[0].depth(), 1.0 / magnitudeMax);
	cv::medianBlur(zr, zr, 5);
	zr.copyTo(zg);

	std::for_each(xy[0].begin<float>(), xy[0].end<float>()
		, [](float& pixel) {
		pixel = pixel + magnitudeMax / (2 * magnitudeMax);
	});

	std::for_each(xy[1].begin<float>(), xy[1].end<float>()
		, [](float& pixel) {
		pixel = pixel + magnitudeMax / (2 * magnitudeMax);

	});
	//std::for_each(zg.begin<float>(), zg.end<float>()
	//	, [](float& pixel) {
	//	pixel = pixel + magnitudeMax / (2 * magnitudeMax);
	//});


	//build hsv image
	cv::Mat _bgr[3], bgr, rgba;
	_bgr[0] = xy[0];
	_bgr[1] = xy[1];
	_bgr[2] = zg; //cv::Mat::ones(angle.size(), CV_32F);
	cv::merge(_bgr, 3, bgr);

	//convert to BGR and show
	//cv::Mat bgr;//CV_32FC3 matrix
	//cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
	cv::namedWindow("optical flow", WINDOW_AUTOSIZE);
	cv::imshow("optical flow", bgr);
	bgr.convertTo(rgba, CV_8UC4, 255);

	imwrite(pathout, rgba);

	return bgr;


}

Mat display_zmagnitude(Mat original, Mat flow, Mat zflow, string pathout) {

	cv::Mat xy[2],z; //X,Y,Z
	cv::split(flow, xy);

	for (int x = 0; x < zflow.cols; x++)
		for (int y = 0; y < zflow.rows; y++) {
			if (zflow.at<short>(y, x) > 50 || zflow.at<short>(y, x) < -50) {
				zflow.at<short>(y, x) = 0;
			}
		}
	
	zflow.convertTo(z, xy[0].depth(), 0.4/ magnitudeMax);


	xy[0].convertTo(xy[0], -1, 1.0 / magnitudeMax);
	xy[1].convertTo(xy[1], -1, 1.0 / magnitudeMax);


	cv::medianBlur(z, z, 5);

	//calculate angle and magnitude
	cv::Mat magnitude = cv::Mat::zeros(z.size(), CV_32F);
	cv::Mat saturation = cv::Mat::zeros(z.size(), CV_32F);

	for (int x = 0; x < z.cols; x++)
		for (int y = 0; y < z.rows; y++){
			
			magnitude.at<float>(y, x) = sqrt(pow(xy[0].at<float>(y, x), 2) + pow(xy[1].at<float>(y, x), 2) + pow(z.at<float>(y, x), 2));
			if (magnitude.at<float>(y, x) > 0.005) {
				saturation.at<float>(y, x) = 1;
			}
			else {
				saturation.at<float>(y, x) = 0;

			}
		}


	//translate magnitude to range [0;1]
	double mag_max;
	cv::minMaxLoc(magnitude, 0, &mag_max);
	magnitude.convertTo(magnitude, -1, 1.0 / magnitudeMax);


	//build hsv image
	cv::Mat _hsv[3], hsv;
	_hsv[0] = (260 - (magnitude * 260));
	
	_hsv[1] = saturation;
	_hsv[2] = cv::Mat::ones(z.size(), CV_32F);
	cv::merge(_hsv, 3, hsv);

	//convert to BGR and show
	cv::Mat bgr;//CV_32FC3 matrix
	cv::Mat temp;
	cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
	cv::namedWindow("optical flow", WINDOW_AUTOSIZE);
	cv::imshow("optical flow", bgr);
	bgr.convertTo(temp, CV_8UC3, 255);
	Mat continuousRGBA(bgr.size(), CV_8UC4, IMGBUFFER);
	cv::cvtColor(temp, continuousRGBA, CV_BGR2BGRA, 4);
	

	std::wstring stemp = std::wstring(pathout.begin(), pathout.end());
	SaveBitmapToFile(IMGBUFFER, 512, 424, 32, stemp.c_str());

	return bgr;

}


int mainFlowVideo()
{
	// add your file name
	VideoCapture cap(inputPath);
	int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
	VideoWriter outputVideo;
	bool open = outputVideo.open(outputPath,ex, cap.get(CV_CAP_PROP_FPS), Size(1280, 720), true);
	if (!open) {
		std::cerr << "Error opening video" << endl;
		getchar();
		return 1;
	}
	Mat flow, frame, prevgray;

	for (;;)
	{

		bool Is = cap.grab();
		if (Is == false) {
			// if video capture failed
			cout << "Video Capture Fail" << endl;
			break;
		}
		else {

			Mat img;
			Mat original;

			// capture frame from video file
			cap.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);
			resize(img, img, Size(1280, 720));

			// save original for later
			img.copyTo(original);

			// just make current frame gray
			cv::cvtColor(img, img, COLOR_BGR2GRAY);

			if (prevgray.empty() == false) {

				// calculate optical flow 
				calcOpticalFlowFarneback(prevgray, img, flow, fb_pyr_scale, fb_levels, fb_winsize, fb_iterations,fb_poly_n, fb_poly_sigma, OPTFLOW_FARNEBACK_GAUSSIAN);
				if (outputType == "magnitude") {
					outputVideo << display_magnitude(original, flow);
				}
				else if (outputType == "direction") {
					outputVideo << display_direction(original, flow);
				}
				else if (outputType == "lines") {
					outputVideo << display_lines(original, flow,"");
				}
				else {
					cerr << "Invalid output type: " << outputType << endl;
					exit(-1);
				}
				 
				// fill previous image again
				img.copyTo(prevgray);

			}
			else {

				// fill previous image in case prevgray.empty() == true
				img.copyTo(prevgray);

			}


			int key1 = waitKey(20);

		}
	}
	outputVideo.release();
	return 0;
}


bool replace(std::string& str, const std::string& from, const std::string& to) {
	size_t start_pos = str.find(from);
	if (start_pos == std::string::npos)
		return false;
	str.replace(start_pos, from.length(), to);
	return true;
}

int mainFlowImages()
	{
		float maxChange = 50.0;

		stringstream ss;
		ss << inputPath << "/color/*.bmp";
	
		cv::String path(ss.str()); //select only bmp
		vector<cv::String> fn;
		vector<cv::Mat> data;
		cv::glob(path, fn, true); // recurse
		prefixSize = 5;
		totaltextSize = 9;
		std::sort(fn.begin(), fn.end(), frameSort);

		//for (size_t k = 0; k<fn.size(); ++k)
		//{
		//	cv::Mat im = cv::imread(fn[k]);
		//	if (im.empty()) continue; //only proceed if sucsessful
		//	data.push_back(im);
		//}


		short *depths = (short*)malloc(sizeof(short) * 512 * 424);
		
		ss.str("");
		ss << inputPath << "/depthbgFiltered/*";
		cv::String pathD(ss.str()); //select only bmp
		vector<cv::String> fnDepth;
		cv::glob(pathD, fnDepth, true); // recurse
		prefixSize = 9;
		std::sort(fnDepth.begin(), fnDepth.end(), frameSort);

		Mat flow, frame, prevgray, prevDepth;

		for (size_t k = 0; k<fn.size(); k++)
		{
			
			cv::Mat image = cv::imread(fn[k]);
			if (image.empty()) continue;
			Mat original;
			image.copyTo(original);
			cv::cvtColor(image, image, COLOR_BGR2GRAY);
			readDepthFile(fnDepth[k], depths, 512, 424);

			if(IMGBUFFER == NULL){
				int size = image.total() * 4;
				IMGBUFFER = new byte[size];  // you will have to delete[] that later
			}

			cv::Mat dImage(424,512 , DataType<short>::type);
			 
			for (int i = 0; i <512; i++) {
				for (int j = 0; j < 424; j++) {
					dImage.at<short>(j, i) = depths[j * 512 + i];
			
				}
				
			}
			

			if (!prevgray.empty())
			{
				// calculate optical flow 
				cv::medianBlur(image, image, 5);
				calcOpticalFlowFarneback(prevgray, image, flow, fb_pyr_scale, fb_levels, fb_winsize, fb_iterations,fb_poly_n, fb_poly_sigma, OPTFLOW_FARNEBACK_GAUSSIAN);
				Mat zFlow = dImage - prevDepth;

			
				string sout = fn[k-1];
				if(outputType == "magnitude"){
					//imwrite(,
					replace(sout, "/color", "/mag");
					display_zmagnitude(original, flow, zFlow,sout);
				}
				else if (outputType == "direction") {
					replace(sout, "/color", "/flow");
					display_direction2(original, flow,sout);
				}
				else if (outputType == "zdirection") {
					replace(sout, "/color", "/flow");
					sout,display_zdirection(original, flow, zFlow,sout);
				}
				else if(outputType == "lines"){
					replace(sout, "/color", "/lines");
					display_lines(original, flow, sout);
				}
				else {
					cerr << "Invalid output type: " << outputType << endl;
					exit(-1);
				}
				// fill previous image again
				image.copyTo(prevgray);
				dImage.copyTo(prevDepth);
			}
			else
			{
				// fill previous image in case prevgray.empty() == true
				image.copyTo(prevgray);
				dImage.copyTo(prevDepth);

			}
			//without this you can't see in the screen
			int key1 = waitKey(20);
		}
		return 0;
	}



void main(int argc, const char** argv) {
	configFile = argv[1];
	loadConfig();
	if (inputType == "video") 
	{
		mainFlowVideo();
	}
	else if (inputType == "pics") 
	{
		mainFlowImages();
	}
}