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
#include "PositionStream.h"

using namespace cv;
using namespace std;

string configFile;

string inputType;
string inputPath;
string outputType;
string outputPath;
string resultsPath;
float distanceThreshold = 15;

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
vector<PositionStream> _trackedPeople;

void loadConfig() {
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(configFile, pt);
	inputType = pt.get<std::string>("inputType");
	inputPath = pt.get<std::string>("inputPath");
	outputType = pt.get<std::string>("outputType");
	outputPath = pt.get<std::string>("outputPath");
	resultsPath = pt.get<std::string>("resultsPath");
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
	distanceThreshold = pt.get<float>("distanceThreshold");
}

vector<string> splitString(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

Mat display_lines(Mat original, Mat flow) {
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

			circle(original, Point(x, y), 0.2, Scalar(0, flowatxy.y * 10, flowatxy.x * 10), -1);

		}
	}

	// draw the results
	cv::namedWindow("lines", WINDOW_AUTOSIZE);
	cv::imshow("lines", original);
	return original;
}

Mat display_average_direction(Mat original, Mat flow, Rect r) {

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

cv::Vec3b HSVtoBGR(const cv::Vec3f& hsv)
{
	cv::Mat_<cv::Vec3f> hsvMat(hsv);
	cv::Mat_<cv::Vec3f> bgrMat;

	cv::cvtColor(hsvMat, bgrMat, CV_HSV2BGR);

	bgrMat *= 255; // Upscale after conversion

				   // Conversion to Vec3b is handled by OpenCV, no need to static_cast
	return bgrMat(0);
}

Mat display_average_magnitude(Mat original, Mat flow, Rect r,float &magtop, float &magtorso, float &maglegs) {

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

	int y, x, ylimit, xlimit;
	float magAvg = 0;
	float npix = 0;
	Point2i br;
	Point2i tl;

	//Head -----------------------------
	for (y = r.tl().y, ylimit = r.y + r.height*0.25; y < ylimit && y < outputHeight; y++) {
		for (x = r.tl().x, xlimit = r.br().x; x < xlimit && x < outputWidth; x++)
		{
			if (y >= 0 && x >= 0 && magnitude.at<float>(y, x) > 0.001) {
				magAvg += magnitude.at<float>(y, x);
				npix++;
			}

		}
	}
	magAvg = magAvg / ((float)npix);
	magtop = magAvg;
	tl.x = r.tl().x;
	tl.y = r.tl().y;
	br.x = xlimit;
	br.y = ylimit;
	int sat = 1;
	if (magAvg < 0.001)
		sat = 0;
	Vec3f hsvh(260 - (magAvg * 260), sat, 1);
	Vec3b bgrh = HSVtoBGR(hsvh);
	rectangle(original, tl, br, cv::Scalar(bgrh[0], bgrh[1], bgrh[2]), 1);

	//Torso--------------------------------
	magAvg = 0;
	npix = 0;
	for (y = r.y + r.height*0.25 + 1, ylimit = r.y + r.height*0.65; y < ylimit && y < outputHeight; y++) {
		for (x = r.tl().x, xlimit = r.br().x; x < xlimit && x < outputWidth; x++)
		{
			if (y >= 0 && x >= 0 && magnitude.at<float>(y, x) > 0.001) {
				magAvg += magnitude.at<float>(y, x);
				npix++;
			}

		}
	}
	magAvg = magAvg / ((float)npix);
	magtorso = magAvg;
	tl.x = r.tl().x;
	tl.y = r.y + r.height*0.25 + 1;
	br.x = xlimit;
	br.y = ylimit;
	sat = 1;
	if (magAvg < 0.001)
		sat = 0;
	Vec3f hsvt(260 - (magAvg * 260), sat, 1);
	Vec3b bgrt = HSVtoBGR(hsvt);
	rectangle(original, tl, br, cv::Scalar(bgrt[0], bgrt[1], bgrt[2]), 1);


	//Legs--------------------------------
	magAvg = 0;
	npix = 0;
	for (y = r.y + r.height*0.65 + 1, ylimit = r.br().y; y < ylimit&& y < outputHeight; y++) {
		for (x = r.tl().x, xlimit = r.br().x; x < xlimit && x < outputWidth; x++)
		{
			if (y > 0 && x > 0 && magnitude.at<float>(y, x) > 0.001) {
				magAvg += magnitude.at<float>(y, x);
				npix++;
			}

		}
	}
	magAvg = magAvg / ((float)npix);
	maglegs = magAvg;
	tl.x = r.tl().x;
	tl.y = r.y + r.height*0.65 + 1;
	br.x = xlimit;
	br.y = ylimit;
	sat = 1;
	if (magAvg < 0.001)
		sat = 0;
	Vec3f hsvl(260 - (magAvg * 260), sat, 1);
	Vec3b bgrl = HSVtoBGR(hsvl);
	rectangle(original, tl, br, cv::Scalar(bgrl[0], bgrl[1], bgrl[2]), 1);

	////build hsv image
	//cv::Mat _hsv[3], hsv;
	//_hsv[0] = (260 - (magnitude * 260));
	//_hsv[1] = magnitude;
	//_hsv[2] = cv::Mat::ones(angle.size(), CV_32F);
	//cv::merge(_hsv, 3, hsv);

	////convert to BGR and show
	//cv::Mat bgr;//CV_32FC3 matrix
	//cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
	//cv::namedWindow("optical flow", WINDOW_AUTOSIZE);
	//cv::imshow("optical flow", bgr);
	//return bgr;
	return original;
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


	//build hsv image
	cv::Mat _hsv[3], hsv;
	_hsv[0] = (360 - (magnitude * 360));
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

bool replace(std::string& str, const std::string& from, const std::string& to) {
	size_t start_pos = str.find(from);
	if (start_pos == std::string::npos)
		return false;
	str.replace(start_pos, from.length(), to);
	return true;
}

//---------------------//
bool setupDone = false;
//std::vector<Vec2i> positions;
int framesSkipped = 0;

//
//void CallBackFunc(int event, int x, int y, int flags, void* userdata)
//{
//	if (event == EVENT_LBUTTONDOWN)
//	{
//		positions.push_back(Vec2i(x, y));
//		std::cout << "Added position " << x << " " << y << endl;
//	}
//}


Mat MatchClosestPerson(Rect box,int &lastid, int currentFrame, Mat original, Mat flow)
{
	float maghead, magtorso, maglegs;
	Mat res = display_average_magnitude(original, flow, box,maghead,magtorso,maglegs);
	int boxx = box.x + box.width / 2.0;
	int boxy = box.y + box.height / 2.0;
	
	int chosenid;
	vector<int> qualifiableTargets;
	for (int k = 0; k < _trackedPeople.size(); k++) 
	{
		if (!_trackedPeople[k]._active) continue;
		Vec2i position = _trackedPeople[k]._positions[_trackedPeople[k]._positions.size()-1];
		float distance = sqrt(pow(boxx - position[0], 2) + pow(boxy - position[1], 2));
		if (distance < distanceThreshold) {
			qualifiableTargets.push_back(k);
		}
	}

	if (qualifiableTargets.size() > 1) {
		for (int i = 0; i < qualifiableTargets.size(); i++) {
			_trackedPeople[qualifiableTargets[i]]._fresh = false;
		}
	}
	
	if(qualifiableTargets.size() ==1) {
		_trackedPeople[qualifiableTargets[0]]._fresh = true;
		_trackedPeople[qualifiableTargets[0]]._positions.push_back(Vec2i(boxx, boxy));
		_trackedPeople[qualifiableTargets[0]]._topValue.push_back(maghead);
		_trackedPeople[qualifiableTargets[0]]._midValue.push_back(magtorso);
		_trackedPeople[qualifiableTargets[0]]._bottomValue.push_back(maglegs);
		_trackedPeople[qualifiableTargets[0]].drawPath(res);
		chosenid = _trackedPeople[qualifiableTargets[0]]._id;
	}
	else {
		PositionStream p(++lastid, currentFrame);
		p._fresh = true;
		p._positions.push_back(Vec2i(boxx, boxy));
		p._topValue.push_back(maghead);
		p._midValue.push_back(magtorso);
		p._bottomValue.push_back(maglegs);
		
		_trackedPeople.push_back(p);
		chosenid = lastid;

	}
	stringstream id;
	id << chosenid;
	cv::putText(res, id.str(), box.br(), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
	return res;

}

void WriteCSVFile() {
	ofstream s;
	s.open(resultsPath.c_str());
	s << "id,frame,posx,posy,top,center,bot" << endl;
	for each (PositionStream ps in _trackedPeople)
	{
		for (int i = 0; i < ps._positions.size(); i++) {
			s << ps._id << "," << (ps._firstFrame + i) << "," << ps._positions[i][0] << "," << ps._positions[i][1] << "," << ps._topValue[i] << "," << ps._midValue[i] << "," << ps._bottomValue[i] << endl;
		}
	}
	s.flush();
	s.close();
}

int main(int argc, const char** argv)
{
	configFile = argv[1];
	loadConfig();

	VideoCapture cap(inputPath);
	HOGDescriptor hog;
	vector<float> detector = hog.getDefaultPeopleDetector();

	cv::namedWindow("people detector", WINDOW_AUTOSIZE);
	//setMouseCallback("people detector", CallBackFunc, NULL);
	VideoWriter outputVideo;
	int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
	bool open = outputVideo.open(outputPath, ex, cap.get(CV_CAP_PROP_FPS), Size(1280, 720), true);
	if (!open) {
		std::cerr << "Error opening video" << endl;
		return 1;
	}

	int personID = 0;
	int frameID = 0;
	hog.setSVMDetector(detector);
	Mat flow, prevgray;

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
			Mat imgbw;

			// capture frame from video file
			cap.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);
			resize(img, img, Size(1280, 720));
			frameID++;
			//SETUP STAGE---------------------------------------------------------
			if(!setupDone) {
				cv::imshow("people detector", img);
				int key = waitKey(0);
				if (key == 'd' || key == 'D') {
					cv::cvtColor(img, imgbw, COLOR_BGR2GRAY);
					vector<Rect> found, found_filtered;
					hog.detectMultiScale(imgbw, found, 0, Size(8, 8), Size(32, 32), 1.05, 3);
					size_t i, j;
					for (i = 0; i < found.size(); i++)
					{
						Rect r = found[i];
						for (j = 0; j < found.size(); j++)
							if (j != i && (r & found[j]) == r)
								break;
						if (j == found.size())
							found_filtered.push_back(r);
					}
					for (i = 0; i < found_filtered.size(); i++)
					{
						Rect r = found_filtered[i];
						rectangle(img, r.tl(), r.br(), cv::Scalar(0, 255, 0), 3);
						circle(img, Point(r.x, r.y), 0.2, cv::Scalar(255, 0, 255),3);		

					}
					cv::imshow("people detector", img);
					key = waitKey(0);
				}
				if (key == 's' || key == 'S'){
					framesSkipped++;
					std::cout << "Frame skipped"<< endl;
					continue;
				}
				
				if (key == 13){
					setupDone = true;
					std::cout << "starting! frames skipped: "<< framesSkipped <<  endl;
					//setMouseCallback("people detector", NULL, NULL);
				}
			}else{
				//SETUP DONE-------------------------------------------------------------
				
				cv::cvtColor(img, imgbw, COLOR_BGR2GRAY);
				if (prevgray.empty() == false) {
					calcOpticalFlowFarneback(prevgray, imgbw, flow, fb_pyr_scale, fb_levels, fb_winsize, fb_iterations, fb_poly_n, fb_poly_sigma, OPTFLOW_FARNEBACK_GAUSSIAN);
					vector<Rect> found, found_filtered;
					hog.detectMultiScale(imgbw, found, 0, Size(8, 8), Size(32, 32), 1.05, 3);
					size_t i, j;

					for (i = 0; i < found.size(); i++)
					{
						Rect r = found[i];
						for (j = 0; j < found.size(); j++)
							if (j != i && (r & found[j]) == r)
								break;
						if (j == found.size())
							found_filtered.push_back(r);
					}			

					for (i = 0; i < _trackedPeople.size(); i++) 
					{
						_trackedPeople[i]._fresh = false;
					}

					for (i = 0; i< found_filtered.size(); i++)
					{
						img = MatchClosestPerson(found_filtered[i],personID,frameID,img,flow);
					}

					for (i = 0; i < _trackedPeople.size(); i++)
					{
						if (!_trackedPeople[i]._fresh)
							_trackedPeople[i]._active = false;
					}

					//for (i = 0; i < found.size(); i++)
					//{
					//	Rect r = found[i];
					//	//positions.push_back(Vec2i(found[i].x, found[i].y));
					//}
					cv::imshow("people detector", img);
					waitKey(1);
			
					outputVideo << img;
					imgbw.copyTo(prevgray);

				}
				else {

					// fill previous image in case prevgray.empty() == true
					imgbw.copyTo(prevgray);

				}
			}
		}
	}
	WriteCSVFile();
	outputVideo.release();
	return 0;
}
