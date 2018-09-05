#include "PositionStream.h"



PositionStream::PositionStream(int id, int firstFrame)
{
	_id = id;
	_firstFrame = firstFrame;
	_active = true;
	_fresh = false;
}


PositionStream::~PositionStream()
{
}

void PositionStream::drawPath(Mat &img) 
{
	cv::Vec2i lastPoint(-1, -1);
	for each (cv::Vec2i pos in _positions)
	{
		if (lastPoint[0] != -1 && lastPoint[1] != -1) {
			cv::line(img, lastPoint, pos, Scalar(0, 0, 255));
		}
		lastPoint = pos;
	}
}