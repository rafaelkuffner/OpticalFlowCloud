#pragma once

#include <vector>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

class PositionStream
{

	
public:
	bool _active;
	bool _fresh;
	int _id;
	int _firstFrame;
	vector<Vec2i> _positions;
	vector<float> _topValue;
	vector<float> _midValue;
	vector<float> _bottomValue;

	PositionStream(int id, int firstFrame);
	~PositionStream();
	void drawPath(Mat &img);
};

