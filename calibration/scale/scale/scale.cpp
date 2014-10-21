// scale.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


using namespace std;
using namespace cv;

string imgfile = "..\\..\\..\\arena\\flyview.jpg";
vector <Point> mouse;
vector <double> sqlength;

Mat rotateImage(Mat src, double angle)
{
	Mat dst;
	Point2f pt(src.cols / 2., src.rows / 2.);
	Mat r = getRotationMatrix2D(pt, angle, 1.0);
	warpAffine(src, dst, r, Size(src.cols, src.rows));
	return dst;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

		mouse.push_back(Point(x, y));
	}
}


int _tmain(int argc, _TCHAR* argv[])
{
	// Read image from file 
	Mat img = imread(imgfile);

	//if fail to read the image
	if (img.empty())
	{
		cout << "Error loading the image" << endl;
		return -1;
	}

	img = rotateImage(img, 15);

	//Create a window
	namedWindow("src", 1);

	//set the callback function for any mouse event
	setMouseCallback("src", CallBackFunc, NULL);

	//show the image
	imshow("src", img);

	waitKey();

	if (mouse.size() >1)
	{
		for (int i = 1; i < mouse.size(); i++)
			sqlength.push_back(cv::norm(mouse[0] - mouse[i]));

		for (int i = 0; i < sqlength.size(); i++)
			printf("%f\n", sqlength.at(i));

		double avg = std::accumulate(sqlength.begin(), sqlength.end(), 0.0) / sqlength.size();

		printf("\naverage distance: %f", avg);
	}

	getchar();

	return 0;
}

