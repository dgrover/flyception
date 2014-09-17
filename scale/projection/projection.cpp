// projection.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

string imgfile = "test.jpg";
vector <Point> mouse;
vector <double> sqlength;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

		mouse.push_back(Point(x, y));
	}
}

int main(int argc, char** argv)
{
	// Read image from file 
	Mat img = imread(imgfile);

	//if fail to read the image
	if (img.empty())
	{
		cout << "Error loading the image" << endl;
		return -1;
	}

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

		double avg = std::accumulate(sqlength.begin(), sqlength.end(), 0.0)/sqlength.size();

		printf("\naverage distance: %f", avg);
	}

	getchar();
	return 0;

}