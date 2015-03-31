#ifndef DAQ_H
#define DAQ_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

#define SIZE 1

class Daq
{
private:
	TaskHandle	taskHandleX;
	TaskHandle	taskHandleY;

	float voltperdeg;
	float galvoheight;

	float thetax, thetay;

	float64     dataX[SIZE];
	float64     dataY[SIZE];

public:
	Daq();
	void reset();
	void configure();
	void start();
	void write();
	void ConvertPtToDeg(Point2f pt);
	void ConvertPixelToDeg(float x, float y);
	Point2f Daq::ConvertDegToPt();
};

#endif