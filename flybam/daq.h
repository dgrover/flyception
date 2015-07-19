#ifndef DAQ_H
#define DAQ_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

#define STEP_SIZE 1

class Daq
{
private:
	TaskHandle	taskHandleX;
	TaskHandle	taskHandleY;

	float thetax, thetay;

	float64     dataX[STEP_SIZE];
	float64     dataY[STEP_SIZE];

public:
	Daq();
	void reset();
	void configure();
	void start();
	void write();
	void ConvertPtToDeg(Point2f pt);
	void ConvertPixelToDeg(float x, float y);
	Point2f ConvertDegToPt();
	Point2f GetGalvoAngles();
	void MoveLeft();
	void MoveRight();
	void MoveUp();
	void MoveDown();
};

#endif