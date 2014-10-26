#ifndef DAQ_H
#define DAQ_H

using namespace cv;

#define SIZE 5

class Daq
{
	private:
		TaskHandle	taskHandleX;
		TaskHandle	taskHandleY;

		float voltperdeg;
		float galvoheight;

		float lastx, lasty;

		float64     dataX[SIZE];
		float64     dataY[SIZE];

	public:
		Daq();
		void configure();
		void start();
		void write();
		void ConvertPtToVoltage(Mat pt);


};

#endif