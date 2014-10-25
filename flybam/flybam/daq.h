#ifndef DAQ_H
#define DAQ_H

using namespace cv;

#define SIZE 2

class Daq
{
	private:
		TaskHandle	taskHandleX;
		TaskHandle	taskHandleY;

		float voltperdeg;
		float galvoheight;

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