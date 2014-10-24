#ifndef DAQ_H
#define DAQ_H

using namespace cv;

class Daq
{
	private:
		TaskHandle	taskHandleX;
		TaskHandle	taskHandleY;

		float voltperdeg;
		float galvoheight;

		float64     dataX[2];
		float64     dataY[2];

	public:
		Daq();
		void configure();
		void start();
		void write();
		void ConvertPtToVoltage(Mat pt);


};

#endif