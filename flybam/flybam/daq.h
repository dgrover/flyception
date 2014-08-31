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

		float64     dataX[1];
		float64     dataY[1];

	public:
		Daq();
		void configure();
		void start();
		void write();
		void ConvertPixelToVoltage(Point2f p);


};

#endif