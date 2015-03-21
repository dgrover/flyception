#include "stdafx.h"
#include "daq.h"

Daq::Daq()
{
	taskHandleX = 0;
	taskHandleY = 0;

	voltperdeg = 0.5;
	galvoheight = 65.0;

	lastx = 0.0;
	lasty = 0.0;

	for (int i = 0; i < SIZE; i++)
	{
		dataX[i] = 0.0;
		dataY[i] = 0.0;
	}
}

void Daq::configure()
{
	// DAQmx Configure Code
	DAQmxCreateTask("", &taskHandleX);
	DAQmxCreateTask("", &taskHandleY);
	DAQmxCreateAOVoltageChan(taskHandleX, "Dev1/ao0", "", -10.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskHandleY, "Dev1/ao1", "", -10.0, 10.0, DAQmx_Val_Volts, "");
}

void Daq::start()
{
	// DAQmx Start Code
	DAQmxStartTask(taskHandleX);
	DAQmxStartTask(taskHandleY);
}

void Daq::write()
{
	// DAQmx Write Code
	DAQmxWriteAnalogF64(taskHandleX,SIZE,0,10.0,DAQmx_Val_GroupByChannel,dataX,NULL,NULL);
	DAQmxWriteAnalogF64(taskHandleY,SIZE,0,10.0,DAQmx_Val_GroupByChannel,dataY,NULL,NULL);

	//function for scalar value
	//DAQmxWriteAnalogScalarF64(taskHandleX, 0, 10.0, dataX, NULL);
}

void Daq::ConvertPtToVoltage(Point2f pt)
{
	// move mirror by half the angle
	float thetax = atan(pt.x / galvoheight) * 180 / (CV_PI * 2);
	float thetay = atan(pt.y / galvoheight) * 180 / (CV_PI * 2);

	dataX[0] = thetax * voltperdeg;
	dataY[0] = thetay * voltperdeg;

	lastx = thetax;
	lasty = thetay;
}

Point2f Daq::ConvertPixelToVoltage(float x, float y)
{
	float thetax = lastx + x;
	float thetay = lasty + y;

	dataX[0] = thetax * voltperdeg;
	dataY[0] = thetay * voltperdeg;

	lastx = thetax;
	lasty = thetay;

	Point2f pt;

	//final point would be at double the mirror angle
	pt.x = galvoheight * tan(lastx * 2 * CV_PI / 180);
	pt.y = galvoheight * tan(lasty * 2 * CV_PI / 180);

	return pt;
}

void Daq::reset()
{
	lastx = 0.0;
	lasty = 0.0;
}
