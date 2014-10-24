#include "stdafx.h"
#include "daq.h"

Daq::Daq()
{
	taskHandleX = 0;
	taskHandleY = 0;

	voltperdeg = 0.5;
	galvoheight = 65.0;

	for (int i = 0; i < 2; i++)
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
	DAQmxWriteAnalogF64(taskHandleX,1,1,10.0,DAQmx_Val_GroupByChannel,dataX,NULL,NULL);
	DAQmxWriteAnalogF64(taskHandleY,1,1,10.0,DAQmx_Val_GroupByChannel,dataY,NULL,NULL);
}

void Daq::ConvertPtToVoltage(Mat pt)
{
	float thetax = atan((float)pt.at<double>(0, 0) / galvoheight) * 180 / PI;
	float thetay = atan((float)pt.at<double>(1, 0) / galvoheight) * 180 / PI;

	//dataX[0] = thetax/2 * voltperdeg;
	//dataY[0] = thetay/2 * voltperdeg;

	float x = thetax/2 * voltperdeg;
	float y = thetay/2 * voltperdeg;

	dataX[0] = (x + dataX[1]) / 2;
	dataY[0] = (y + dataY[1]) / 2;

	dataX[1] = x;
	dataY[1] = y;
}