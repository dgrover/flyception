#include "stdafx.h"
#include "daq.h"

#define PI 3.14159265

Daq::Daq()
{
	taskHandleX = 0;
	taskHandleY = 0;

	voltperdeg = 0.5;
	galvoheight = 1000.0;

	dataX[1] = { 0.0 };
	dataY[1] = { 0.0 };
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
	float thetax = atan(pt.at<double>(0, 0) / galvoheight) * 180 / PI;
	float thetay = atan(pt.at<double>(1, 0) / galvoheight) * 180 / PI;

	dataX[0] = thetax/2 * voltperdeg;
	dataY[0] = thetay/2 * voltperdeg;
}