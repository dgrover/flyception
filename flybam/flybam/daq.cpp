#include "stdafx.h"
#include "daq.h"

#define PI 3.14159265

Daq::Daq()
{
	taskHandleX = 0;
	taskHandleY = 0;

	voltperdeg = 0.5;
	galvoheight = 10.0;

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

void Daq::ConvertPixelToVoltage(Point2f p)
{
	//dataX[0] = (p.x / imageWidth * maxVoltage) - maxVoltage / 2;
	//dataY[0] = (p.y / imageHeight * maxVoltage) - maxVoltage / 2;

	float thetax = atan(p.x / galvoheight) * 180 / PI;
	float thetay = atan(p.y / galvoheight) * 180 / PI;

	dataX[0] = thetax * voltperdeg;
	dataY[0] = thetay * voltperdeg;
}