#include "stdafx.h"
#include "daq.h"

Daq::Daq()
{
	taskHandleX = 0;
	taskHandleY = 0;

	thetax = 0.0;
	thetay = 0.0;

	dataX[0] = 0.0;
	dataY[0] = 0.0;
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

	dataX[0] = XOFFSET + (thetax * XVOLTPERDEGREE);
	dataY[0] = YOFFSET + (thetay * YVOLTPERDEGREE);
	
	// DAQmx Write Code
	DAQmxWriteAnalogF64(taskHandleX, STEP_SIZE, 0, 10.0, DAQmx_Val_GroupByChannel, dataX, NULL, NULL);
	DAQmxWriteAnalogF64(taskHandleY, STEP_SIZE, 0, 10.0, DAQmx_Val_GroupByChannel, dataY, NULL, NULL);

	//function for scalar value
	//DAQmxWriteAnalogScalarF64(taskHandleX, 0, 10.0, dataX, NULL);
}

void Daq::ConvertPtToDeg(Point2f pt)
{
	// move mirror by half the angle
	thetax = atan(pt.x / (GALVO_Y_HEIGHT + GALVO_XY_DIST)) * 180 / (CV_PI * 2);
	thetay = atan(pt.y / ( GALVO_Y_HEIGHT / cos(thetax * 2 * CV_PI / 180 ) )  ) * 180 / (CV_PI * 2);
}

void Daq::ConvertPixelToDeg(float x, float y)
{
	thetax = thetax + x;
	thetay = thetay + y;
}

Point2f Daq::ConvertDegToPt()
{
	Point2f pt;

	//final point would be at double the mirror angle
	pt.x = (GALVO_Y_HEIGHT + GALVO_XY_DIST) * tan(thetax * 2 * CV_PI / 180);
	pt.y = (GALVO_Y_HEIGHT / cos(thetax * 2 * CV_PI / 180)) * tan(thetay * 2 * CV_PI / 180);

	return pt;
}

void Daq::reset()
{
	thetax = 0.0;
	thetay = 0.0;
}

Point2f Daq::GetGalvoAngles()
{
	Point2f angle;

	angle.x = thetax;
	angle.y = thetay;

	return angle;
}

void Daq::MoveLeft()
{
	thetax += 0.1;
}

void Daq::MoveRight()
{
	thetax -= 0.1;
}

void Daq::MoveUp()
{
	thetay += 0.1;
}

void Daq::MoveDown()
{
	thetay -= 0.1;
}