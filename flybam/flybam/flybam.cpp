// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

void ConvertPixelToVoltage(Point p, int imageWidth, int imageHeight, int maxVoltage, float64 dataX[], float64 dataY[])
{
	dataX[0] = ((float)(p.x) / imageWidth * maxVoltage) - maxVoltage / 2;
	dataY[0] = ((float)(p.y) / imageHeight * maxVoltage) - maxVoltage / 2;
}

int _tmain(int argc, _TCHAR* argv[])
{
	Point p;

	Flycam arena_cam;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;

	Error error;

	int nframes, imageWidth, imageHeight, success;

	FmfReader fmf;

	if (argc == 2)
	{
		//fmf.GetFileExtension(string(argv[1]));

		success = fmf.Open(argv[1]);
		success = fmf.ReadHeader();
		nframes = fmf.GetFrameCount();
		fmf.GetImageSize(imageWidth, imageHeight);
	}
	else
	{
		nframes = -1;

		error = busMgr.GetNumOfCameras(&numCameras);

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

		printf("Number of cameras detected: %u\n", numCameras);

		if (numCameras < 1)
		{
			printf("Insufficient number of cameras... exiting\n");
			return -1;
		}

		//Get arena camera information
		error = busMgr.GetCameraFromIndex(0, &guid);

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

		error = arena_cam.Connect(guid);

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

		error = arena_cam.SetCameraParameters();

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

		arena_cam.GetImageSize(imageWidth, imageHeight);

		// Start arena camera
		error = arena_cam.Start();

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}
	}

	Tracker tkf;

	TaskHandle	taskHandleX=0;
	TaskHandle	taskHandleY=0;

	float64     dataX[1] = {0.0};
	float64     dataY[1] = {0.0};

	// DAQmx Configure Code
	DAQmxCreateTask("",&taskHandleX);
	DAQmxCreateTask("",&taskHandleY);
	DAQmxCreateAOVoltageChan(taskHandleX,"Dev1/ao0","",-10.0,10.0,DAQmx_Val_Volts,"");
	DAQmxCreateAOVoltageChan(taskHandleY,"Dev1/ao1","",-10.0,10.0,DAQmx_Val_Volts,"");

	// DAQmx Start Code
	DAQmxStartTask(taskHandleX);
	DAQmxStartTask(taskHandleY);

	BackgroundSubtractorMOG mog_cpu;
	mog_cpu.set("nmixtures", 3);
	Mat frame, fgmask;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	for (int imageCount = 0; imageCount != nframes; imageCount++)
	{
		if (argc == 2)
			frame = fmf.ReadFrame(imageCount);
		else
		 	frame = arena_cam.GrabFrame();

		if (!frame.empty())
		{
					mog_cpu(frame, fgmask, 0.01);

					findContours(fgmask, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
					vector<RotatedRect> minEllipse(contours.size());

					for (int i = 0; i < contours.size(); i++)
					{
						if (contours[i].size() > 25)
						{
							minEllipse[i] = fitEllipse(Mat(contours[i]));
							drawContours(fgmask, contours, 0, Scalar(255, 255, 255), CV_FILLED, 8, hierarchy);
							circle(frame, minEllipse[i].center, 1, Scalar(255, 255, 255), CV_FILLED, 1);
							ellipse(frame, minEllipse[i], Scalar(255, 255, 255), 1, 1);

							tkf.Predict(minEllipse[i].center.x, minEllipse[i].center.y);
							tkf.Correct();
							tkf.GetTrackedPoint(p);
						}
					}

					imshow("raw image", frame);
					imshow("FG mask", fgmask);
		}
		else
			p = fmf.ReadFrame();		//Read coordinates from txt file

		ConvertPixelToVoltage(p, imageWidth, imageHeight, 3.0, dataX, dataY);
		//printf("%f %f\n", dataX[0], dataY[0]);

		// DAQmx Write Code
		DAQmxWriteAnalogF64(taskHandleX,1,1,10.0,DAQmx_Val_GroupByChannel,dataX,NULL,NULL);
		DAQmxWriteAnalogF64(taskHandleY,1,1,10.0,DAQmx_Val_GroupByChannel,dataY,NULL,NULL);

		waitKey(1);

		if ( GetAsyncKeyState(VK_ESCAPE) )
			break;
	}

	if (argc == 2)
		fmf.Close();
	else
		arena_cam.Stop();

	printf("\nPress Enter to exit...\n");
	getchar();

	return 0;
}
