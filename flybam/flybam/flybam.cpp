// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

string GetFileExtension(const string& FileName)
{
    if(FileName.find_last_of(".") != string::npos)
        return FileName.substr(FileName.find_last_of(".")+1);
    return "";
}

int _tmain(int argc, _TCHAR* argv[])
{
	
	Flycam wcam;

	int ftype = 0;
	bool track = false;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;

	Error error;
	
	string fext;
	int nframes, success;

	CsvReader csv;
	FmfReader fmf;

	//Mat img(512, 512, CV_8UC3, Scalar(0,0,0));

	if (argc == 2)
	{
		fext = GetFileExtension(string(argv[1]));

		if (fext == "fmf")
		{
			ftype = 1;
			success = fmf.Open(argv[1]);
			success = fmf.ReadHeader();
			nframes = fmf.GetFrameCount();
		}
		else if (fext == "txt")
		{
			ftype = 2;
			success = csv.Open(argv[1]);
			nframes = csv.GetFrameCount();
		}
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

		error = busMgr.GetCameraFromIndex(0, &guid);

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

		error = wcam.Connect(guid);

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

		error = wcam.SetCameraParameters();

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

		// Start wide-field camera
		error = wcam.Start();
		
		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

		printf("Streaming. Press [SPACE] to start tracking\n\n");
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
		if (ftype != 2)
		{
			if (ftype == 1)
			{
				success = fmf.ReadFrame(imageCount);
				frame = fmf.ConvertToCvMat();

			}
			else if (ftype == 0)
			{
				error = wcam.GrabFrame();
				
				if (error != PGRERROR_OK)
				{
					error.PrintErrorTrace();
					return -1;
				}

				frame = wcam.ConvertImage2Mat();
			}

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

					tkf.ConvertPixelToVoltage(512, 512, 3.0, dataX, dataY);
				}
			}
								
			imshow("raw image", frame);
			imshow("FG mask", fgmask);

		}
		else if (ftype == 2)
		{
				success = csv.ReadLine();
				csv.ConvertPixelToVoltage(512, 512, 3.0, dataX, dataY);
		}

		//printf("%f %f\n", dataX[0], dataY[0]);

		// DAQmx Write Code
		DAQmxWriteAnalogF64(taskHandleX,1,1,10.0,DAQmx_Val_GroupByChannel,dataX,NULL,NULL);
		DAQmxWriteAnalogF64(taskHandleY,1,1,10.0,DAQmx_Val_GroupByChannel,dataY,NULL,NULL);

		waitKey(1);

		if ( GetAsyncKeyState(VK_ESCAPE) )
			break;
	}

	if (ftype == 0)
		wcam.Stop();
	else if (ftype == 1)
		fmf.Close();
	else if (ftype == 2)
		csv.Close();

	printf("\nPress Enter to exit...\n");
	getchar();

	return 0;
}
