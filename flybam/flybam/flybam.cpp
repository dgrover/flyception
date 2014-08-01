// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "fmfreader.h"
#include <NIDAQmx.h>

using namespace cv;

int _tmain(int argc, _TCHAR* argv[])
{
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

	if (argc != 2)
	{
		printf("Filename not specified... exiting\n");
		return -1;
	}

	FmfReader fin;

	BackgroundSubtractorMOG mog_cpu;
	
	Mat frame, fgmask;
	
	int success;

	success = fin.Open(argv[1]);
	success = fin.ReadHeader();
	
	for (int imageCount = 0; ; imageCount++)
	{
		success = fin.ReadFrame(imageCount);
			
		if (success)
			frame = fin.ConvertToCvMat();
		else
			break;			
		
		imshow("raw image", frame);

		mog_cpu(frame, fgmask, 0.01);
		
		imshow("FG mask", fgmask);

		// DAQmx Write Code
		DAQmxWriteAnalogF64(taskHandleX,1,1,10.0,DAQmx_Val_GroupByChannel,dataX,NULL,NULL);
		DAQmxWriteAnalogF64(taskHandleY,1,1,10.0,DAQmx_Val_GroupByChannel,dataY,NULL,NULL);
		
		char key = waitKey(1);
		
		if (key == 27)		//press [ESC] to exit
			break;
	}

	fin.Close();
	
	printf("\nDone! Press Enter to exit...\n");
	getchar();

	return 0;
}

