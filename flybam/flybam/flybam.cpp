// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "fmfreader.h"
#include "csvreader.h"
#include <NIDAQmx.h>

using namespace std;
using namespace cv;

string GetFileExtension(const string& FileName)
{
    if(FileName.find_last_of(".") != string::npos)
        return FileName.substr(FileName.find_last_of(".")+1);
    return "";
}

int _tmain(int argc, _TCHAR* argv[])
{
	if (argc != 2)
	{
		printf("Filename not specified... exiting\n");
		return -1;
	}

	string fname = string(argv[1]);
	string fext = GetFileExtension(fname);

	int ftype;

	if (fext == "fmf")
		ftype = 1;
	else if (fext == "txt")
		ftype = 2;

	CsvReader csv;
	FmfReader fmf;

	TaskHandle	taskHandleX=0;
	TaskHandle	taskHandleY=0;

	float64     data[2] = {0.0, 0.0};

	// DAQmx Configure Code
	DAQmxCreateTask("",&taskHandleX);
	DAQmxCreateTask("",&taskHandleY);
	DAQmxCreateAOVoltageChan(taskHandleX,"Dev1/ao0","",-10.0,10.0,DAQmx_Val_Volts,"");
	DAQmxCreateAOVoltageChan(taskHandleY,"Dev1/ao1","",-10.0,10.0,DAQmx_Val_Volts,"");

	// DAQmx Start Code
	DAQmxStartTask(taskHandleX);
	DAQmxStartTask(taskHandleY);

	BackgroundSubtractorMOG mog_cpu;

	Mat frame, fgmask;

	int nframes, success;

	if (ftype == 1)
	{
			success = fmf.Open(argv[1]);
			success = fmf.ReadHeader();
			nframes = fmf.GetFrameCount();
	}
	else
	{
			success = csv.Open(argv[1]);
			nframes = csv.GetFrameCount();
	}

	for (int imageCount = 0; imageCount < nframes; imageCount++)
	{

		if (ftype == 1)
		{
				success = fmf.ReadFrame(imageCount);
				frame = fmf.ConvertToCvMat();

				imshow("raw image", frame);

				//mog_cpu(frame, fgmask, 0.01);

				//imshow("FG mask", fgmask);

		}
		else
		{
				success = csv.ReadLine();
				success = csv.ConvertPixelToVoltage(&data);
		}

		// DAQmx Write Code
		DAQmxWriteAnalogF64(taskHandleX,1,1,10.0,DAQmx_Val_GroupByChannel,data[0],NULL,NULL);
		DAQmxWriteAnalogF64(taskHandleY,1,1,10.0,DAQmx_Val_GroupByChannel,data[1],NULL,NULL);

		char key = waitKey(1);

		if (key == 27)		//press [ESC] to exit
			break;
	}

	printf("\nDone! Press Enter to exit...\n");
	getchar();

	return 0;
}
