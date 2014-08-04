// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

struct mouse_pos { int x,y; };
struct mouse_pos mouse_info = {-1,-1};

void on_mouse(int event, int x, int y, int flags, void* param)
{
    mouse_info.x = x;
    mouse_info.y = y;
}

string GetFileExtension(const string& FileName)
{
    if(FileName.find_last_of(".") != string::npos)
        return FileName.substr(FileName.find_last_of(".")+1);
    return "";
}

int _tmain(int argc, _TCHAR* argv[])
{
  int ftype = 0;

  string fext;
  int nframes, success;

  CsvReader csv;
  FmfReader fmf;

  Mat img(512, 512, CV_8UC3);

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
      printf("File not specified. Switching to mouse control...\n");

      nframes = -1;

      namedWindow("mouse kalman");
      setMouseCallback("mouse kalman", on_mouse, 0);

      if (mouse_info.x < 0 || mouse_info.y < 0)
      {
          imshow("mouse kalman", img);
          waitKey(30);
          continue;
      }
  }

  Tracker tkf(mouse_info.x, mouse_info.y);

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
	Mat frame, fgmask;

	for (int imageCount = 0; imageCount != nframes; imageCount++)
	{

		if (ftype == 1)
		{
				success = fmf.ReadFrame(imageCount);
				frame = fmf.ConvertToCvMat();

				imshow("raw image", frame);
				mog_cpu(frame, fgmask, 0.01);
				imshow("FG mask", fgmask);

		}
		else if (ftype == 2)
		{
				success = csv.ReadLine();
				csv.ConvertPixelToVoltage(dataX, dataY);
		}
    else if (ftype == 0)
    {
        tkf.Predict(mouse_info.x, mouse_info.y);
        tkf.Correct();

        int ms = tkf.mmtv.size();
        int es = tkf.estv.size();

        if (ms > 1)
          line(img, tkf.mmtv[ms-1], tkf.mmtv[ms-2], Scalar(255,255,0), 1);

        if (es > 1)
          line(img, tkf.estv[ks-1], tkf.estv[ks-2], Scalar(0,255,0), 1);

        imshow( "mouse kalman", img );

        tkf.ConvertPixelToVoltage(dataX, dataY);

    }

		//printf("%f %f\n", dataX[0], dataY[0]);

		// DAQmx Write Code
		DAQmxWriteAnalogF64(taskHandleX,1,1,10.0,DAQmx_Val_GroupByChannel,dataX,NULL,NULL);
		DAQmxWriteAnalogF64(taskHandleY,1,1,10.0,DAQmx_Val_GroupByChannel,dataY,NULL,NULL);

		waitKey(1);

		if ( GetAsyncKeyState(VK_ESCAPE) )
			break;
	}

	if (ftype == 1)
		fmf.Close();
	else if (ftype == 2)
		csv.Close();

	printf("\nPress Enter to exit...\n");
	getchar();

	return 0;
}
