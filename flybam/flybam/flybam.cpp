// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

void ConvertPixelToVoltage(Point2f p, int imageWidth, int imageHeight, int maxVoltage, float64 dataX[], float64 dataY[])
{
	dataX[0] = (p.x / imageWidth * maxVoltage) - maxVoltage / 2;
	dataY[0] = (p.y / imageHeight * maxVoltage) - maxVoltage / 2;
}

int _tmain(int argc, _TCHAR* argv[])
{
	Point2f p;

	string filename = "..\\..\\images\\camera_projection_data.xml";

	Mat cameraMatrix, distCoeffs;
	Mat rvec(1, 3, cv::DataType<double>::type);
	Mat tvec(1, 3, cv::DataType<double>::type);
	Mat rotationMatrix(3, 3, cv::DataType<double>::type);

	Flycam arena_cam;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;

	Error error;

	int nframes, imageWidth, imageHeight, success;

	FileReader f;

	if (argc == 2)
	{
		success = f.Open(argv[1]);
		success = f.ReadHeader();
		nframes = f.GetFrameCount();
		f.GetImageSize(imageWidth, imageHeight);
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

	FileStorage fs(filename, FileStorage::READ);

	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;
	fs["rvec"] >> rvec;
	fs["tvec"] >> tvec;
	fs["Rotation_Matrix"] >> rotationMatrix;

	fs.release();

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
	Mat frame, fgmask, cframe;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	for (int imageCount = 0; imageCount != nframes; imageCount++)
	{
		if (argc == 2)
			frame = f.ReadFrame(imageCount);
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
							
							cvtColor(frame, cframe, CV_GRAY2RGB);
							
							circle(cframe, minEllipse[i].center, 1, Scalar(255, 0, 0), CV_FILLED, 1);
							ellipse(frame, minEllipse[i], Scalar(255, 0, 0), 1, 1);

							printf("[%f %f] ", minEllipse[i].center.x, minEllipse[i].center.y);

							cv::Mat uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); // [u v 1]
							uvPoint.at<double>(0, 0) = minEllipse[i].center.x;
							uvPoint.at<double>(1, 0) = minEllipse[i].center.y;

							cv::Mat tempMat, tempMat2;
							double s;

							tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
							tempMat2 = rotationMatrix.inv() * tvec;
							s = tempMat2.at<double>(2, 0); //height Zconst is zero
							s /= tempMat.at<double>(2, 0);

							printf("%f ", s);
														
							cv::Mat pt = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
							printf("[%f %f %f] ", pt.at<double>(0, 0), pt.at<double>(1, 0), pt.at<double>(2, 0));

							cv::Mat backPt = 1 / s * cameraMatrix * (rotationMatrix * pt + tvec);
							printf("[%f %f]\n", backPt.at<double>(0, 0), backPt.at<double>(1, 0));
							
							//project center point back to image coordinate system
							circle(frame, cvPoint(backPt.at<double>(0, 0), backPt.at<double>(1, 0)), 1, Scalar(0, 255, 0), CV_FILLED, 1);

							//tkf.Predict(minEllipse[i].center.x, minEllipse[i].center.y);
							//tkf.Correct();
							//tkf.GetTrackedPoint(p);

						}
					}

					imshow("raw image", frame);
					imshow("FG mask", fgmask);
		}
		else
			p = f.ReadFrame();		//Read coordinates from txt file

		//ConvertPixelToVoltage(p, imageWidth, imageHeight, 5.0, dataX, dataY);
		//printf("%f %f\n", dataX[0], dataY[0]);

		// DAQmx Write Code
		//DAQmxWriteAnalogF64(taskHandleX,1,1,10.0,DAQmx_Val_GroupByChannel,dataX,NULL,NULL);
		//DAQmxWriteAnalogF64(taskHandleY,1,1,10.0,DAQmx_Val_GroupByChannel,dataY,NULL,NULL);

		waitKey(1);

		if ( GetAsyncKeyState(VK_ESCAPE) )
			break;
	}

	if (argc == 2)
		f.Close();
	else
		arena_cam.Stop();

	printf("\nPress Enter to exit...\n");
	getchar();

	return 0;
}
