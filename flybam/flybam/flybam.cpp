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

	Mat cameraMatrix, distCoeffs;

	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point3f> objectPoints;

	cv::Mat rvec(1, 3, cv::DataType<double>::type);
	cv::Mat tvec(1, 3, cv::DataType<double>::type);
	cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);

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

		FileStorage fs("..\\..\\calib\\images\\out_camera_data.xml", FileStorage::READ);

		fs["Camera_Matrix"] >> cameraMatrix;
		fs["Distortion_Coefficients"] >> distCoeffs;

		fs.release();

		//x,y coordinates in camera image
		imagePoints.push_back(cv::Point2f(258., 258.));
		imagePoints.push_back(cv::Point2f(208., 258.));
		imagePoints.push_back(cv::Point2f(313., 259.));
		imagePoints.push_back(cv::Point2f(258., 208.));
		imagePoints.push_back(cv::Point2f(258., 306.));

		//object points (measured in millimeters because calibration is done in mm)
		objectPoints.push_back(cv::Point3f(0., 0., 0.));
		objectPoints.push_back(cv::Point3f(-50., 0., 0.));
		objectPoints.push_back(cv::Point3f(50., 0., 0.));
		objectPoints.push_back(cv::Point3f(0., 50., 0.));
		objectPoints.push_back(cv::Point3f(0., -50., 0.));

		cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
		cv::Rodrigues(rvec, rotationMatrix);

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

							
							cv::Mat uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
							uvPoint.at<double>(0, 0) = minEllipse[i].center.x; //got this point using mouse callback
							uvPoint.at<double>(1, 0) = minEllipse[i].center.y;

							cv::Mat tempMat, tempMat2;
							double s;

							tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
							tempMat2 = rotationMatrix.inv() * tvec;
							s = tempMat2.at<double>(2, 0); //height Zconst is zero
							s /= tempMat.at<double>(2, 0);
							//std::cout << "P = " << rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec) << std::endl;
							
							cv::Mat pt = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
							
							//tkf.Predict(minEllipse[i].center.x, minEllipse[i].center.y);
							//tkf.Correct();
							//tkf.GetTrackedPoint(p);

							printf("%f %f \n", pt.at<double>(0, 0), pt.at<double>(1, 0));

						}
					}

					imshow("raw image", frame);
					imshow("FG mask", fgmask);
		}
		else
			p = fmf.ReadFrame();		//Read coordinates from txt file

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
		fmf.Close();
	else
		arena_cam.Stop();

	printf("\nPress Enter to exit...\n");
	getchar();

	return 0;
}
