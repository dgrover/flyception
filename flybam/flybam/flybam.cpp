// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

int _tmain(int argc, _TCHAR* argv[])
{
	Point2f p;

	string filename = "..\\..\\arena\\camera_projection_data.xml";

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

	Tracker tkf;
	Daq ndq;

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

	//configure and start NIDAQ
	ndq.configure();
	ndq.start();
	
	BackgroundSubtractorMOG mog_cpu;
	mog_cpu.set("nmixtures", 3);
	Mat frame, fgmask, cframe;

	vector<vector<Point> > contours, flycontour(1);
	vector<Vec4i> hierarchy;

	for (int imageCount = 0; imageCount != nframes; imageCount++)
	{
		if (argc == 2)
			frame = f.ReadFrame(imageCount);
		else
		 	frame = arena_cam.GrabFrame();

		cvtColor(frame, cframe, CV_GRAY2RGB);
		
		if (!frame.empty())
		{
			mog_cpu(frame, fgmask, 0.01);

			findContours(fgmask, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE);
			
			RotatedRect minEllipse;
			
			for (int i = 0; i < contours.size(); i++)
				flycontour[0].insert(end(flycontour[0]), begin(contours[i]), end(contours[i]));

			if (flycontour[0].size() > 25)
			{
					minEllipse = fitEllipse(Mat(flycontour[0]));
					//drawContours(fgmask, contours, i, Scalar(255, 255, 255), CV_FILLED, 8, hierarchy);
					drawContours(fgmask, flycontour, 0, Scalar(255, 255, 255), CV_FILLED, 8);

					circle(cframe, minEllipse.center, 1, Scalar(255, 0, 0), CV_FILLED, 1);
					ellipse(cframe, minEllipse, Scalar(255, 0, 0), 1, 1);

					//printf("[%f %f] ", minEllipse.center.x, minEllipse.center.y);
					
					cv::Mat uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); // [u v 1]
					uvPoint.at<double>(0, 0) = minEllipse.center.x;
					uvPoint.at<double>(1, 0) = minEllipse.center.y;

					cv::Mat tempMat, tempMat2;
					double s;

					tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
					tempMat2 = rotationMatrix.inv() * tvec;
					s = tempMat2.at<double>(2, 0); //height Zconst is zero
					s /= tempMat.at<double>(2, 0);

					//printf("%f ", s);
														
					cv::Mat pt = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
					//printf("[%f %f %f]\n", pt.at<double>(0, 0), pt.at<double>(1, 0), pt.at<double>(2, 0));

					tkf.Predict(pt.at<double>(0, 0), pt.at<double>(1, 0));
					tkf.Correct();
					tkf.GetTrackedPoint(p);

					//p.x = pt.at<double>(0, 0);
					//p.y = pt.at<double>(1, 0);

					//cv::Mat backPt = 1 / s * cameraMatrix * (rotationMatrix * pt + tvec);
					//printf("[%f %f]\n", backPt.at<double>(0, 0), backPt.at<double>(1, 0));
							
					//project center point back to image coordinate system
					//circle(cframe, cvPoint(backPt.at<double>(0, 0), backPt.at<double>(1, 0)), 1, Scalar(0, 255, 0), CV_FILLED, 1);
			}

			imshow("raw image", cframe);
			imshow("FG mask", fgmask);

			flycontour[0].clear();

		}
		else
			p = f.ReadFrame();		//Read coordinates from txt file

		ndq.ConvertPixelToVoltage(p);
		ndq.write();

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
