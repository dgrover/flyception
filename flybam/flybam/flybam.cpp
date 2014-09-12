// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

#define N 750

Mat backProject(Point2f p, Mat cameraMatrix, Mat rotationMatrix, Mat tvec)
{
	cv::Mat uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); // [u v 1]
	uvPoint.at<double>(0, 0) = p.x;
	uvPoint.at<double>(1, 0) = p.y;

	cv::Mat tempMat, tempMat2;
	double s;

	tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
	tempMat2 = rotationMatrix.inv() * tvec;
	s = -3.175 + tempMat2.at<double>(2, 0); //height Zconst is zero
	s /= tempMat.at<double>(2, 0);

	Mat pt = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
	//printf("[%f %f %f]\n", pt.at<double>(0, 0), pt.at<double>(1, 0), pt.at<double>(2, 0));

	//cv::Mat backPt = 1 / s * cameraMatrix * (rotationMatrix * pt + tvec);
	//printf("[%f %f]\n", backPt.at<double>(0, 0), backPt.at<double>(1, 0));

	return pt;

}

//Mat rotate(Mat src, double angle)
//{
//	Mat dst;
//	Point2f pt(src.cols / 2., src.rows / 2.);
//	Mat r = getRotationMatrix2D(pt, angle, 1.0);
//	warpAffine(src, dst, r, Size(src.cols, src.rows));
//	return dst;
//}

int _tmain(int argc, _TCHAR* argv[])
{
	string filename = "..\\..\\arena\\camera_projection_data.xml";

	Mat pt;
	Mat cameraMatrix, distCoeffs;
	Mat rvec(1, 3, cv::DataType<double>::type);
	Mat tvec(1, 3, cv::DataType<double>::type);
	Mat rotationMatrix(3, 3, cv::DataType<double>::type);

	Flycam arena_cam, fly_cam;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;
	FlyCapture2::Error error;

	FileReader f;

	Tracker tkf;
	Daq ndq;

	int nframes = -1;
	int imageWidth, imageHeight;

	if (argc == 2)
	{
		f.Open(argv[1]);
		f.ReadHeader();
		f.GetImageSize(imageWidth, imageHeight);
		nframes = f.GetFrameCount();	
	}
	else
	{
		error = busMgr.GetNumOfCameras(&numCameras);
		printf("Number of cameras detected: %u\n", numCameras);

		if (numCameras < 2)
		{
			printf("Insufficient number of cameras... exiting\n");
			return -1;
		}

		//Initialize arena camera
		error = busMgr.GetCameraFromIndex(0, &guid);
		error = arena_cam.Connect(guid);
		error = arena_cam.SetCameraParameters(512, 512);
		arena_cam.GetImageSize(imageWidth, imageHeight);
		error = arena_cam.Start();

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

		//Initialize fly camera
		error = busMgr.GetCameraFromIndex(1, &guid);
		error = fly_cam.Connect(guid);
		error = fly_cam.SetCameraParameters(512, 512);
		error = fly_cam.Start();

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
	
	FlyCapture2::Image fly_img, arena_img;
	Mat arena_frame, arena_bg, arena_mask;
	Mat fly_frame, fly_mask;

	vector<vector<Point>> arena_contours;
	vector<Vec4i> hierarchy;

	Point2f p;

	//background calculation for arena view
	printf("\nComputing background model... ");
	arena_bg = Mat::zeros(Size(imageWidth, imageHeight), CV_32FC1);

	for (int imageCount = 0; imageCount != N; imageCount++)
	{
		if (argc == 2)
			arena_frame = f.ReadFrame(imageCount);
		else
		{
			arena_img = arena_cam.GrabFrame();
			arena_frame = arena_cam.convertImagetoMat(arena_img);
		}

		accumulate(arena_frame, arena_bg);
	}

	arena_bg = arena_bg / N;
	arena_bg.convertTo(arena_bg, CV_8UC1);

	printf("Done\n");

	for (int imageCount = 0; imageCount != nframes; imageCount++)
	{
		p = tkf.Predict();

		pt = backProject(p, cameraMatrix, rotationMatrix, tvec);

		ndq.ConvertPtToVoltage(pt);
		ndq.write();

		fly_img = fly_cam.GrabFrame();
		fly_frame = fly_cam.convertImagetoMat(fly_img);
		
		// fly feature detection and position update
		imshow("fly image", fly_frame);

		// if no fly detected, switch back to arena view to get coarse fly location and position update
		
		if (argc == 2)
			arena_frame = f.ReadFrame(imageCount);
		else
		{
			arena_img = arena_cam.GrabFrame();
			arena_frame = arena_cam.convertImagetoMat(arena_img);
		}

		
		// detect fly in arena view
		absdiff(arena_frame, arena_bg, arena_mask);
		threshold(arena_mask, arena_mask, 50, 255, THRESH_BINARY);
		
		imshow("arena mask", arena_mask);

		findContours(arena_mask, arena_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		vector<RotatedRect> minEllipse(arena_contours.size());

		for (int i = 0; i < arena_contours.size(); i++)
		{
			drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
			if (arena_contours[i].size() > 5)
			{
				minEllipse[i] = fitEllipse(Mat(arena_contours[i]));
				ellipse(arena_frame, minEllipse[i], Scalar(255, 255, 255), 1, 1);
				p = tkf.Correct(minEllipse[i].center);
			}
		}

		circle(arena_frame, p, 1, Scalar(255, 255, 255), FILLED, 1);
		//printf("[%f %f] ", p.x, p.y);

		imshow("arena image", arena_frame);
				
		waitKey(1);

		if ( GetAsyncKeyState(VK_ESCAPE) )
			break;
	}

	if (argc == 2)
		f.Close();
	else
	{
		arena_cam.Stop();
		fly_cam.Stop();
	}

	printf("\nPress Enter to exit...\n");
	getchar();

	return 0;
}
