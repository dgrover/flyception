// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

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

	Flycam arena_cam;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;
	Error error;

	FileReader f;

	Tracker tkf;
	Daq ndq;

	int nframes = -1;

	if (argc == 2)
	{
		f.Open(argv[1]);
		f.ReadHeader();
		nframes = f.GetFrameCount();	
	}
	else
	{
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

		error = arena_cam.SetCameraParameters(384, 256, 512, 512);

		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			return -1;
		}

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

	//Mat prev, rigid_mat, dst;

	vector<vector<Point>> contours, flycontour(1);
	vector<Vec4i> hierarchy;

	Point2f p;

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
			findContours(fgmask, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			
			for (int i = 0; i < contours.size(); i++)
				flycontour[0].insert(end(flycontour[0]), begin(contours[i]), end(contours[i]));

			if (flycontour[0].size() > 20)
			{
				RotatedRect minEllipse = fitEllipse(Mat(flycontour[0]));
				//drawContours(fgmask, flycontour, 0, Scalar(255, 255, 255), CV_FILLED, 8);

				ellipse(cframe, minEllipse, Scalar(255, 0, 0), 1, 1);
				circle(cframe, minEllipse.center, 1, Scalar(255, 0, 0), CV_FILLED, 1);
				//printf("[%f %f] ", minEllipse.center.x, minEllipse.center.y);

				tkf.Predict();
				p = tkf.Correct(minEllipse.center);
			}
	
			circle(cframe, p, 1, Scalar(0, 255, 0), CV_FILLED, 1);
			//printf("[%f %f] ", p.x, p.y);

			cv::Mat uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); // [u v 1]
			uvPoint.at<double>(0, 0) = p.x;
			uvPoint.at<double>(1, 0) = p.y;

			cv::Mat tempMat, tempMat2;
			double s;

			tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
			tempMat2 = rotationMatrix.inv() * tvec;
			s = tempMat2.at<double>(2, 0); //height Zconst is zero
			s /= tempMat.at<double>(2, 0);

			pt = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
			printf("[%f %f %f]\n", pt.at<double>(0, 0), pt.at<double>(1, 0), pt.at<double>(2, 0));
			
			//cv::Mat backPt = 1 / s * cameraMatrix * (rotationMatrix * pt + tvec);
			//printf("[%f %f]\n", backPt.at<double>(0, 0), backPt.at<double>(1, 0));
			
			imshow("raw image", cframe);
			//imshow("FG mask", fgmask);

			//if (p.x + 30 < 512 && p.y + 30 < 512 && p.x - 30 >= 0 && p.y - 30 >= 0)
			//{
			//	Mat subImage(frame, cv::Rect(p.x - 30, p.y - 30, 60, 60));

			//	if (prev.empty())
			//		prev = subImage.clone();

			//	rigid_mat = estimateRigidTransform(prev, subImage, false);
			//
			//	if (!rigid_mat.empty())
			//	{
			//		warpAffine(subImage, dst, rigid_mat, subImage.size(), INTER_NEAREST | WARP_INVERSE_MAP, BORDER_CONSTANT);
			//		//Mat rotImage = rotate(dst, angle);
			//		imshow("sub image", dst);
			//	}

			//	prev = subImage.clone();

			//}

			flycontour[0].clear();

		}
		else
			pt = f.ReadFrame();		//Read coordinates from txt file

		//ndq.ConvertPtToVoltage(pt);
		//ndq.write();

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
