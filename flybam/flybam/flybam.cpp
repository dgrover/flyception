// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

//Mat extractFlyROI(Mat img, RotatedRect rect)
//{
//	Mat M, rotated, cropped;
//	
//	// get angle and size from the bounding box
//	float angle = rect.angle;
//	Size rect_size = rect.size;
//	
//	if (rect.angle < -45.) 
//	{
//		angle += 90.0;
//		swap(rect_size.width, rect_size.height);
//	}
//	
//	// get the rotation matrix
//	M = getRotationMatrix2D(rect.center, angle, 1.0);
//	
//	// perform the affine transformation
//	warpAffine(img, rotated, M, img.size(), INTER_CUBIC);
//	
//	// crop the resulting image
//	getRectSubPix(rotated, rect_size, rect.center, cropped);
//
//	return cropped;
//}

Mat rotateImage(Mat src, double angle)
{
	Mat dst;
	Point2f pt(src.cols / 2., src.rows / 2.);
	Mat r = getRotationMatrix2D(pt, angle, 1.0);
	warpAffine(src, dst, r, Size(src.cols, src.rows));
	return dst;
}

Mat refineFlyCenter(Mat pt, Point2f p)
{
	float scale = 2.5 / 141.667914;
	//float angle = 15 * 180 / PI;

	p.x -= 256;
	p.y -= 256;

	cv::Mat newPt = cv::Mat::ones(2, 1, cv::DataType<double>::type);

	newPt.at<double>(0, 0) = pt.at<double>(0, 0) + ((double)p.x*scale);
	newPt.at<double>(1, 0) = pt.at<double>(1, 0) + ((double)p.y*scale);

	//printf("[%f %f]\n", pt.at<double>(0, 0), pt.at<double>(1, 0));
	//printf("[%f %f]\n", newPt.at<double>(0, 0), newPt.at<double>(1, 0));

	return newPt;
}

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

vector<Point2f> project3d2d(Mat pt, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
{
	vector<Point3f> p3d;
	vector<Point2f> p2d;

	p3d.push_back(Point3f((float)pt.at<double>(0, 0), (float)pt.at<double>(1, 0), -3.175));
	projectPoints(p3d, rvec, tvec, cameraMatrix, distCoeffs, p2d);

	return p2d;
}

RotatedRect createArenaMask(Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
{
	Point2f center(0, 0);
	double radius = 25; //in mm

	vector<Point3f> c3d;
	vector<Point2f> c2d;

	RotatedRect circleMask;

	for (double angle = 0; angle <= 2 * PI; angle += 0.001)//You are using radians so you will have to increase by a very small amount
		c3d.push_back(Point3f(center.x + radius*cos(angle), center.y + radius*sin(angle), -3.175));

	projectPoints(c3d, rvec, tvec, cameraMatrix, distCoeffs, c2d);

	circleMask = fitEllipse(c2d);

	return circleMask;
}

int _tmain(int argc, _TCHAR* argv[])
{
	string filename = "..\\..\\arena\\camera_projection_data.xml";

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
	
	int arena_image_width, arena_image_height;
	int fly_image_width, fly_image_height;

	if (argc == 2)
	{
		f.Open(argv[1]);
		f.ReadHeader();
		f.GetImageSize(arena_image_width, arena_image_height);
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
		arena_cam.GetImageSize(arena_image_width, arena_image_height);
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
		fly_cam.GetImageSize(fly_image_width, fly_image_height);
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

	Mat outer_mask = Mat::zeros(Size(arena_image_width, arena_image_height), CV_8UC1);
	RotatedRect arenaMask = createArenaMask(cameraMatrix, distCoeffs, rvec, tvec);
	ellipse(outer_mask, arenaMask, Scalar(255, 255, 255), CV_FILLED);
	
	FlyCapture2::Image fly_img, arena_img;

	Mat arena_frame, arena_mask;
	Mat fly_frame, fly_mask;

	int arena_thresh = 70;
	int fly_thresh = 75;

	Mat erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

	bool flyview_track = false;

	for (int imageCount = 0; imageCount != nframes; imageCount++)
	{
		Mat pt = tkf.Predict();

		ndq.ConvertPtToVoltage(pt);
		ndq.write();

		waitKey(1);
		
		if (argc == 2)
			fly_frame = f.ReadFrame(imageCount);
		else
		{
			fly_img = fly_cam.GrabFrame();
			fly_frame = fly_cam.convertImagetoMat(fly_img);
		}

		createTrackbar("Fly thresh", "fly image", &fly_thresh, 255);

		threshold(fly_frame, fly_mask, fly_thresh, 255, THRESH_BINARY_INV);
		fly_frame = rotateImage(fly_frame, 15);
		fly_mask = rotateImage(fly_mask, 15);

		//erode(fly_mask, fly_mask, erodeElement, Point(-1, -1), 1);
		//dilate(fly_mask, fly_mask, dilateElement, Point(-1, -1), 1);

		vector<vector<Point>> fly_contours;
		vector<Vec4i> fly_hierarchy;

		findContours(fly_mask, fly_contours, fly_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

		/// Get the moments and mass centers
		vector<Moments> fly_mu(fly_contours.size());
		vector<Point2f> fly_mc(fly_contours.size());
		for (int i = 0; i < fly_contours.size(); i++)
		{
			fly_mu[i] = moments(fly_contours[i], false);
			fly_mc[i] = Point2f(fly_mu[i].m10 / fly_mu[i].m00, fly_mu[i].m01 / fly_mu[i].m00);
		}

		double fly_max_area = 0;
		int fly_max_contour_index = -1;

		for (int i = 0; i < fly_contours.size(); i++)
		{
			drawContours(fly_mask, fly_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
			if (fly_mu[i].m00 > fly_max_area)
			{
				fly_max_area = fly_mu[i].m00;
				fly_max_contour_index = i;                //Store the index of largest contour
			}
		}

		if (flyview_track == true && fly_max_contour_index != -1)
		{
			circle(fly_frame, fly_mc[fly_max_contour_index], 1, Scalar(255, 255, 255), CV_FILLED, 1);
			Mat fly_pt = refineFlyCenter(pt, fly_mc[fly_max_contour_index]);
			
			pt = tkf.Correct(fly_pt);

			imshow("fly image", fly_frame);
			//imshow("fly mask", fly_mask);
		}
		else
		{
			// if no fly detected, switch back to arena view to get coarse fly location and position update
			if (argc == 2)
				arena_frame = f.ReadFrame(imageCount);
			else
			{
				arena_img = arena_cam.GrabFrame();
				arena_frame = arena_cam.convertImagetoMat(arena_img);
			}

			createTrackbar("Arena thresh", "arena image", &arena_thresh, 255);

			threshold(arena_frame, arena_mask, arena_thresh, 255, THRESH_BINARY_INV);
			arena_mask &= outer_mask;

			vector<vector<Point>> arena_contours;
			vector<Vec4i> arena_hierarchy;

			findContours(arena_mask, arena_contours, arena_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

			/// Get the moments and mass centers
			vector<Moments> arena_mu(arena_contours.size());
			vector<Point2f> arena_mc(arena_contours.size());
			for (int i = 0; i < arena_contours.size(); i++)
			{
				arena_mu[i] = moments(arena_contours[i], false);
				arena_mc[i] = Point2f(arena_mu[i].m10 / arena_mu[i].m00, arena_mu[i].m01 / arena_mu[i].m00);
			}

			double arena_max_area = 0;
			int arena_max_contour_index = -1;

			for (int i = 0; i < arena_contours.size(); i++)
			{
				drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
				if (arena_mu[i].m00 > arena_max_area)
				{
					arena_max_area = arena_mu[i].m00;
					arena_max_contour_index = i;                //Store the index of largest contour
				}
			}

			if (arena_max_contour_index != -1)
			{
				circle(arena_frame, arena_mc[arena_max_contour_index], 1, Scalar(255, 255, 255), CV_FILLED, 1);
				Mat arena_pt = backProject(arena_mc[arena_max_contour_index], cameraMatrix, rotationMatrix, tvec);

				pt = tkf.Correct(arena_pt);
			}

			imshow("arena image", arena_frame);
			//imshow("arena mask", arena_mask);
		}

		//waitKey(1);
		
		if (GetAsyncKeyState(VK_SPACE))
			flyview_track = true;

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
