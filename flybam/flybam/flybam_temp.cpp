// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

void extractFlyROI(Mat img, RotatedRect rect)
{
	Mat M, rotated, cropped;
	
	// get angle and size from the bounding box
	float angle = rect.angle;
	Size rect_size = rect.size;
	
	if (rect.angle < -45.) 
	{
		angle += 90.0;
		swap(rect_size.width, rect_size.height);
	}
	
	// get the rotation matrix
	M = getRotationMatrix2D(rect.center, angle, 1.0);
	
	// perform the affine transformation
	warpAffine(img, rotated, M, img.size(), INTER_CUBIC);
	
	// crop the resulting image
	getRectSubPix(rotated, rect_size, rect.center, cropped);
}

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
	printf("[%f %f %f] ", pt.at<double>(0, 0), pt.at<double>(1, 0), pt.at<double>(2, 0));

	cv::Mat backPt = 1 / s * cameraMatrix * (rotationMatrix * pt + tvec);
	printf("[%f %f] ", backPt.at<double>(0, 0), backPt.at<double>(1, 0));

	return pt;

}

int _tmain(int argc, _TCHAR* argv[])
{
	string filename = "..\\..\\arena\\camera_projection_data.xml";

	Mat pt;

	Mat cameraMatrix, distCoeffs;
	Mat rvec(1, 3, cv::DataType<double>::type);
	Mat tvec(1, 3, cv::DataType<double>::type);
	Mat rotationMatrix(3, 3, cv::DataType<double>::type);

	FileReader f;

	Tracker tkf;
	
	int nframes = -1;
	
	int arena_image_width, arena_image_height;
	int fly_image_width, fly_image_height;

	f.Open(argv[1]);
	f.ReadHeader();
	f.GetImageSize(arena_image_width, arena_image_height);
	nframes = f.GetFrameCount();	
	

	FileStorage fs(filename, FileStorage::READ);

	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;
	fs["rvec"] >> rvec;
	fs["tvec"] >> tvec;
	fs["Rotation_Matrix"] >> rotationMatrix;

	fs.release();

	pt = cv::Mat::zeros(2, 1, cv::DataType<double>::type);

	Mat outer_mask = Mat::zeros(Size(arena_image_width, arena_image_height), CV_8UC1);
	circle(outer_mask, Point(arena_image_width/2, arena_image_height/2), 250, Scalar(255, 255, 255), CV_FILLED);
	
	Mat arena_frame, arena_mask;
	Mat fly_frame, fly_mask;

	int arena_thresh = 80;
	int fly_thresh = 95;

	Mat erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

	for (int imageCount = 0; imageCount != nframes; imageCount++)
	{
		fly_frame = f.ReadFrame(imageCount);
	
		fly_frame = rotateImage(fly_frame, 15);

		createTrackbar("Fly thresh", "fly image", &fly_thresh, 255);

		threshold(fly_frame, fly_mask, fly_thresh, 255, THRESH_TOZERO_INV);
		threshold(fly_mask, fly_mask, 5, 255, THRESH_BINARY);

		erode(fly_mask, fly_mask, erodeElement, Point(-1, -1), 1);
		dilate(fly_mask, fly_mask, dilateElement, Point(-1, -1), 3);

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

		if (fly_max_contour_index != -1)
		{
			RotatedRect fly_area = fitEllipse(Mat(fly_contours[fly_max_contour_index]));
			
			circle(fly_frame, fly_mc[fly_max_contour_index], 1, Scalar(255, 255, 255), CV_FILLED, 1);
			Mat fly_pt = refineFlyCenter(pt, fly_mc[fly_max_contour_index]);

			imshow("fly image", fly_frame);
			imshow("fly mask", fly_mask);
		}
		else
		{
			// if no fly detected, switch back to arena view to get coarse fly location and position update
			arena_frame = f.ReadFrame(imageCount);
		
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

				imshow("arena image", arena_frame);
				imshow("arena mask", arena_mask);
			}
		}

		waitKey(1);
		
		if ( GetAsyncKeyState(VK_ESCAPE) )
			break;
	}

	f.Close();

	printf("\nPress Enter to exit...\n");
	getchar();

	return 0;
}
