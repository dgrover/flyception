// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

#define BASE_HEIGHT 3.175
#define SCALE 2.5/123.006073

#define NFLIES 1

bool stream = true;
bool flyview_track = false;
bool flyview_record = false;

queue <Mat> flyDispStream;
queue <Image> flyImageStream;
queue <TimeStamp> flyTimeStamps;

Mat refineFlyCenter(Mat pt, Point2f p, int image_width, int image_height)
{
	Point2f temp;

	//rotate fly center point by 15 degrees due to the tilt of the galvo x-mirror
	temp.x = (cos(-15 * PI / 180)*(p.x - image_width / 2) - sin(-15 * PI / 180)*(p.y - image_height / 2));
	temp.y = (sin(-15 * PI / 180)*(p.x - image_width / 2) + cos(-15 * PI / 180)*(p.y - image_height / 2));

	cv::Mat newPt = cv::Mat::ones(2, 1, cv::DataType<double>::type);

	newPt.at<double>(0, 0) = pt.at<double>(0, 0) + ((double)temp.x * SCALE);
	newPt.at<double>(1, 0) = pt.at<double>(1, 0) + ((double)temp.y * SCALE);

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
	s = BASE_HEIGHT + tempMat2.at<double>(2, 0); //height Zconst is zero
	s /= tempMat.at<double>(2, 0);

	Mat pt = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
	//printf("[%f %f %f]\n", pt.at<double>(0, 0), pt.at<double>(1, 0), pt.at<double>(2, 0));

	//cv::Mat backPt = 1 / s * cameraMatrix * (rotationMatrix * pt + tvec);
	//printf("[%f %f]\n", backPt.at<double>(0, 0), backPt.at<double>(1, 0));

	return pt;

}

//vector<Point2f> project3d2d(Mat pt, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
//{
//	vector<Point3f> p3d;
//	vector<Point2f> p2d;
//
//	p3d.push_back(Point3f((float)pt.at<double>(0, 0), (float)pt.at<double>(1, 0), BASE_HEIGHT));
//	projectPoints(p3d, rvec, tvec, cameraMatrix, distCoeffs, p2d);
//
//	return p2d;
//}

RotatedRect createArenaMask(Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
{
	Point2f center(0, 0);
	double radius = 20; //in mm

	vector<Point3f> c3d;
	vector<Point2f> c2d;

	RotatedRect circleMask;

	for (double angle = 0; angle <= 2 * PI; angle += 0.001) //You are using radians so you will have to increase by a very small amount
		c3d.push_back(Point3f(center.x + radius*cos(angle), center.y + radius*sin(angle), BASE_HEIGHT));

	projectPoints(c3d, rvec, tvec, cameraMatrix, distCoeffs, c2d);

	circleMask = fitEllipse(c2d);

	return circleMask;
}

double dist(Mat p1, Mat p2)
{
	double dx = (p2.at<double>(0, 0) - p1.at<double>(0, 0));
	double dy = (p2.at<double>(1, 0) - p1.at<double>(1, 0));
	return(sqrt(dx*dx + dy*dy));
}


int findClosestPoint(Mat pt, vector<Mat> nbor)
{
	int fly_index = 0;
	if (nbor.size() == 1)
		return fly_index;
	else
	{
		double fly_dist = dist(pt, nbor[0]);
		
		for (int i = 1; i < nbor.size(); i++)
		{
			double res = dist(pt, nbor[i]);
			if (res < fly_dist)
			{
				fly_dist = res;
				fly_index = i;                //Store the index of nearest point
			}
		}

		return fly_index;
	}
}

//int sign(int v)
//{
//	return v > 0 ? 1 : -1;
//}

int _tmain(int argc, _TCHAR* argv[])
{
	string filename = "..\\..\\arena\\camera_projection_data.xml";

	Mat cameraMatrix, distCoeffs;
	Mat rvec(1, 3, cv::DataType<double>::type);
	Mat tvec(1, 3, cv::DataType<double>::type);
	Mat rotationMatrix(3, 3, cv::DataType<double>::type);

	PGRcam arena_cam, fly_cam;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;
	FlyCapture2::Error error;

	FmfReader fin;
	FmfWriter fout;

	vector<Tracker> tkf(NFLIES);
	vector<Mat> pt(NFLIES);

	Daq ndq;

	int arena_image_width = 512, arena_image_height = 512;
	int fly_image_width = 256, fly_image_height = 256;

	//fin.Open(argv[1]);
	//fin.ReadHeader();
	//fin.GetImageSize(arena_image_width, arena_image_height);
	//nframes = fin.GetFrameCount();	
	
	error = busMgr.GetNumOfCameras(&numCameras);
	//printf("Number of cameras detected: %u\n", numCameras);

	if (numCameras < 2)
	{
		printf("Insufficient number of cameras... exiting\n");
		return -1;
	}

	printf("Initializing arena view camera ");
	//Initialize arena camera
	error = busMgr.GetCameraFromIndex(0, &guid);
	error = arena_cam.Connect(guid);
	error = arena_cam.SetCameraParameters(arena_image_width, arena_image_height);
	//arena_cam.GetImageSize(arena_image_width, arena_image_height);
	error = arena_cam.Start();

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}
	printf("[OK]\n");

	printf("Initializing fly view camera ");
	//Initialize fly camera
	error = busMgr.GetCameraFromIndex(1, &guid);
	error = fly_cam.Connect(guid);
	error = fly_cam.SetCameraParameters(fly_image_width, fly_image_height);
	//fly_cam.GetImageSize(fly_image_width, fly_image_height);
	error = fly_cam.Start();
	
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}
	printf("[OK]\n\n");

	fout.Open();
	fout.InitHeader(fly_image_width, fly_image_height);
	fout.WriteHeader();

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
	FlyCapture2::TimeStamp fly_stamp;

	Mat arena_frame, arena_mask;
	Mat fly_frame, fly_mask_min, fly_mask_max;

	int arena_thresh = 75;
	int fly_min = 75;
	int fly_max = 120;
	int laser_pos = 0;

	Mat erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	Mat dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

	#pragma omp parallel sections num_threads(3)
	{
		#pragma omp section
		{
			while (true)
			{
				for (int i = 0; i < NFLIES; i++)
					pt[i] = tkf[i].Predict();

				ndq.ConvertPtToVoltage(pt[0]);
				ndq.write();

				//fly_frame = fin.ReadFrame(imageCount);
				fly_img = fly_cam.GrabFrame();
				fly_stamp = fly_cam.GetTimeStamp();
				fly_frame = fly_cam.convertImagetoMat(fly_img);

				threshold(fly_frame, fly_mask_min, fly_min, 255, THRESH_BINARY_INV);
				threshold(fly_frame, fly_mask_max, fly_max, 255, THRESH_BINARY_INV);
				
				erode(fly_mask_min, fly_mask_min, erodeElement, Point(-1, -1), 1);
				dilate(fly_mask_min, fly_mask_min, dilateElement, Point(-1, -1), 1);

				erode(fly_mask_max, fly_mask_max, erodeElement, Point(-1, -1), 1);
				dilate(fly_mask_max, fly_mask_max, dilateElement, Point(-1, -1), 1);

				if (flyview_track)
				{
					vector<vector<Point>> fly_contours_min, fly_contours_max;
					vector<Vec4i> fly_hierarchy_min, fly_hierarchy_max;

					findContours(fly_mask_min, fly_contours_min, fly_hierarchy_min, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
					findContours(fly_mask_max, fly_contours_max, fly_hierarchy_max, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					/// Get the moments and mass centers
					vector<Moments> fly_mu_min(fly_contours_min.size());
					vector<Point2f> fly_mc_min(fly_contours_min.size());

					vector<Mat> fly_pt_min;

					for (int i = 0; i < fly_contours_min.size(); i++)
					{
						drawContours(fly_mask_min, fly_contours_min, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());

						fly_mu_min[i] = moments(fly_contours_min[i], false);
						fly_mc_min[i] = Point2f(fly_mu_min[i].m10 / fly_mu_min[i].m00, fly_mu_min[i].m01 / fly_mu_min[i].m00);

						fly_pt_min.push_back(refineFlyCenter(pt[0], fly_mc_min[i], fly_image_width, fly_image_height));
					}

					/// Get the moments and mass centers
					vector<Moments> fly_mu_max(fly_contours_max.size());
					vector<Point2f> fly_mc_max(fly_contours_max.size());

					vector<Mat> fly_pt_max;

					for (int i = 0; i < fly_contours_max.size(); i++)
					{
						drawContours(fly_mask_max, fly_contours_max, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());

						fly_mu_max[i] = moments(fly_contours_max[i], false);
						fly_mc_max[i] = Point2f(fly_mu_max[i].m10 / fly_mu_max[i].m00, fly_mu_max[i].m01 / fly_mu_max[i].m00);

						fly_pt_max.push_back(refineFlyCenter(pt[0], fly_mc_max[i], fly_image_width, fly_image_height));
					}

					if ((fly_pt_min.size() > 0) && (fly_pt_max.size() > 0))
					{
						int j = findClosestPoint(pt[0], fly_pt_min);
						circle(fly_frame, fly_mc_min[j], 1, Scalar(255, 255, 255), CV_FILLED, 1);
						
						int k = findClosestPoint(fly_pt_min[j], fly_pt_max);

						RotatedRect flyEllipse = fitEllipse(Mat(fly_contours_min[j]));
						
						double turn = flyEllipse.angle - 90;
						Point2f p1((fly_mc_min[j].x + cos(turn * PI / 180) * laser_pos), (fly_mc_min[j].y + sin(turn * PI / 180) * laser_pos));

						turn = flyEllipse.angle + 90;
						Point2f p2((fly_mc_min[j].x + cos(turn * PI / 180) * laser_pos), (fly_mc_min[j].y + sin(turn * PI / 180) * laser_pos));

						double res1 = cv::norm(p1 - fly_mc_max[k]);
						double res2 = cv::norm(p2 - fly_mc_max[k]);

						Mat fly_pt;

						if (res1 > res2)
						{
							fly_pt = refineFlyCenter(pt[0], p1, fly_image_width, fly_image_height);
							circle(fly_frame, p1, 1, Scalar(255, 255, 255), CV_FILLED, 1);
						}
						else
						{
							fly_pt = refineFlyCenter(pt[0], p2, fly_image_width, fly_image_height);
							circle(fly_frame, p2, 1, Scalar(255, 255, 255), CV_FILLED, 1);
						}

						pt[0] = tkf[0].Correct(fly_pt);
					}
					else
						flyview_track = false;
				}
				else
				{
					// if no fly detected, switch back to arena view to get coarse fly location and position update
					//arena_frame = fin.ReadFrame(imageCount);
					arena_img = arena_cam.GrabFrame();
					arena_frame = arena_cam.convertImagetoMat(arena_img);
					//undistort(arena_tframe, arena_frame, cameraMatrix, distCoeffs);

					threshold(arena_frame, arena_mask, arena_thresh, 255, THRESH_BINARY_INV);
					arena_mask &= outer_mask;

					erode(arena_mask, arena_mask, erodeElement, Point(-1, -1), 1);
					dilate(arena_mask, arena_mask, dilateElement, Point(-1, -1), 1);

					vector<vector<Point>> arena_contours;
					vector<Vec4i> arena_hierarchy;

					findContours(arena_mask, arena_contours, arena_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					/// Get the moments and mass centers
					vector<Moments> arena_mu(arena_contours.size());
					vector<Point2f> arena_mc(arena_contours.size());

					vector<Mat> arena_pt;

					for (int i = 0; i < arena_contours.size(); i++)
					{
						drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());

						arena_mu[i] = moments(arena_contours[i], false);
						arena_mc[i] = Point2f(arena_mu[i].m10 / arena_mu[i].m00, arena_mu[i].m01 / arena_mu[i].m00);

						circle(arena_frame, arena_mc[i], 1, Scalar(255, 255, 255), CV_FILLED, 1);
						arena_pt.push_back(backProject(arena_mc[i], cameraMatrix, rotationMatrix, tvec));
					}
					
					for (int i = 0; i < NFLIES; i++)
					{
						if (arena_pt.size() > 0)
						{
							int j = findClosestPoint(pt[i], arena_pt);
							pt[i] = tkf[i].Correct(arena_pt[j]);

							arena_pt.erase(arena_pt.begin() + j);
						}
					}

					ellipse(arena_frame, arenaMask, Scalar(255, 255, 255));

					//imshow("arena image", arena_frame);
					//imshow("arena mask", arena_mask);
				}
								
				#pragma omp critical
				{
					flyDispStream.push(fly_frame);
					flyImageStream.push(fly_img);
					flyTimeStamps.push(fly_stamp);
				}

				//imshow("fly image", fly_frame);
				//imshow("fly mask min", fly_mask_min);
				//imshow("fly mask max", fly_mask_max);
				//waitKey(1);

				if (GetAsyncKeyState(VK_SPACE))
					flyview_record = true;

				if (GetAsyncKeyState(VK_RETURN))
					flyview_track = true;

				if (GetAsyncKeyState(VK_ESCAPE))
				{
					stream = false;
					break;
				}
			}
		}


		#pragma omp section
		{
			while (true)
			{
				if (!flyImageStream.empty())
				{
					if (flyview_record)
					{
						Image wImage = flyImageStream.front();
						TimeStamp wStamp = flyTimeStamps.front();

						fout.WriteFrame(wStamp, wImage);
						fout.WriteLog(wStamp);
						fout.nframes++;
					}

					#pragma omp critical
					{
						flyImageStream.pop();
						flyTimeStamps.pop();
					}
				}

				printf("Recording buffer size %d, Frames written %d\r", flyImageStream.size(), fout.nframes);

				if (flyImageStream.size() == 0 && !stream)
					break;
			}
		}

		#pragma omp section
		{
			namedWindow("controls");
			createTrackbar("arena thresh", "controls", &arena_thresh, 255);
			createTrackbar("fly min", "controls", &fly_min, 255);
			createTrackbar("fly max", "controls", &fly_max, 255);
			createTrackbar("laser pos", "controls", &laser_pos, 100);

			while (true)
			{
				if (!flyDispStream.empty())
				{
					imshow("fly image", flyDispStream.back());
					waitKey(1);
					
					#pragma omp critical
					{
						flyDispStream = queue<Mat>();
					}
				}

				if (!stream)
				{
					destroyWindow("fly image");
					break;
				}
			}
		}



	}

	//fin.Close();
	arena_cam.Stop();
	fly_cam.Stop();

	fout.Close();
	
	printf("\n\nPress Enter to exit...\n");
	getchar();

	return 0;
}
