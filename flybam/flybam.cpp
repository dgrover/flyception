// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

#define BASE_HEIGHT 3.175

#define SCALE 0.61/(2*65.638107)
#define GALVO_X_MIRROR_ANGLE 15

#define NFLIES 1
#define NLOSTFRAMES 1
#define MAXRECFRAMES 50000

bool stream = true;
bool flyview_track = false;
bool flyview_record = false;

queue <Mat> arenaDispStream;
queue <Mat> arenaMaskStream;

queue <Mat> flyDispStream;
queue <Mat> flyMinMaskStream;

queue <Image> flyImageStream;
queue <TimeStamp> flyTimeStamps;

queue <Point2f> laser_pt;
queue <Point2f> fly_pt;

struct {
	bool operator() (cv::Vec4i pt1, cv::Vec4i pt2) { return (pt1[3] > pt2[3]); }
} mycomp;

Point2f refineFlyCenter(Point2f pt, Point2f p, int image_width, int image_height)
{
	Point2f temp, refPt;

	//rotate fly center point by 15 degrees due to the tilt of the galvo x-mirror
	temp.x = (cos(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.x - image_width / 2) - sin(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.y - image_height / 2));
	temp.y = (sin(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.x - image_width / 2) + cos(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.y - image_height / 2));

	refPt.x = pt.x + temp.x * SCALE;
	refPt.y = pt.y + temp.y * SCALE;

	//printf("[%f %f]\n", pt.at<double>(0, 0), pt.at<double>(1, 0));
	//printf("[%f %f]\n", refPt.at<double>(0, 0), refPt.at<double>(1, 0));

	return refPt;
}

Point2f backProject(Point2f p, Mat cameraMatrix, Mat rotationMatrix, Mat tvec)
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

	return Point2f((float)pt.at<double>(0, 0), (float)pt.at<double>(1, 0));
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

	for (double angle = 0; angle <= 2 * CV_PI; angle += 0.001) //You are using radians so you will have to increase by a very small amount
		c3d.push_back(Point3f(center.x + radius*cos(angle), center.y + radius*sin(angle), BASE_HEIGHT));

	projectPoints(c3d, rvec, tvec, cameraMatrix, distCoeffs, c2d);

	circleMask = fitEllipse(c2d);

	return circleMask;
}

float dist(Point2f p1, Point2f p2)
{
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	return(sqrt(dx*dx + dy*dy));
}

int findClosestPoint(Point2f pt, vector<Point2f> nbor)
{
	int fly_index = 0;
	if (nbor.size() == 1)
		return fly_index;
	else
	{
		float fly_dist = dist(pt, nbor[0]);

		for (int i = 1; i < nbor.size(); i++)
		{
			float res = dist(pt, nbor[i]);
			if (res < fly_dist)
			{
				fly_dist = res;
				fly_index = i;                //Store the index of nearest point
			}
		}

		return fly_index;
	}
}

bool isLeft(Point a, Point b, Point c){
	return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) > 0;
}

//int sign(int v)
//{
//	return v > 0 ? 1 : -1;
//}


int _tmain(int argc, _TCHAR* argv[])
{
	int arena_image_width = 512, arena_image_height = 512;
	int arena_image_left = 384, arena_image_top = 256;

	//int fly_image_width = 512, fly_image_height = 512;
	//int fly_image_left = 384, fly_image_top = 256;

	int fly_image_width = 256, fly_image_height = 256;
	int fly_image_left = 512, fly_image_top = 384;

	PGRcam arena_cam, fly_cam;
	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;
	FlyCapture2::Error error;

	string filename = "..\\calibration\\arena\\camera_projection_data.xml";

	Mat cameraMatrix, distCoeffs;
	Mat rvec(1, 3, cv::DataType<double>::type);
	Mat tvec(1, 3, cv::DataType<double>::type);
	Mat rotationMatrix(3, 3, cv::DataType<double>::type);

	FmfWriter fout;

	vector<Tracker> tkf(NFLIES);
	vector<Point2f> pt(NFLIES);

	Point2f pt2d;

	error = busMgr.GetNumOfCameras(&numCameras);
	printf("Number of cameras detected: %u\n", numCameras);

	if (numCameras < 2)
	{
		printf("Insufficient number of cameras... exiting\n");
		return -1;
	}

	printf("Initializing arena view camera ");
	error = busMgr.GetCameraFromIndex(0, &guid);
	error = arena_cam.Connect(guid);
	error = arena_cam.SetCameraParameters(arena_image_left, arena_image_top, arena_image_width, arena_image_height);
	//arena_cam.GetImageSize(arena_image_width, arena_image_height);
	error = arena_cam.SetProperty(SHUTTER, 1.003);
	error = arena_cam.SetProperty(GAIN, 0.0);
	error = arena_cam.Start();

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}
	printf("[OK]\n");

	printf("Initializing fly view camera ");
	error = busMgr.GetCameraFromIndex(1, &guid);
	error = fly_cam.Connect(guid);
	error = fly_cam.SetCameraParameters(fly_image_left, fly_image_top, fly_image_width, fly_image_height);
	//fly_cam.GetImageSize(fly_image_width, fly_image_height);
	error = fly_cam.SetTrigger();
	error = fly_cam.SetProperty(SHUTTER, 1.003);
	error = fly_cam.SetProperty(GAIN, 0.0);
	
	error = fly_cam.Start();
	
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}
	printf("[OK]\n\n");

	FileStorage fs(filename, FileStorage::READ);
	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	fs["rvec"] >> rvec;
	fs["tvec"] >> tvec;
	fs["rotation_matrix"] >> rotationMatrix;
	fs.release();

	//configure and start NIDAQ
	Daq ndq;
	ndq.configure();
	ndq.start();

	//create arena mask
	Mat outer_mask = Mat::zeros(Size(arena_image_width, arena_image_height), CV_8UC1);
	RotatedRect arenaMask = createArenaMask(cameraMatrix, distCoeffs, rvec, tvec);
	ellipse(outer_mask, arenaMask, Scalar(255, 255, 255), FILLED);
	
	Image fly_img, arena_img;
	TimeStamp fly_stamp;

	Mat arena_frame, arena_mask;
	Mat fly_frame, fly_mask_min;

	int arena_thresh = 85;
	int fly_min = 65; 

	int fly_erode = 2;
	int fly_dilate = 2;

	Mat fly_erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
	Mat fly_dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));

	Mat arena_erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	Mat arena_dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

	int rcount = 0;
	int key_state = 0;
	int lost = 0;

	printf("Press [F1] to start/stop recording. Press [ESC] to exit.\n\n");

	#pragma omp parallel sections num_threads(3)
	{
		#pragma omp section
		{
			int ltime = 0;
			int ctime = 0;
			int dtime = 0;

			while (true)
			{
				for (int i = 0; i < NFLIES; i++)
					pt[i] = tkf[i].Predict();

				pt2d = Point2f(-1 , -1);

				ndq.ConvertPtToVoltage(pt[0]);
				ndq.write();

				fly_img = fly_cam.GrabFrame();
				fly_stamp = fly_cam.GetTimeStamp();
				fly_frame = fly_cam.convertImagetoMat(fly_img);

				threshold(fly_frame, fly_mask_min, fly_min, 255, THRESH_BINARY_INV);

				erode(fly_mask_min, fly_mask_min, fly_erodeElement, Point(-1, -1), fly_erode);
				dilate(fly_mask_min, fly_mask_min, fly_dilateElement, Point(-1, -1), fly_dilate);
				
				if (flyview_track)
				{
					vector<vector<Point>> fly_contours_min;
					findContours(fly_mask_min, fly_contours_min, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					vector<vector<Point>> hull(fly_contours_min.size());
					vector<vector<int>> hull2(fly_contours_min.size());
					vector<vector<Vec4i>> defects(fly_contours_min.size());

					int j;

					if (fly_contours_min.size() > 0)
					{
						lost = 0;
						double max_size = 0;

						// Get the moments and mass centers
						vector<Moments> fly_mu_min(fly_contours_min.size());
						vector<Point2f> fly_mc_min(fly_contours_min.size());

						for (int i = 0; i < fly_contours_min.size(); i++)
						{
							//drawContours(fly_mask_min, fly_contours_min, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
							
							convexHull(Mat(fly_contours_min[i]), hull[i], false);
							convexHull(Mat(fly_contours_min[i]), hull2[i], false);
							
							convexityDefects(Mat(fly_contours_min[i]), hull2[i], defects[i]);

							fly_mu_min[i] = moments(fly_contours_min[i], false);
							fly_mc_min[i] = Point2f(fly_mu_min[i].m10 / fly_mu_min[i].m00, fly_mu_min[i].m01 / fly_mu_min[i].m00);

							double csize = contourArea(fly_contours_min[i]);

							if (csize > max_size)
							{
								j = i;
								max_size = csize;
							}

						}
						
						//int j = findClosestPoint(Point2f(fly_image_width/2, fly_image_height/2), fly_mc_min);
						//pt2d = fly_mc_min[j];

						drawContours(fly_frame, fly_contours_min, j, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
						//drawContours(fly_frame, hull, j, Scalar::all(255), 1, 8, vector<Vec4i>(), 0, Point());

						std::sort(defects[j].begin(), defects[j].end(), mycomp);

						if (defects[j].size() >= 3)
						{
							int ind1 = defects[j][1][2];
							int ind2 = defects[j][2][2];

							circle(fly_frame, fly_contours_min[j][ind1], 5, Scalar(255, 255, 255), FILLED, 1);
							circle(fly_frame, fly_contours_min[j][ind2], 5, Scalar(255, 255, 255), FILLED, 1);

							bool ans = isLeft(fly_contours_min[j][ind1], fly_contours_min[j][ind2], fly_mc_min[j]);

							vector<Point> fly_head;

							for (int k = 0; k < fly_contours_min[j].size(); k++)
							{
								if (isLeft(fly_contours_min[j][ind1], fly_contours_min[j][ind2], fly_contours_min[j][k]) != ans)
								{
									fly_head.push_back(fly_contours_min[j][k]);
									//circle(fly_frame, fly_contours_min[j][k], 1, Scalar(255, 255, 255), FILLED, 1);
								}
							}

							Moments fly_head_mu = moments(fly_head, false);
							Point2f fly_head_mc = Point2f(fly_head_mu.m10 / fly_head_mu.m00, fly_head_mu.m01 / fly_head_mu.m00);

							circle(fly_frame, fly_head_mc, 1, Scalar(255, 255, 255), FILLED, 1);
							pt2d = fly_head_mc;

							//circle(fly_frame, pt2d, 1, Scalar(255, 255, 255), FILLED, 1);
							tkf[0].Correct(refineFlyCenter(pt[0], pt2d, fly_image_width, fly_image_height));
						}
					}
					else
					{
						lost++;

						if (lost > NLOSTFRAMES)
						{
							flyview_track = false;
							for (int i = 0; i < NFLIES; i++)
								tkf[i].Init();
						}
					}
				}
						
				// if no fly detected, switch back to arena view to get coarse fly location and position update
				if (!flyview_track)
				{
					arena_img = arena_cam.GrabFrame();
					arena_frame = arena_cam.convertImagetoMat(arena_img);
					//Mat arena_tframe = arena_cam.convertImagetoMat(arena_img);

					//undistort(arena_tframe, arena_frame, cameraMatrix, distCoeffs);

					threshold(arena_frame, arena_mask, arena_thresh, 255, THRESH_BINARY_INV);
					arena_mask &= outer_mask;

					erode(arena_mask, arena_mask, arena_erodeElement, Point(-1, -1), 1);
					dilate(arena_mask, arena_mask, arena_dilateElement, Point(-1, -1), 1);

					vector<vector<Point>> arena_contours;

					findContours(arena_mask, arena_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					if (arena_contours.size() > 0)
					{
						// Get the moments and mass centers
						vector<Moments> arena_mu(arena_contours.size());
						vector<Point2f> arena_mc(arena_contours.size());

						vector<Point2f> arena_pt;

						for (int i = 0; i < arena_contours.size(); i++)
						{
							//drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
							arena_mu[i] = moments(arena_contours[i], false);
							arena_mc[i] = Point2f(arena_mu[i].m10 / arena_mu[i].m00, arena_mu[i].m01 / arena_mu[i].m00);

							circle(arena_frame, arena_mc[i], 1, Scalar(255, 255, 255), FILLED, 1);
							arena_pt.push_back(backProject(arena_mc[i], cameraMatrix, rotationMatrix, tvec));
						}

						for (int i = 0; i < NFLIES; i++)
						{
							if (arena_pt.size() > 0)
							{
								int j = findClosestPoint(pt[i], arena_pt);
								tkf[i].Correct(arena_pt[j]);
								arena_pt.erase(arena_pt.begin() + j);
							}
						}
					}
				}

				ctime = fly_stamp.cycleCount;

				if (ctime < ltime)
					dtime = ctime + (8000 - ltime);
				else
					dtime = ctime - ltime;

				if (dtime > 0)
					dtime = 8000 / dtime;
				else
					dtime = 0;

				ltime = ctime;

				putText(fly_frame, to_string(dtime), Point(225, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
				
				if (flyview_record)
					putText(fly_frame, to_string(rcount), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));


				#pragma omp critical
				{
					if (!flyview_track)
					{
						arenaMaskStream.push(arena_mask);
						arenaDispStream.push(arena_frame);
					}

					flyMinMaskStream.push(fly_mask_min);
					flyDispStream.push(fly_frame);

					if (flyview_record)
					{
						laser_pt.push(pt[0]);
						fly_pt.push(pt2d);

						flyTimeStamps.push(fly_stamp);
						flyImageStream.push(fly_img);
					}
				}

				if (GetAsyncKeyState(VK_F1))
				{
					if (!key_state)
					{
						flyview_record = !flyview_record;
						rcount = 0;
					}

					key_state = 1;
				}
				else
					key_state = 0;
				
				if (GetAsyncKeyState(VK_RETURN))
					flyview_track = true;

				if (GetAsyncKeyState(VK_ESCAPE))
				{
					stream = false;
					break;
				}

				if (flyview_record)
				{
					if (rcount++ == MAXRECFRAMES)
					{
						rcount = 0;
						flyview_record = false;
					}
				}

			}
		}

		#pragma omp section
		{
			while (true)
			{
				if (!flyImageStream.empty())
				{
					if (!fout.IsOpen())
					{
						fout.Open();
						fout.InitHeader(fly_image_width, fly_image_height);
						fout.WriteHeader();
						printf("Recording ");
					}

					fout.WriteFrame(flyTimeStamps.front(), flyImageStream.front());
					fout.WriteLog(flyTimeStamps.front());
					fout.WriteTraj(laser_pt.front(), fly_pt.front());
					fout.nframes++;

					#pragma omp critical
					{
						flyImageStream.pop();
						flyTimeStamps.pop();
						laser_pt.pop();
						fly_pt.pop();
					}
				}
				else
				{
					if (!flyview_record && fout.IsOpen())
					{
						fout.Close();
						printf("[OK]\n");
					}
				}

				if (!stream)
				{
					if (flyImageStream.empty())
					{
						if (flyview_record)
						{
							fout.Close();
							printf("[OK]\n");
						}
						break;
					}

				}
			}
		}

		#pragma omp section
		{
			namedWindow("controls", WINDOW_AUTOSIZE);
			createTrackbar("arena thresh", "controls", &arena_thresh, 255);
			createTrackbar("fly min", "controls", &fly_min, 255);
			createTrackbar("erode", "controls", &fly_erode, 5);
			createTrackbar("dilate", "controls", &fly_dilate, 5);

			while (true)
			{

				if (!arenaDispStream.empty())
				{
					ellipse(arenaDispStream.front(), arenaMask, Scalar(255, 255, 255));
					imshow("arena image", arenaDispStream.front());
					imshow("arena mask", arenaMaskStream.front());

					#pragma omp critical
					{
						arenaDispStream = queue<Mat>();
						arenaMaskStream = queue<Mat>();
					}
				}
				
				if (!flyDispStream.empty())
				{
					imshow("fly image", flyDispStream.front());
					imshow("fly min mask", flyMinMaskStream.front());
					
					#pragma omp critical
					{
						flyDispStream = queue<Mat>();
						flyMinMaskStream = queue<Mat>();
					}
				}

				waitKey(1);

				if (!stream)
				{
					destroyWindow("controls");
					destroyWindow("arena image");
					destroyWindow("arena mask");
					
					destroyWindow("fly image");
					destroyWindow("fly min mask");

					break;
				}
			}
		}
	}

	arena_cam.Stop();
	fly_cam.Stop();

	printf("\n\nCentering galvo ");
	ndq.ConvertPtToVoltage(Point2f(0, 0));
	ndq.write();
	printf("[OK]\n");

	return 0;
}
