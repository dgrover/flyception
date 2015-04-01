// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

#define BASE_HEIGHT 3.175

#define GALVO_X_MIRROR_ANGLE 15

#define SCALEX 0.00075
#define SCALEY 0.00075

#define NFLIES 1
#define NLOSTFRAMES 5
#define MAXRECFRAMES 50000

bool stream = true;
bool flyview_track = false;
bool flyview_record = false;

queue <Mat> arenaDispStream;
queue <Mat> arenaMaskStream;

queue <Mat> flyDispStream;
queue <Mat> flyMaskStream;

queue <Image> flyImageStream;
queue <TimeStamp> flyTimeStamps;

queue <Point2f> laser_pt;
queue <Point2f> fly_pt;

//struct {
//	bool operator() (cv::Vec4i pt1, cv::Vec4i pt2) { return (pt1[3] > pt2[3]); }
//} mycomp_dsize;
//
//struct {
//	bool operator() (cv::Vec4i pt1, cv::Vec4i pt2) { return (pt1[2] < pt2[2]); }
//} mycomp_dind;

bool myfny(Point p1, Point p2)
{
	return p1.y < p2.y;
}

bool myfnx(Point p1, Point p2)
{
	return p1.x < p2.x;
}

Point2f rotateFlyCenter(Point2f p, int image_width, int image_height)
{
	Point2f temp, refPt;
	
	//move point to origin and rotate by 15 degrees due to the tilt of the galvo x-mirror
	temp.x = (cos(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.x - image_width / 2) - sin(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.y - image_height / 2));
	temp.y = (sin(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.x - image_width / 2) + cos(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.y - image_height / 2));

	refPt.x = (image_width / 2) + temp.x;
	refPt.y = (image_height / 2) + temp.y;

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

//float angleBetween(Point v1, Point v2, Point c)
//{
//	v1 = v1 - c;
//	v2 = v2 - c;
//
//	float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
//	float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);
//
//	float dot = v1.x * v2.x + v1.y * v2.y;
//
//	float a = dot / (len1 * len2);
//
//	if (a >= 1.0)
//		return 0.0;
//	else if (a <= -1.0)
//		return CV_PI;
//	else
//		return acos(a) * 180 / CV_PI;
//}

bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
{
	float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
	s10_x = p1_x - p0_x;
	s10_y = p1_y - p0_y;
	s32_x = p3_x - p2_x;
	s32_y = p3_y - p2_y;

	denom = s10_x * s32_y - s32_x * s10_y;
	if (denom == 0)
		return false; // Collinear
	bool denomPositive = denom > 0;

	s02_x = p0_x - p2_x;
	s02_y = p0_y - p2_y;
	s_numer = s10_x * s02_y - s10_y * s02_x;
	if ((s_numer < 0) == denomPositive)
		return false; // No collision

	t_numer = s32_x * s02_y - s32_y * s02_x;
	if ((t_numer < 0) == denomPositive)
		return false; // No collision

	if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
		return false; // No collision
	// Collision detected
	t = t_numer / denom;
	if (i_x != NULL)
		*i_x = p0_x + (t * s10_x);
	if (i_y != NULL)
		*i_y = p0_y + (t * s10_y);

	return true;
}

//int sign(int v)
//{
//	return v > 0 ? 1 : -1;
//}


int _tmain(int argc, _TCHAR* argv[])
{
	int arena_image_width = 512, arena_image_height = 512;
	int arena_image_left = 384, arena_image_top = 256;

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

	//vector<Tracker> tkf(NFLIES);
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
	error = arena_cam.SetProperty(SHUTTER, 0.498);
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
	error = fly_cam.SetProperty(GAIN, 5.105);

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

	Serial* SP = new Serial("COM4");    // adjust as needed

	if (SP->IsConnected())
		printf("Connecting arduino [OK]\n");
	
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
	Mat fly_frame, fly_mask;

	int arena_thresh = 85;
	int fly_thresh = 75; 

	int fly_erode = 2;
	int fly_dilate = 2;

	int head_center = 50;

	Mat fly_erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
	Mat fly_dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));

	Mat arena_erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	Mat arena_dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

	int rcount = 0;
	int key_state = 0;
	
	int track_key_state = 0;
	int flash_key_state = 0;

	int lost = 0;

	printf("Press [F1] to start/stop tracking. [F2] to start/stop recording. Press [ESC] to exit.\n\n");

	#pragma omp parallel sections num_threads(3)
	{
		#pragma omp section
		{
			int ltime = 0;
			int ctime = 0;
			int dtime = 0;

			while (true)
			{
				//for (int i = 0; i < NFLIES; i++)
				//	pt[i] = tkf[i].Predict();

				//ndq.ConvertPtToDeg(pt[0]);
				//ndq.write();
				
				pt2d = Point2f(-1 , -1);

				fly_img = fly_cam.GrabFrame();
				fly_stamp = fly_cam.GetTimeStamp();
				fly_frame = fly_cam.convertImagetoMat(fly_img);

				//GaussianBlur(fly_frame, fly_frame, Size(5, 5), 50.0);

				threshold(fly_frame, fly_mask, fly_thresh, 255, THRESH_BINARY_INV);

				erode(fly_mask, fly_mask, fly_erodeElement, Point(-1, -1), fly_erode);
				dilate(fly_mask, fly_mask, fly_dilateElement, Point(-1, -1), fly_dilate);
				


				if (flyview_track)
				{
					vector<vector<Point>> fly_contours;
					findContours(fly_mask, fly_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					vector<vector<Point>> hull(fly_contours.size());
					//vector<vector<int>> hull2(fly_contours.size());
					//vector<vector<Vec4i>> defects(fly_contours.size());

					int j;

					if (fly_contours.size() > 0)
					{
						lost = 0;
						double max_size = 0;

						// Get the moments and mass centers
						vector<Moments> fly_mu(fly_contours.size());
						vector<Point2f> fly_mc(fly_contours.size());

						for (int i = 0; i < fly_contours.size(); i++)
						{
							//drawContours(fly_mask, fly_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
							
							convexHull(Mat(fly_contours[i]), hull[i], false);
							//convexHull(Mat(fly_contours[i]), hull2[i], false);
							//convexityDefects(Mat(fly_contours[i]), hull2[i], defects[i]);

							fly_mu[i] = moments(fly_contours[i], false);
							fly_mc[i] = Point2f(fly_mu[i].m10 / fly_mu[i].m00, fly_mu[i].m01 / fly_mu[i].m00);

							double csize = contourArea(fly_contours[i]);

							if (csize > max_size)
							{
								j = i;
								max_size = csize;
							}
						}
						
						//int j = findClosestPoint(Point2f(fly_image_width/2, fly_image_height/2), fly_mc);
						//pt2d = fly_mc[j];
						//circle(fly_frame, pt2d, 1, Scalar(255, 255, 255), FILLED, 1);

						drawContours(fly_frame, fly_contours, j, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
						//drawContours(fly_frame, hull, j, Scalar::all(255), 1, 8, vector<Vec4i>(), 0, Point());

						
						vector<bool> n = { false, false, false, false };

						vector<Point> left, top, bottom, right;
						vector<Point> edge;

						for (int i = 0; i < fly_contours[j].size(); i++)
						{
							if (fly_contours[j][i].x == 1)
							{
								n[0] = true;
								left.push_back(fly_contours[j][i]);
							}

							if (fly_contours[j][i].x == 254)
							{
								n[1] = true;
								right.push_back(fly_contours[j][i]);
							}

							if (fly_contours[j][i].y == 1)
							{
								n[2] = true;
								top.push_back(fly_contours[j][i]);
							}

							if (fly_contours[j][i].y == 254)
							{
								n[3] = true;
								bottom.push_back(fly_contours[j][i]);
							}
						}
						
						int sum = std::accumulate(n.begin(), n.end(), 0);

						if (sum == 1)
						{
							if (!left.empty())
							{
								edge.push_back(*min_element(left.begin(), left.end(), myfny));
								edge.push_back(*max_element(left.begin(), left.end(), myfny));
							}

							if (!right.empty())
							{
								edge.push_back(*min_element(right.begin(), right.end(), myfny));
								edge.push_back(*max_element(right.begin(), right.end(), myfny));
							}

							if (!top.empty())
							{
								edge.push_back(*min_element(top.begin(), top.end(), myfnx));
								edge.push_back(*max_element(top.begin(), top.end(), myfnx));
							}

							if (!bottom.empty())
							{
								edge.push_back(*min_element(bottom.begin(), bottom.end(), myfnx));
								edge.push_back(*max_element(bottom.begin(), bottom.end(), myfnx));
							}
						}

						if (sum == 2)
						{
							if (!left.empty() && !top.empty())
							{
								edge.push_back(*max_element(left.begin(), left.end(), myfny));
								edge.push_back(*max_element(top.begin(), top.end(), myfnx));
							}

							if (!left.empty() && !bottom.empty())
							{
								edge.push_back(*min_element(left.begin(), left.end(), myfny));
								edge.push_back(*max_element(bottom.begin(), bottom.end(), myfnx));
							}

							if (!right.empty() && !bottom.empty())
							{
								edge.push_back(*min_element(bottom.begin(), bottom.end(), myfnx));
								edge.push_back(*min_element(right.begin(), right.end(), myfny));
							}

							if (!right.empty() && !top.empty())
							{
								edge.push_back(*min_element(top.begin(), top.end(), myfnx));
								edge.push_back(*max_element(right.begin(), right.end(), myfny));
							}
						}

						if (edge.size() == 2)
						{
							Point2f body_center = fly_mc[j];
							Point2f edge_center = Point((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);

							//line(fly_frame, body_center, edge_center, Scalar::all(255), 1, 8, 0);

							Point2f p1, p2;
							float m, c;

							bool flag = false;

							// Check if the line is a vertical line because vertical lines don't have slope
							if (abs(body_center.x - edge_center.x) > 3.0)
							{
								flag = true;

								p1.x = 0;
								p2.x = fly_image_width;

								// Slope equation (y1 - y2) / (x1 - x2)
								m = (body_center.y - edge_center.y) / (body_center.x - edge_center.x);

								// Line equation:  y = mx + c
								c = body_center.y - (m * body_center.x);
								p1.y = m * p1.x + c;
								p2.y = m * p2.x + c;
							}
							else
							{
								p1.x = edge_center.x;
								p2.x = edge_center.x;

								p1.y = 0;
								p2.y = fly_image_height;
							}

							vector<Point2f> head;

							//for (int i = 1; i <= hull[j].size(); i++)
							//{
							//	float x, y;
							//	Point2f p, q, r, s;

							//	p = p1;
							//	q = p2;

							//	if (i < hull[j].size())
							//	{
							//		r = hull[j][i - 1];
							//		s = hull[j][i];
							//	}
							//	else
							//	{
							//		r = hull[j][i - 1];
							//		s = hull[j][0];

							//	}

							//	bool cross = get_line_intersection(p.x, p.y, q.x, q.y, r.x, r.y, s.x, s.y, &x, &y);
							//
							//	if (cross)
							//		head.push_back(Point2f(x, y));
							//}


							Mat tmask(256, 256, CV_8UC1);
							drawContours(tmask, hull, j, Scalar(255, 255, 255), FILLED, 1);
							//drawContours(tmask, fly_contours, j, Scalar(255, 255, 255), FILLED, 1);

							Point curr, last;

							if (flag)
							{
								for (int i = 1; i < fly_image_width; i++)
								{
									bool bound = true;

									last.x = i - 1;
									last.y = m*last.x + c;

									if (last.y < 0 || last.y >= fly_image_height)
										bound = false;

									curr.x = i;
									curr.y = m*curr.x + c;

									if (curr.y < 0 || curr.y >= fly_image_height)
										bound = false;

									if (bound)
									{
										if (tmask.at<uchar>(curr.y, curr.x) != tmask.at<uchar>(last.y, last.x))
										{
											head.push_back(curr);
											//circle(fly_frame, curr, 5, Scalar(255, 255, 255), FILLED, 1);
										}
									}
								}
							}
							else
							{
								for (int i = 1; i < fly_image_height; i++)
								{
									bool bound = true;

									last.x = p1.x;
									last.y = i-1;

									if (last.x < 0 || last.x >= fly_image_width)
										bound = false;

									curr.x = p1.x;
									curr.y = i;

									if (curr.x < 0 || curr.x >= fly_image_width)
										bound = false;

									if (bound)
									{
										if (tmask.at<uchar>(curr.y, curr.x) != tmask.at<uchar>(last.y, last.x))
										{
											head.push_back(curr);
											//circle(fly_frame, curr, 5, Scalar(255, 255, 255), FILLED, 1);
										}
									}
								}
							}


							if (head.size() > 0)
							{
								int i = findClosestPoint(Point2f(fly_image_width / 2, fly_image_height / 2), head);
								
								Point2f a = head[i];
								Point2f b = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);

								Point2f h;

								if (flag)
								{
									if (a.x > b.x)
										h.x = a.x - head_center / sqrt(1 + (m*m));
									else
										h.x = a.x + head_center / sqrt(1 + (m*m));

									h.y = m*h.x + c;
								}
								else
								{
									h.x = a.x;

									if (b.y < a.y)
										h.y = a.y - head_center;
									else
										h.y = a.y + head_center;
								}

								circle(fly_frame, h, 1, Scalar(255, 255, 255), FILLED, 1);
								pt2d = h;

								pt2d = rotateFlyCenter(pt2d, fly_image_width, fly_image_height);

								float diffx = pt2d.x - (fly_image_width / 2);
								float diffy = pt2d.y - (fly_image_height / 2);

								ndq.ConvertPixelToDeg(diffx*SCALEX, diffy*SCALEY);
								pt[0] = ndq.ConvertDegToPt();
								//tkf[0].Correct(pt[0]);
								ndq.write();
							}
						}
					}
					else
					{
						lost++;

						if (lost > NLOSTFRAMES)
						{
							flyview_track = false;
							flyview_record = false;
							ndq.reset();
							
							//for (int i = 0; i < NFLIES; i++)
							//	tkf[i].Init();
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
								
								pt[i] = arena_pt[j];
								//tkf[i].Correct(pt[i]);
								
								arena_pt.erase(arena_pt.begin() + j);
							}
						}
					}

					ndq.ConvertPtToDeg(pt[0]);
					ndq.write();

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

				putText(fly_frame, to_string(dtime), Point((fly_image_width-50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
				
				if (flyview_record)
					putText(fly_frame, to_string(rcount), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

				#pragma omp critical
				{
					if (!flyview_track)
					{
						arenaMaskStream.push(arena_mask);
						arenaDispStream.push(arena_frame);
					}

					flyMaskStream.push(fly_mask);
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
					if (!track_key_state)
						flyview_track = !flyview_track;

					track_key_state = 1;
				}
				else
					track_key_state = 0;

				if (GetAsyncKeyState(VK_F2))
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

				if (GetAsyncKeyState(VK_F3))
				{
					if (!flash_key_state)
						SP->WriteData("1", 1);

					flash_key_state = 1;
				}
				else
					flash_key_state = 0;
				
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

					//fout.WriteFrame(flyTimeStamps.front(), flyImageStream.front());
					fout.WriteFrame(flyImageStream.front());
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
			createTrackbar("fly thresh", "controls", &fly_thresh, 255);
			createTrackbar("erode", "controls", &fly_erode, 5);
			createTrackbar("dilate", "controls", &fly_dilate, 5);
			createTrackbar("head", "controls", &head_center, 100);

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
					imshow("fly mask", flyMaskStream.front());
					
					#pragma omp critical
					{
						flyDispStream = queue<Mat>();
						flyMaskStream = queue<Mat>();
					}
				}

				waitKey(1);

				if (!stream)
				{
					destroyWindow("controls");
					destroyWindow("arena image");
					destroyWindow("arena mask");
					
					destroyWindow("fly image");
					destroyWindow("fly mask");

					break;
				}
			}
		}
	}

	arena_cam.Stop();
	fly_cam.Stop();

	printf("\n\nCentering galvo ");
	ndq.ConvertPtToDeg(Point2f(0, 0));
	ndq.write();
	printf("[OK]\n");

	return 0;
}
