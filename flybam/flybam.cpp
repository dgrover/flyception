// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

#define BASE_HEIGHT 3.175			//in mm
#define GALVO_HEIGHT 65.0			//in mm
#define GALVO_X_MIRROR_ANGLE 15		//in degrees
#define ARENA_RADIUS 20				//in mm
#define TAIL_LENGTH 100

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

Point2f backProject(Point2f p, Mat cameraMatrix, Mat rotationMatrix, Mat tvec, float height)
{
	cv::Mat uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); // [u v 1]
	uvPoint.at<double>(0, 0) = p.x;
	uvPoint.at<double>(1, 0) = p.y;

	cv::Mat tempMat, tempMat2;
	double s;

	tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
	tempMat2 = rotationMatrix.inv() * tvec;
	s = height + tempMat2.at<double>(2, 0); //height Zconst is zero
	s /= tempMat.at<double>(2, 0);

	Mat pt = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
	//printf("[%f %f %f]\n", pt.at<double>(0, 0), pt.at<double>(1, 0), pt.at<double>(2, 0));

	//cv::Mat backPt = 1 / s * cameraMatrix * (rotationMatrix * pt + tvec);
	//printf("[%f %f]\n", backPt.at<double>(0, 0), backPt.at<double>(1, 0));

	return Point2f((float)pt.at<double>(0, 0), (float)pt.at<double>(1, 0));
}

Point2f project3d2d(Point2f pt, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
{
	vector<Point3f> p3d;
	vector<Point2f> p2d;

	p3d.push_back(Point3f(pt.x, pt.y, BASE_HEIGHT));
	projectPoints(p3d, rvec, tvec, cameraMatrix, distCoeffs, p2d);

	return p2d[0];
}

RotatedRect createArenaMask(Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
{
	Point2f center(0, 0);
	
	vector<Point3f> c3d;
	vector<Point2f> c2d;

	RotatedRect circleMask;

	for (double angle = 0; angle <= 2 * CV_PI; angle += 0.001) //You are using radians so you will have to increase by a very small amount
		c3d.push_back(Point3f(center.x + ARENA_RADIUS*cos(angle), center.y + ARENA_RADIUS*sin(angle), BASE_HEIGHT));

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

float dist3d(Point3f p1, Point3f p2)
{
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	float dz = p2.z - p1.z;
	return(sqrt(dx*dx + dy*dy + dz*dz));
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

int findFurthestPoint(Point2f pt, vector<Point2f> nbor)
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
			if (res > fly_dist)
			{
				fly_dist = res;
				fly_index = i;                //Store the index of nearest point
			}
		}

		return fly_index;
	}
}

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

Point2f findAxisCenter(Point2f p1, Point2f p2, Point2f origin)
{
	Point2f p;

	float xdiff = abs(p1.x - p2.x);
	float ydiff = abs(p1.y - p2.y);

	if (xdiff == ydiff)
		return origin;

	float avgdist = (xdiff + ydiff) / 2;

	if (xdiff > ydiff)
	{
		p.y = origin.y;

		if (origin.x == 1)
			p.x = max(p1.x, p2.x) - avgdist;
		else
			p.x = min(p1.x, p2.x) + avgdist;
	}
	else if (xdiff < ydiff)
	{
		p.x = origin.x;

		if (origin.y == 1)
			p.y = max(p1.y, p2.y) - avgdist;
		else
			p.y = min(p1.y, p2.y) + avgdist;
	}

	return p;
}

float findEdgeDistance(Point2f p1, Point2f p2)
{
	float xdiff = abs(p1.x - p2.x);
	float ydiff = abs(p1.y - p2.y);

	return (xdiff + ydiff);
}

int ConvertTimeToFPS(int ctime, int ltime)
{
	int dtime;

	if (ctime < ltime)
		dtime = ctime + (8000 - ltime);
	else
		dtime = ctime - ltime;

	if (dtime > 0)
		dtime = 8000 / dtime;
	else
		dtime = 0;

	return dtime;
}

int _tmain(int argc, _TCHAR* argv[])
{
	int arena_image_width = 512, arena_image_height = 512;
	int arena_image_left = 384, arena_image_top = 256;

	int fly_image_width = 256, fly_image_height = 256;
	int fly_image_left = 512, fly_image_top = 384;

	Point3f galvo_center(0, 0, (BASE_HEIGHT - sqrt((GALVO_HEIGHT * GALVO_HEIGHT) - (ARENA_RADIUS * ARENA_RADIUS))));

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
	vector<Point2f> arena_pt(NFLIES);
	vector<vector<Point2f>> arena_pt_vec(NFLIES);

	Point2f pt2d, wpt;

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
	error = arena_cam.SetProperty(SHUTTER, 0.249);
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
	error = fly_cam.SetProperty(SHUTTER, 0.498);
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
	TimeStamp fly_stamp, arena_stamp;

	Mat arena_frame, arena_mask;
	Mat fly_frame, fly_mask;

	int arena_thresh = 75;
	int fly_thresh = 45; 

	int fly_erode = 1;
	int fly_dilate = 1;

	int head_center = 60;
	int sep = 50;

	Mat fly_erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	Mat fly_dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

	Mat arena_erodeElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	Mat arena_dilateElement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

	int rcount = 0;
	
	int record_key_state = 0;
	int track_key_state = 0;
	int flash_key_state = 0;

	int lost = 0;

	Point2f edge_center;

	//Press [F1] to start/stop tracking. [F2] to start/stop recording. Press [ESC] to exit.
	#pragma omp parallel sections num_threads(5)
	{
		#pragma omp section
		{
			int fly_ltime = 0;
			int fly_fps = 0;

			while (true)
			{
				//for (int i = 0; i < NFLIES; i++)
				//	pt[i] = tkf[i].Predict();

				fly_img = fly_cam.GrabFrame();
				fly_stamp = fly_cam.GetTimeStamp();
				fly_frame = fly_cam.convertImagetoMat(fly_img);

				//GaussianBlur(fly_frame, fly_frame, Size(5, 5), 0, 0);

				threshold(fly_frame, fly_mask, fly_thresh, 255, THRESH_BINARY_INV);

				erode(fly_mask, fly_mask, fly_erodeElement, Point(-1, -1), fly_erode);
				dilate(fly_mask, fly_mask, fly_dilateElement, Point(-1, -1), fly_dilate);
				
				if (flyview_track)
				{
					vector<vector<Point>> fly_contours;
					findContours(fly_mask, fly_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					vector<vector<Point>> hull(fly_contours.size());
					
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
							convexHull(Mat(fly_contours[i]), hull[i], false);
							
							fly_mu[i] = moments(fly_contours[i], false);
							fly_mc[i] = Point2f(fly_mu[i].m10 / fly_mu[i].m00, fly_mu[i].m01 / fly_mu[i].m00);

							double csize = contourArea(fly_contours[i]);

							if (csize > max_size)
							{
								j = i;
								max_size = csize;
							}
						}
						
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
						
						// fly makes contact with single edge
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

							edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
						}

						// fly makes contact with two edges
						if (sum == 2)
						{
							if (!left.empty() && !top.empty())
							{
								Point lmin = *min_element(left.begin(), left.end(), myfny);
								Point lmax = *max_element(left.begin(), left.end(), myfny);
								Point tmin = *min_element(top.begin(), top.end(), myfnx);
								Point tmax = *max_element(top.begin(), top.end(), myfnx);

								float ed = findEdgeDistance(lmin, tmin);

								if (ed < sep)
								{
									edge.push_back(lmax);
									edge.push_back(tmax);

									edge_center = findAxisCenter(edge[0], edge[1], Point2f(1, 1));
								}
								else
								{
									Point2f ec1 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);
									Point2f ec2 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);

									if (dist(edge_center, ec1) < dist(edge_center, ec2))
									{
										edge.push_back(lmin);
										edge.push_back(lmax);

										edge_center = ec1;
									}
									else
									{
										edge.push_back(tmin);
										edge.push_back(tmax);

										edge_center = ec2;
									}
								}
							}

							if (!left.empty() && !bottom.empty())
							{
								Point lmin = *min_element(left.begin(), left.end(), myfny);
								Point lmax = *max_element(left.begin(), left.end(), myfny);
								Point bmin = *min_element(bottom.begin(), bottom.end(), myfnx);
								Point bmax = *max_element(bottom.begin(), bottom.end(), myfnx);

								float ed = findEdgeDistance(lmax, bmin);

								if (ed < sep)
								{
									edge.push_back(lmin);
									edge.push_back(bmax);

									edge_center = findAxisCenter(edge[0], edge[1], Point2f(1, 254));
								}
								else
								{
									Point2f ec1 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);
									Point2f ec2 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

									if (dist(edge_center, ec1) < dist(edge_center, ec2))
									{
										edge.push_back(lmin);
										edge.push_back(lmax);

										edge_center = ec1;
									}
									else
									{
										edge.push_back(bmin);
										edge.push_back(bmax);

										edge_center = ec2;
									}
								}
							}

							if (!right.empty() && !bottom.empty())
							{
								Point rmin = *min_element(right.begin(), right.end(), myfny);
								Point rmax = *max_element(right.begin(), right.end(), myfny);
								Point bmin = *min_element(bottom.begin(), bottom.end(), myfnx);
								Point bmax = *max_element(bottom.begin(), bottom.end(), myfnx);

								float ed = findEdgeDistance(rmax, bmax);

								if (ed < sep)
								{
									edge.push_back(rmin);
									edge.push_back(bmin);

									edge_center = findAxisCenter(edge[0], edge[1], Point2f(254, 254));
								}
								else
								{
									Point2f ec1 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);
									Point2f ec2 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

									if (dist(edge_center, ec1) < dist(edge_center, ec2))
									{
										edge.push_back(rmin);
										edge.push_back(rmax);

										edge_center = ec1;
									}
									else
									{
										edge.push_back(bmin);
										edge.push_back(bmax);

										edge_center = ec2;
									}
								}
							}

							if (!right.empty() && !top.empty())
							{
								Point tmin = *min_element(top.begin(), top.end(), myfnx);
								Point tmax = *max_element(top.begin(), top.end(), myfnx);
								Point rmin = *min_element(right.begin(), right.end(), myfny);
								Point rmax = *max_element(right.begin(), right.end(), myfny);

								float ed = findEdgeDistance(tmax, rmin);

								if (ed < sep)
								{
									edge.push_back(tmin);
									edge.push_back(rmax);

									edge_center = findAxisCenter(edge[0], edge[1], Point2f(254, 1));
								}
								else
								{
									Point2f ec1 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);
									Point2f ec2 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);

									if (dist(edge_center, ec1) < dist(edge_center, ec2))
									{
										edge.push_back(rmin);
										edge.push_back(rmax);

										edge_center = ec1;
									}
									else
									{
										edge.push_back(tmin);
										edge.push_back(tmax);

										edge_center = ec2;
									}
								}
							}

							if (!left.empty() && !right.empty())
							{
								Point lmin = *min_element(left.begin(), left.end(), myfny);
								Point lmax = *max_element(left.begin(), left.end(), myfny);

								Point rmin = *min_element(right.begin(), right.end(), myfny);
								Point rmax = *max_element(right.begin(), right.end(), myfny);

								Point2f ec1 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);
								Point2f ec2 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);

								if (dist(edge_center, ec1) < dist(edge_center, ec2))
								{
									edge.push_back(lmin);
									edge.push_back(lmax);
								}
								else
								{
									edge.push_back(rmin);
									edge.push_back(rmax);
								}

								edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
							}

							if (!top.empty() && !bottom.empty())
							{
								Point tmin = *min_element(top.begin(), top.end(), myfnx);
								Point tmax = *max_element(top.begin(), top.end(), myfnx);

								Point bmin = *min_element(bottom.begin(), bottom.end(), myfnx);
								Point bmax = *max_element(bottom.begin(), bottom.end(), myfnx);

								Point2f ec1 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);
								Point2f ec2 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

								if (dist(edge_center, ec1) < dist(edge_center, ec2))
								{
									edge.push_back(tmin);
									edge.push_back(tmax);
								}
								else
								{
									edge.push_back(bmin);
									edge.push_back(bmax);
								}

								edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
							}
						}


						// fly makes contact with three edges
						if (sum == 3)
						{
							if (!left.empty() && !bottom.empty() && !right.empty())
							{
								Point lmin = *min_element(left.begin(), left.end(), myfny);
								Point lmax = *max_element(left.begin(), left.end(), myfny);

								Point rmin = *min_element(right.begin(), right.end(), myfny);
								Point rmax = *max_element(right.begin(), right.end(), myfny);

								Point bmin = *min_element(bottom.begin(), bottom.end(), myfnx);
								Point bmax = *max_element(bottom.begin(), bottom.end(), myfnx);


								float ed1 = findEdgeDistance(lmax, bmin);
								float ed2 = findEdgeDistance(bmax, rmax);

								if ((ed1 < sep) && (ed2 < sep))
								{

									Point2f ec1 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);
									Point2f ec2 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);

									if (dist(edge_center, ec1) < dist(edge_center, ec2))
									{
										edge.push_back(lmin);
										edge.push_back(lmax);
									}
									else
									{
										edge.push_back(rmin);
										edge.push_back(rmax);
									}

									edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);

								}
								else if ((ed1 > sep) && (ed2 > sep))
								{
									Point2f ec1 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);
									Point2f ec2 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);
									Point2f ec3 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

									float ldist = dist(ec1, edge_center);
									float rdist = dist(ec2, edge_center);
									float bdist = dist(ec3, edge_center);

									float res = ldist;

									edge.push_back(lmin);
									edge.push_back(lmax);

									if (rdist < res)
									{
										edge[0] = rmin;
										edge[1] = rmax;
										res = rdist;
									}

									if (bdist < res)
									{
										edge[0] = bmin;
										edge[1] = bmax;
									}

									edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
								}
								else
								{
									if (ed1 < sep)
									{
										Point2f ec1 = findAxisCenter(lmin, bmax, Point2f(1, 254));
										Point2f ec2 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);

										if (dist(ec1, edge_center) < dist(ec2, edge_center))
										{
											edge.push_back(lmin);
											edge.push_back(bmax);

											edge_center = findAxisCenter(edge[0], edge[1], Point2f(1, 254));
										}
										else
										{
											edge.push_back(rmin);
											edge.push_back(rmax);

											edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
										}
									}

									if (ed2 < sep)
									{
										Point2f ec1 = findAxisCenter(rmin, bmin, Point2f(254, 254));
										Point2f ec2 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);

										if (dist(ec1, edge_center) < dist(ec2, edge_center))
										{
											edge.push_back(rmin);
											edge.push_back(bmin);

											edge_center = findAxisCenter(edge[0], edge[1], Point2f(254, 254));
										}
										else
										{
											edge.push_back(lmin);
											edge.push_back(lmax);

											edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
										}
									}
								}
							}



							if (!left.empty() && !top.empty() && !right.empty())
							{
								Point lmin = *min_element(left.begin(), left.end(), myfny);
								Point lmax = *max_element(left.begin(), left.end(), myfny);

								Point rmin = *min_element(right.begin(), right.end(), myfny);
								Point rmax = *max_element(right.begin(), right.end(), myfny);

								Point tmin = *min_element(top.begin(), top.end(), myfnx);
								Point tmax = *max_element(top.begin(), top.end(), myfnx);


								float ed1 = findEdgeDistance(lmin, tmin);
								float ed2 = findEdgeDistance(tmax, rmin);

								if ((ed1 < sep) && (ed2 < sep))
								{

									Point2f ec1 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);
									Point2f ec2 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);

									if (dist(edge_center, ec1) < dist(edge_center, ec2))
									{
										edge.push_back(lmin);
										edge.push_back(lmax);
									}
									else
									{
										edge.push_back(rmin);
										edge.push_back(rmax);
									}

									edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);

								}
								else if ((ed1 > sep) && (ed2 > sep))
								{
									Point2f ec1 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);
									Point2f ec2 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);
									Point2f ec3 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);

									float ldist = dist(ec1, edge_center);
									float rdist = dist(ec2, edge_center);
									float tdist = dist(ec3, edge_center);

									float res = ldist;

									edge.push_back(lmin);
									edge.push_back(lmax);

									if (rdist < res)
									{
										edge[0] = rmin;
										edge[1] = rmax;
										res = rdist;
									}

									if (tdist < res)
									{
										edge[0] = tmin;
										edge[1] = tmax;
									}

									edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
								}
								else
								{
									if (ed1 < sep)
									{
										Point2f ec1 = findAxisCenter(lmax, tmax, Point2f(1, 1));
										Point2f ec2 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);

										if (dist(ec1, edge_center) < dist(ec2, edge_center))
										{
											edge.push_back(lmax);
											edge.push_back(tmax);

											edge_center = findAxisCenter(edge[0], edge[1], Point2f(1, 1));
										}
										else
										{
											edge.push_back(rmin);
											edge.push_back(rmax);

											edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
										}
									}

									if (ed2 < sep)
									{
										Point2f ec1 = findAxisCenter(rmax, tmin, Point2f(254, 1));
										Point2f ec2 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);

										if (dist(ec1, edge_center) < dist(ec2, edge_center))
										{
											edge.push_back(rmax);
											edge.push_back(tmin);

											edge_center = findAxisCenter(edge[0], edge[1], Point2f(254, 1));
										}
										else
										{
											edge.push_back(lmin);
											edge.push_back(lmax);

											edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
										}
									}
								}
							}


							if (!top.empty() && !left.empty() && !bottom.empty())
							{
								Point tmin = *min_element(top.begin(), top.end(), myfnx);
								Point tmax = *max_element(top.begin(), top.end(), myfnx);

								Point lmin = *min_element(left.begin(), left.end(), myfny);
								Point lmax = *max_element(left.begin(), left.end(), myfny);

								Point bmin = *min_element(bottom.begin(), bottom.end(), myfnx);
								Point bmax = *max_element(bottom.begin(), bottom.end(), myfnx);

								float ed1 = findEdgeDistance(tmin, lmin);
								float ed2 = findEdgeDistance(lmax, bmin);

								if ((ed1 < sep) && (ed2 < sep))
								{

									Point2f ec1 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);
									Point2f ec2 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

									if (dist(edge_center, ec1) < dist(edge_center, ec2))
									{
										edge.push_back(tmin);
										edge.push_back(tmax);
									}
									else
									{
										edge.push_back(bmin);
										edge.push_back(bmax);
									}

									edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);

								}
								else if ((ed1 > sep) && (ed2 > sep))
								{
									Point2f ec1 = Point2f((lmin.x + lmax.x) / 2, (lmin.y + lmax.y) / 2);
									Point2f ec2 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);
									Point2f ec3 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

									float ldist = dist(ec1, edge_center);
									float tdist = dist(ec2, edge_center);
									float bdist = dist(ec3, edge_center);

									float res = ldist;

									edge.push_back(lmin);
									edge.push_back(lmax);

									if (tdist < res)
									{
										edge[0] = tmin;
										edge[1] = tmax;
										res = tdist;
									}

									if (bdist < res)
									{
										edge[0] = bmin;
										edge[1] = bmax;
									}

									edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
								}
								else
								{
									if (ed1 < sep)
									{
										Point2f ec1 = findAxisCenter(tmax, lmax, Point2f(1, 1));
										Point2f ec2 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

										if (dist(ec1, edge_center) < dist(ec2, edge_center))
										{
											edge.push_back(tmax);
											edge.push_back(lmax);

											edge_center = findAxisCenter(edge[0], edge[1], Point2f(1, 1));
										}
										else
										{
											edge.push_back(bmin);
											edge.push_back(bmax);

											edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
										}
									}

									if (ed2 < sep)
									{
										Point2f ec1 = findAxisCenter(bmax, lmin, Point2f(1, 254));
										Point2f ec2 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);

										if (dist(ec1, edge_center) < dist(ec2, edge_center))
										{
											edge.push_back(bmax);
											edge.push_back(lmin);

											edge_center = findAxisCenter(edge[0], edge[1], Point2f(1, 254));
										}
										else
										{
											edge.push_back(tmin);
											edge.push_back(tmax);

											edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
										}
									}
								}
							}



							if (!top.empty() && !right.empty() && !bottom.empty())
							{
								Point tmin = *min_element(top.begin(), top.end(), myfnx);
								Point tmax = *max_element(top.begin(), top.end(), myfnx);

								Point rmin = *min_element(right.begin(), right.end(), myfny);
								Point rmax = *max_element(right.begin(), right.end(), myfny);

								Point bmin = *min_element(bottom.begin(), bottom.end(), myfnx);
								Point bmax = *max_element(bottom.begin(), bottom.end(), myfnx);


								float ed1 = findEdgeDistance(tmax, rmin);
								float ed2 = findEdgeDistance(rmax, bmax);

								if ((ed1 < sep) && (ed2 < sep))
								{

									Point2f ec1 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);
									Point2f ec2 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

									if (dist(edge_center, ec1) < dist(edge_center, ec2))
									{
										edge.push_back(tmin);
										edge.push_back(tmax);
									}
									else
									{
										edge.push_back(bmin);
										edge.push_back(bmax);
									}

									edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);

								}
								else if ((ed1 > sep) && (ed2 > sep))
								{
									Point2f ec1 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);
									Point2f ec2 = Point2f((rmin.x + rmax.x) / 2, (rmin.y + rmax.y) / 2);
									Point2f ec3 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

									float tdist = dist(ec1, edge_center);
									float rdist = dist(ec2, edge_center);
									float bdist = dist(ec3, edge_center);

									float res = tdist;

									edge.push_back(tmin);
									edge.push_back(tmax);

									if (rdist < res)
									{
										edge[0] = rmin;
										edge[1] = rmax;
										res = rdist;
									}

									if (bdist < res)
									{
										edge[0] = bmin;
										edge[1] = bmax;
									}

									edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
								}
								else
								{
									if (ed1 < sep)
									{
										Point2f ec1 = findAxisCenter(tmin, rmax, Point2f(254, 1));
										Point2f ec2 = Point2f((bmin.x + bmax.x) / 2, (bmin.y + bmax.y) / 2);

										if (dist(ec1, edge_center) < dist(ec2, edge_center))
										{
											edge.push_back(tmin);
											edge.push_back(rmax);

											edge_center = findAxisCenter(edge[0], edge[1], Point2f(254, 1));
										}
										else
										{
											edge.push_back(bmin);
											edge.push_back(bmax);

											edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
										}
									}

									if (ed2 < sep)
									{
										Point2f ec1 = findAxisCenter(bmin, rmin, Point2f(254, 254));
										Point2f ec2 = Point2f((tmin.x + tmax.x) / 2, (tmin.y + tmax.y) / 2);

										if (dist(ec1, edge_center) < dist(ec2, edge_center))
										{
											edge.push_back(bmin);
											edge.push_back(rmin);

											edge_center = findAxisCenter(edge[0], edge[1], Point2f(254, 254));
										}
										else
										{
											edge.push_back(tmin);
											edge.push_back(tmax);

											edge_center = Point2f((edge[0].x + edge[1].x) / 2, (edge[0].y + edge[1].y) / 2);
										}
									}
								}
							}
						}
						
						// fly makes contact with four edges
						if (sum == 4)
						{
							Point tmin = *min_element(top.begin(), top.end(), myfnx);
							Point tmax = *max_element(top.begin(), top.end(), myfnx);

							Point lmin = *min_element(left.begin(), left.end(), myfny);
							Point lmax = *max_element(left.begin(), left.end(), myfny);

							Point rmin = *min_element(right.begin(), right.end(), myfny);
							Point rmax = *max_element(right.begin(), right.end(), myfny);

							Point bmin = *min_element(bottom.begin(), bottom.end(), myfnx);
							Point bmax = *max_element(bottom.begin(), bottom.end(), myfnx);

							float ed1 = findEdgeDistance(tmin, lmin);
							float ed2 = findEdgeDistance(tmax, rmin);

							if (ed1 < ed2)
							{
								Point2f ec1 = findAxisCenter(tmax, lmax, Point2f(1, 1));
								Point2f ec2 = findAxisCenter(bmin, rmin, Point2f(1, 1));

								if (dist(edge_center, ec1) < dist(edge_center, ec2))
								{
									edge.push_back(tmax);
									edge.push_back(lmax);

									edge_center = findAxisCenter(edge[0], edge[1], Point2f(1, 1));
								}
								else
								{
									edge.push_back(bmin);
									edge.push_back(rmin);

									edge_center = findAxisCenter(edge[0], edge[1], Point2f(254, 254));
								}
							}
							else
							{
								Point2f ec1 = findAxisCenter(tmin, rmax, Point2f(1, 1));
								Point2f ec2 = findAxisCenter(lmin, bmax, Point2f(1, 1));

								if (dist(edge_center, ec1) < dist(edge_center, ec2))
								{
									edge.push_back(tmin);
									edge.push_back(rmax);

									edge_center = findAxisCenter(edge[0], edge[1], Point2f(254, 1));
								}
								else
								{
									edge.push_back(lmin);
									edge.push_back(bmax);

									edge_center = findAxisCenter(edge[0], edge[1], Point2f(1, 254));
								}
							}
						}


						
						if (edge.size() == 2)
						{
							Point2f body_center = fly_mc[j];
							//line(fly_frame, body_center, edge_center, Scalar::all(255), 1, 8, 0);

							Point2f p1, p2;
							Point2f p1a, p1b, p2a, p2b;

							float m, c;

							// Check for vertical line (same x coordinate) because vertical lines don't have a slope
							if (body_center.x != edge_center.x)
							{
								p1a.x = 0;
								p2a.x = fly_image_width;

								p1b.y = 0;
								p2b.y = fly_image_height;

								// Slope equation (y1 - y2) / (x1 - x2)
								m = (body_center.y - edge_center.y) / (body_center.x - edge_center.x);

								// Line equation:  y = mx + c
								c = body_center.y - (m * body_center.x);


								p1a.y = m * p1a.x + c;
								p2a.y = m * p2a.x + c;

								p1b.x = (p1b.y - c) / m;
								p2b.x = (p2b.y - c) / m;

								if (dist(p1a, Point2f(fly_image_width / 2, fly_image_height / 2)) <= dist(p1b, Point2f(fly_image_width / 2, fly_image_height / 2)))
									p1 = p1a;
								else
									p1 = p1b;

								if (dist(p2a, Point2f(fly_image_width / 2, fly_image_height / 2)) <= dist(p2b, Point2f(fly_image_width / 2, fly_image_height / 2)))
									p2 = p2a;
								else
									p2 = p2b;
							}
							else
							{
								p1.x = body_center.x;
								p2.x = body_center.x;

								p1.y = 0;
								p2.y = fly_image_height;
							}

							vector<Point2f> head;
							for (int i = 1; i <= hull[j].size(); i++)
							{
								float x, y;
								Point2f r, s;

								if (i < hull[j].size())
								{
									r = hull[j][i - 1];
									s = hull[j][i];
								}
								else
								{
									r = hull[j][i - 1];
									s = hull[j][0];

								}

								bool cross = get_line_intersection(p1.x, p1.y, p2.x, p2.y, r.x, r.y, s.x, s.y, &x, &y);
							
								if (cross)
									head.push_back(Point2f(x, y));
							}

							if (head.size() > 0)
							{
								int head_index = findFurthestPoint(edge_center, head);

								Point2f a = head[head_index];
								Point2f b = edge_center;

								Point2f h;

								if (a.x != b.x)
								{
									// Slope equation (y1 - y2) / (x1 - x2)
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
								
								Point2f rotpt = rotateFlyCenter(pt2d, fly_image_width, fly_image_height);

								float diffx = rotpt.x - (fly_image_width / 2);
								float diffy = rotpt.y - (fly_image_height / 2);

								ndq.ConvertPixelToDeg(diffx*SCALEX, diffy*SCALEY);
								wpt = ndq.ConvertDegToPt();
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
							
							pt2d = Point2f(0, 0);
							wpt = Point2f(0, 0);

							//for (int i = 0; i < NFLIES; i++)
							//	tkf[i].Init();
						}
					}
				}
					
				//Calculate frame rate
				fly_fps = ConvertTimeToFPS(fly_stamp.cycleCount, fly_ltime);
				fly_ltime = fly_stamp.cycleCount;

				putText(fly_frame, to_string(fly_fps), Point((fly_image_width - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
				
				if (flyview_record)
					putText(fly_frame, to_string(rcount), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

				#pragma omp critical
				{
					flyMaskStream.push(fly_mask);
					flyDispStream.push(fly_frame);

					if (flyview_record)
					{
						laser_pt.push(wpt);
						fly_pt.push(pt2d);

						flyTimeStamps.push(fly_stamp);
						flyImageStream.push(fly_img);
					}
				}

				if (GetAsyncKeyState(VK_F1))
				{
					if (!track_key_state)
					{
						flyview_track = !flyview_track;

						if (flyview_record)
							flyview_record = !flyview_record;
					}

					track_key_state = 1;
				}
				else
					track_key_state = 0;

				if (GetAsyncKeyState(VK_F2))
				{
					if (!record_key_state)
					{
						flyview_record = !flyview_record;
						rcount = 0;
					}

					record_key_state = 1;
				}
				else
					record_key_state = 0;

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
			int arena_ltime = 0;
			int arena_fps = 0;

			while (true)
			{
				arena_img = arena_cam.GrabFrame();
				arena_stamp = arena_cam.GetTimeStamp();
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

					vector<Point2f> arena_ctr_pts;
					
					for (int i = 0; i < arena_contours.size(); i++)
					{
						//drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
						arena_mu[i] = moments(arena_contours[i], false);
						arena_mc[i] = Point2f(arena_mu[i].m10 / arena_mu[i].m00, arena_mu[i].m01 / arena_mu[i].m00);

						arena_ctr_pts.push_back(arena_mc[i]);

						//circle(arena_frame, arena_mc[i], 1, Scalar(255, 255, 255), FILLED, 1);
						//arena_pts.push_back(backProject(arena_mc[i], cameraMatrix, rotationMatrix, tvec, BASE_HEIGHT));

						////fly z correction code
						//float z = 0;
						//float flydist = 0;
						//Point2f fly_pos;

						//while (flydist < GALVO_HEIGHT)
						//{
						//	fly_pos = backProject(arena_mc[i], cameraMatrix, rotationMatrix, tvec, z += 0.025);
						//	flydist = dist3d(galvo_center, Point3f(fly_pos.x, fly_pos.y, z));
						//}

						//float xang = asin(fly_pos.x / GALVO_HEIGHT);
						//float xdiff = (z - BASE_HEIGHT) * tan(xang);
						//fly_pos.x -= xdiff;

						//float yang = asin(fly_pos.y / GALVO_HEIGHT);
						//float ydiff = (z - BASE_HEIGHT) * tan(yang);
						//fly_pos.y -= ydiff;

						//arena_pt.push_back(fly_pos);

					}

					for (int i = 0; i < NFLIES; i++)
					{
						if (arena_ctr_pts.size() > 0)
						{
							int j = findClosestPoint(arena_pt[i], arena_ctr_pts);

							arena_pt[i] = arena_ctr_pts[j];
							pt[i] = backProject(arena_pt[i], cameraMatrix, rotationMatrix, tvec, BASE_HEIGHT);
							
							//tkf[i].Correct(pt[i]);

							arena_pt_vec[i].push_back(arena_pt[i]);

							for (int k = 0; k < arena_pt_vec[i].size() - 1; k++)
								line(arena_frame, arena_pt_vec[i][k], arena_pt_vec[i][k + 1], Scalar(255, 255, 0), 1);

							if (arena_pt_vec[i].size() > TAIL_LENGTH)
								arena_pt_vec[i].erase(arena_pt_vec[i].begin());

							arena_ctr_pts.erase(arena_ctr_pts.begin() + j);
						}
					}
					
					if (!flyview_track)
					{
						ndq.ConvertPtToDeg(pt[0]);
						ndq.write();
					}
				}

				//Calculate frame rate
				arena_fps = ConvertTimeToFPS(arena_stamp.cycleCount, arena_ltime);
				arena_ltime = arena_stamp.cycleCount;

				putText(arena_frame, to_string(arena_fps), Point((arena_image_width - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

				#pragma omp critical
				{
					arenaMaskStream.push(arena_mask);
					arenaDispStream.push(arena_frame);
				}

				if (!stream)
					break;

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

					#pragma omp critical
					{
						fout.WriteFrame(flyImageStream.front());
						fout.WriteLog(flyTimeStamps.front());
						fout.WriteTraj(laser_pt.front(), fly_pt.front());
						fout.nframes++;
						
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
				#pragma omp critical
				{
					if (!arenaDispStream.empty())
					{
						ellipse(arenaDispStream.front(), arenaMask, Scalar(255, 255, 255));
						imshow("arena image", arenaDispStream.front());
						imshow("arena mask", arenaMaskStream.front());
					}

					arenaDispStream = {};
					arenaMaskStream = {};
				}

				waitKey(1);

				if (!stream)
				{
					destroyWindow("controls");
					destroyWindow("arena image");
					destroyWindow("arena mask");

					break;
				}
			}
		}


		#pragma omp section
		{
			while (true)
			{
				#pragma omp critical
				{
					if (!flyDispStream.empty())
					{
						imshow("fly image", flyDispStream.front());
						imshow("fly mask", flyMaskStream.front());
					}

					flyDispStream = {};
					flyMaskStream = {};
				}

				waitKey(1);

				if (!stream)
				{
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
