#include "stdafx.h"
#include "utility.h"

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

//RotatedRect createArenaMask(int arena_radius, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
//{
//	Point2f center(0, 0);
//
//	vector<Point3f> c3d;
//	vector<Point2f> c2d;
//
//	RotatedRect circleMask;
//
//	for (double angle = 0; angle <= 2 * CV_PI; angle += 0.001) //You are using radians so you will have to increase by a very small amount
//		c3d.push_back(Point3f(center.x + arena_radius*cos(angle), center.y + arena_radius*sin(angle), BASE_HEIGHT));
//
//	projectPoints(c3d, rvec, tvec, cameraMatrix, distCoeffs, c2d);
//
//	circleMask = fitEllipse(c2d);
//
//	return circleMask;
//}

RotatedRect createArenaMask(float x_rad, float y_rad, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
{
	Point2f center(0, 0);

	Point3f ellipsePath;

	vector<Point3f> c3d;
	vector<Point2f> c2d;

	RotatedRect ellipseMask;

	for (double angle = 0; angle <= 2 * CV_PI; angle += 0.001) //You are using radians so you will have to increase by a very small amount
	{
		if ( (angle >= 0 && angle < 90) || (angle > 270 && angle <= 360) )
		{
			ellipsePath.x = (x_rad*y_rad) / (sqrt((y_rad*y_rad) + x_rad*x_rad*tan(angle)*tan(angle)));
			ellipsePath.y = (x_rad*y_rad*tan(angle)) / (sqrt((y_rad*y_rad) + x_rad*x_rad*tan(angle)*tan(angle)));
			ellipsePath.z = BASE_HEIGHT;

			c3d.push_back(ellipsePath);
		}

		if (angle > 90 && angle < 270)
		{
			ellipsePath.x = -(x_rad*y_rad) / (sqrt((y_rad*y_rad) + x_rad*x_rad*tan(angle)*tan(angle)));
			ellipsePath.y = -(x_rad*y_rad*tan(angle)) / (sqrt((y_rad*y_rad) + x_rad*x_rad*tan(angle)*tan(angle)));
			ellipsePath.z = BASE_HEIGHT;

			c3d.push_back(ellipsePath);
		}
	}

	projectPoints(c3d, rvec, tvec, cameraMatrix, distCoeffs, c2d);

	ellipseMask = fitEllipse(c2d);

	return ellipseMask;
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
