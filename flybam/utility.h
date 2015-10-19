#ifndef UTILITY_H
#define UTILITY_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

bool myfny(Point p1, Point p2);
bool myfnx(Point p1, Point p2);
Point2f rotateFlyCenter(Point2f p, int image_width, int image_height);
Point2f backProject(Point2f p, Mat cameraMatrix, Mat rotationMatrix, Mat tvec, float height);
Point2f project3d2d(Point2f pt, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec);
//RotatedRect createArenaMask(int arena_radius, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec);
RotatedRect createArenaMask(float x_rad, float y_rad, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec);
float dist(Point2f p1, Point2f p2);
float dist3d(Point3f p1, Point3f p2);
int findClosestPoint(Point2f pt, vector<Point2f> nbor);
int findFurthestPoint(Point2f pt, vector<Point2f> nbor);
bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y);
Point2f findAxisCenter(Point2f p1, Point2f p2, Point2f origin);
float findEdgeDistance(Point2f p1, Point2f p2);
int ConvertTimeToFPS(int ctime, int ltime);

#endif
