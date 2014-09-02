// projection.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

#define HEIGHT_FROM_BASE 0.0

std::vector<cv::Point3f> create3DChessboardCorners(cv::Size boardSize, float squareSize)
{
	// create the 3D points of your chessboard in its own coordinate system
	std::vector<cv::Point3f> corners;

	//for (int i = 0; i < boardSize.height; i++)
	//{
	//	for (int j = 0; j < boardSize.width; j++)
	//	{
	//		corners.push_back(cv::Point3f(float(j*squareSize),
	//			float(i*squareSize), 0));
	//	}
	//}

	for (int i = -boardSize.height / 2; i <= boardSize.height / 2; i++)
	{
		for (int j = -boardSize.width / 2; j <= boardSize.width / 2; j++)
		{
			corners.push_back(cv::Point3f(float(j*squareSize),
				float(i*squareSize), HEIGHT_FROM_BASE));
		}
	}

	return corners;
}

int _tmain(int argc, _TCHAR* argv[])
{
	string infilename = "..\\..\\arena\\out_camera_data.xml";
	string outfilename = "..\\..\\arena\\camera_projection_data.xml";
	string imgfile = "..\\..\\arena\\cbview.jpg";

	int boardHeight, boardWidth;
	float squareSize;

	Mat img = imread(imgfile);

	Mat cameraMatrix, distCoeffs;

	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point3f> objectPoints;

	cv::Mat rvec(1, 3, cv::DataType<double>::type);
	cv::Mat tvec(1, 3, cv::DataType<double>::type);
	cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);

	FileStorage fs(infilename, FileStorage::READ);

	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;
	fs["board_Height"] >> boardHeight;
	fs["board_Width"] >> boardWidth;
	fs["square_Size"] >> squareSize;

	fs.release();

	Size cbSize = Size(boardHeight, boardWidth);

	bool found = findChessboardCorners(img, cbSize, imagePoints, CALIB_CB_FAST_CHECK);

	if (found)
	{

		drawChessboardCorners(img, cbSize, imagePoints, found);
		imshow("image", img);

		objectPoints = create3DChessboardCorners(cbSize, squareSize);

		cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
		cv::Rodrigues(rvec, rotationMatrix);

		//show the pose estimation data
		cout << fixed << setprecision(2) << "rvec = ["
			<< rvec.at<double>(0, 0) << ", "
			<< rvec.at<double>(1, 0) << ", "
			<< rvec.at<double>(2, 0) << "] \t" << "tvec = ["
			<< tvec.at<double>(0, 0) << ", "
			<< tvec.at<double>(1, 0) << ", "
			<< tvec.at<double>(2, 0) << "]" << endl;

		FileStorage fs(outfilename, FileStorage::WRITE);

		fs << "Camera_Matrix" << cameraMatrix;
		fs << "Distortion_Coefficients" << distCoeffs;
		fs << "rvec" << rvec;
		fs << "tvec" << tvec;
		fs << "Rotation_Matrix" << rotationMatrix;

		fs.release();
	}

	waitKey(0);

	return 0;
}

