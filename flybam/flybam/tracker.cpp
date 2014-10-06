#include "stdafx.h"
#include "tracker.h"

Tracker::Tracker(float x, float y)
{
    KF.init(4, 2, 0);
	
	measurement.create(2,1);
	measurement.setTo(Scalar(0));

    KF.statePre.at<float>(0) = x;
    KF.statePre.at<float>(1) = y;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

	setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));		//adjust this for faster convergence - but higher noise (default: 1e-2)
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(0.1));
}

Tracker::~Tracker()
{}

//Point2f Tracker::Predict()
//{
//    prediction = KF.predict();
//    Point2f predictPt(prediction.at<float>(0),prediction.at<float>(1));
//    
//	return predictPt;
//}
//
//Point2f Tracker::Correct(Point2f measPt)
//{
//	measurement(0) = measPt.x;
//	measurement(1) = measPt.y;
//
//    estimated = KF.correct(measurement);
//
//	Point2f statePt(estimated.at<float>(0), estimated.at<float>(1));
//
//	return statePt;
//}

cv::Mat Tracker::Predict()
{
	prediction = KF.predict();
	
	cv::Mat predictPt = cv::Mat::ones(2, 1, cv::DataType<double>::type);
	predictPt.at<double>(0, 0) = (double) prediction.at<float>(0);
	predictPt.at<double>(1, 0) = (double) prediction.at<float>(1);
	//predictPt.at<double>(2, 0) = 0;

	//KF.statePre.copyTo(KF.statePost);
	//KF.errorCovPre.copyTo(KF.errorCovPost);

	return predictPt;
}

cv::Mat Tracker::Correct(cv::Mat measPt)
{
	measurement(0) = (float) measPt.at<double>(0, 0);
	measurement(1) = (float)measPt.at<double>(1, 0);

	estimated = KF.correct(measurement);

	cv::Mat statePt = cv::Mat::ones(2, 1, cv::DataType<double>::type);
	statePt.at<double>(0, 0) = (double)estimated.at<float>(0);
	statePt.at<double>(1, 0) = (double)estimated.at<float>(1);

	return statePt;
}