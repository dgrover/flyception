#include "stdafx.h"
#include "tracker.h"

Tracker::Tracker()
{
    KF.init(4, 2, 0);
	
	measurement.create(2,1);
	measurement.setTo(Scalar(0));

    KF.statePre.at<float>(0) = -1;
    KF.statePre.at<float>(1) = -1;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-2));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
}

Tracker::~Tracker()
{}

Point2f Tracker::Predict()
{
    prediction = KF.predict();
    Point2f predictPt(prediction.at<float>(0),prediction.at<float>(1));
    
	return predictPt;
}

Point2f Tracker::Correct(Point2f measPt)
{
	measurement(0) = measPt.x;
	measurement(1) = measPt.y;

    estimated = KF.correct(measurement);

	Point2f statePt(estimated.at<float>(0), estimated.at<float>(1));

	return statePt;
}
