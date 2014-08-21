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

    mt.clear();
    pred.clear();
    est.clear();
}

Tracker::~Tracker()
{}

void Tracker::Predict(int x, int y)
{
    Mat prediction = KF.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
    pred.push_back(predictPt);

    measurement(0) = x;
    measurement(1) = y;

    Point measPt(measurement(0),measurement(1));
    mt.push_back(measPt);
}

void Tracker::Correct()
{
    Mat estimated = KF.correct(measurement);
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    est.push_back(statePt);
}

void Tracker::GetTrackedPoint(Point2f &p)
{
	int size = est.size();

	p.x = est[size - 1].x;
	p.y = est[size - 1].y;

}

