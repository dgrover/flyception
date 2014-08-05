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

    mmtv.clear();
    predv.clear();
    estv.clear();
}

Tracker::~Tracker()
{}

void Tracker::Predict(int x, int y)
{
    Mat prediction = KF.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
    predv.push_back(predictPt);

    measurement(0) = x;
    measurement(1) = y;

    Point measPt(measurement(0),measurement(1));
    mmtv.push_back(measPt);
}

void Tracker::Correct()
{
    Mat estimated = KF.correct(measurement);
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    estv.push_back(statePt);
}

void Tracker::ConvertPixelToVoltage(int imageWidth, int imageHeight, int maxVoltage, float64 dataX[], float64 dataY[])
{
    int size = estv.size();

    dataX[0] = ( (float)(estv[size-1].x) / imageWidth * maxVoltage ) - maxVoltage/2;
    dataY[0] = ( (float)(estv[size-1].y) / imageHeight * maxVoltage ) - maxVoltage/2;
}
