#include "stdafx.h"
#include "tracker.h"

Tracker::Tracker(int x, int y)
{
    measurement.setTo(Scalar(0));

    KF.statePre.at<float>(0) = x;
    KF.statePre.at<float>(1) = y;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));

    mousev.clear();
    kalmanv.clear();
}

void Tracker::Predict(int x, int y)
{
    Mat prediction = KF.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

    measurement(0) = x;
    measurement(1) = y;

    Point measPt(measurement(0),measurement(1));
    mousev.push_back(measPt);
}

void Tracker::Correct()
{
    Mat estimated = KF.correct(measurement);
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    kalmanv.push_back(statePt);
}

void Tracker::ConvertPixelToVoltage(float64 dataX[], float64 dataY[])
{
    int imageWidth = 512;
    int imageHeight = 512;

    int maxVoltage = 3.0;

    int size = kalmanv.size();

    dataX[0] = ( kalmanv[size-1].x / imageWidth * maxVoltage ) - maxVoltage/2;
    dataY[0] = ( kalmanv[size-1].y / imageHeight * maxVoltage ) - maxVoltage/2;
}
