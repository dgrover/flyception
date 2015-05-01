#ifndef TRACKER_H
#define TRACKER_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

class Tracker
{
    private:

		KalmanFilter KF;
		Mat_<float> measurement, prediction, estimated;
    
	public:

		Tracker();
		~Tracker();

		void Init(Point2f pt);
		Point2f Predict();
		Point2f Correct(Point2f measPt);
};

#endif
