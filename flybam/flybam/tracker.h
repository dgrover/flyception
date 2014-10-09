#ifndef TRACKER_H
#define TRACKER_H

using namespace cv;
using namespace std;

class Tracker
{
    private:

		KalmanFilter KF;
		Mat_<float> measurement, prediction, estimated;
    
	public:

		Tracker();
		~Tracker();

		void Init(float x = 0, float y = 0);
		cv::Mat Predict();
		cv::Mat Correct(cv::Mat measPt);
};

#endif
