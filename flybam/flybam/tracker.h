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

	  //Point2f Predict();
	  //Point2f Correct(Point2f measPt);

      cv::Mat Predict();
      cv::Mat Correct(cv::Mat measPt);
};

#endif
