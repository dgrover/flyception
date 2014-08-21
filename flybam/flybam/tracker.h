#ifndef TRACKER_H
#define TRACKER_H

using namespace cv;
using namespace std;

class Tracker
{
    private:

      KalmanFilter KF;
	  Mat_<float> measurement;
	  vector<Point> mt, pred, est;

    public:

      Tracker();
      ~Tracker();

      void Predict(int x, int y);
      void Correct();
	  void GetTrackedPoint(Point2f &p);
};

#endif
