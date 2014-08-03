#ifndef TRACKER_H
#define TRACKER_H

using namespace cv;
using namespace std;

class Tracker
{
    private:

      KalmanFilter KF(4, 2, 0);
      Mat_<float> state(4, 1);          // (x, y, Vx, Vy)
      Mat processNoise(4, 1, CV_32F);
      Mat_<float> measurement(2,1);



    public:

      vector<Point> mousev, kalmanv;

      Tracker(int x, int y);
      ~Tracker();
      void Predict(int x, int y);
      void Correct();
      void ConvertPixelToVoltage(float64 dataX[], float64 dataY[]);
};

#endif
