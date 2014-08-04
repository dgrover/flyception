#ifndef TRACKER_H
#define TRACKER_H

using namespace cv;
using namespace std;

class Tracker
{
    private:

      KalmanFilter KF;
	  Mat_<float> measurement;
      
    public:

      vector<Point> mmtv, predv, estv;

      Tracker(int x, int y);
      ~Tracker();

      void Predict(int x, int y);
      void Correct();
      void ConvertPixelToVoltage(int imageWidth, int imageHeight, int maxVoltage, float64 dataX[], float64 dataY[]);
};

#endif
