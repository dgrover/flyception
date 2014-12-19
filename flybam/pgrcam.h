#ifndef PGRCAM_H
#define PGRCAM_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

class PGRcam
{
private:
	Camera cam;
	CameraInfo camInfo;

	Image rawImage, convertedImage;
	TimeStamp timestamp;

	FlyCapture2::Error error;

	Mode k_fmt7Mode;
	PixelFormat k_fmt7PixFmt;

	Format7Info fmt7Info;
	bool supported;

	Format7ImageSettings fmt7ImageSettings;
	Format7PacketInfo fmt7PacketInfo;

	TriggerModeInfo triggerModeInfo;
	TriggerMode triggerMode;

	bool valid;


public:

	PGRcam();
	~PGRcam();

	FlyCapture2::Error Connect(PGRGuid guid);
	FlyCapture2::Error SetCameraParameters(int width, int height);
	FlyCapture2::Error SetTrigger();
	FlyCapture2::Error SetProperty(PropertyType type, float absValue);
	FlyCapture2::Error Start();
	FlyCapture2::Error Stop();
	FlyCapture2::Image GrabFrame();
	Mat convertImagetoMat(Image img);
	FlyCapture2::TimeStamp GetTimeStamp();
	void GetImageSize(int &imageWidth, int &imageHeight);
};

#endif
