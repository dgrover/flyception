#ifndef FLYCAM_H
#define FLYCAM_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

class Flycam
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

	bool valid;


public:

	Flycam();
	~Flycam();

	FlyCapture2::Error Connect(PGRGuid guid);
	FlyCapture2::Error SetCameraParameters(int width, int height);
	FlyCapture2::Error Start();
	FlyCapture2::Error Stop();
	FlyCapture2::Image GrabFrame();
	Mat convertImagetoMat(Image img);
	FlyCapture2::TimeStamp GetTimeStamp();
	void GetImageSize(int &imageWidth, int &imageHeight);
};

#endif
