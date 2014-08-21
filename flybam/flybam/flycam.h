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

	Error error;

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

	Error Connect(PGRGuid guid);
	Error SetCameraParameters();
	Error Start();
	Error Stop();
	Mat GrabFrame();
	void GetImageSize(int &imageWidth, int &imageHeight);
};

#endif
