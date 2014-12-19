#include "stdafx.h"
#include "pgrcam.h"

PGRcam::PGRcam()
{
	k_fmt7Mode = MODE_0;
	k_fmt7PixFmt = PIXEL_FORMAT_RAW8;
}

PGRcam::~PGRcam()
{}

FlyCapture2::Error PGRcam::Connect(PGRGuid guid)
{
	// Connect to a camera
	error = cam.Connect(&guid);

	return error;
}

FlyCapture2::Error PGRcam::SetCameraParameters(int width, int height)
{
	// Get the camera information
	error = cam.GetCameraInfo(&camInfo);

	// Query for available Format 7 modes
	fmt7Info.mode = k_fmt7Mode;
	error = cam.GetFormat7Info(&fmt7Info, &supported);

	// Pixel format not supported!
	if ((k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0)
		printf("Pixel format is not supported\n");

	fmt7ImageSettings.mode = k_fmt7Mode;
	fmt7ImageSettings.offsetX = fmt7Info.maxWidth / 2 - width / 2;
	fmt7ImageSettings.offsetY = fmt7Info.maxHeight / 2 - height / 2;	
	fmt7ImageSettings.width = width;			
	fmt7ImageSettings.height = height;			
	fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

	// Validate the settings to make sure that they are valid
	error = cam.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);

	if (!valid)
		printf("Format7 settings are not valid\n");

	// Set the settings to the camera
	error = cam.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);

	return error;
}

FlyCapture2::Error PGRcam::SetTrigger()
{
	// Check for external trigger support
	error = cam.GetTriggerModeInfo(&triggerModeInfo);

	if (triggerModeInfo.present != true)
		printf("Camera does not support external trigger! Exiting...\n");

	// Get current trigger settings
	error = cam.GetTriggerMode(&triggerMode);

	// Set camera to trigger mode 0
	triggerMode.onOff = true;
	triggerMode.mode = 0;
	triggerMode.parameter = 0;

	// Triggering the camera externally using source 0.
	triggerMode.source = 0;

	error = cam.SetTriggerMode(&triggerMode);

	return error;
}

FlyCapture2::Error PGRcam::SetProperty(FlyCapture2::PropertyType type, float absValue)
{
	FlyCapture2::Property pProp;

	pProp.type = type;
	pProp.absControl = true;
	pProp.onePush = false;
	pProp.onOff = true;
	pProp.autoManualMode = false;
	pProp.absValue = absValue;

	error = cam.SetProperty(&pProp);

	return error;
}

FlyCapture2::Error PGRcam::Start()
{
	// Start capturing images
	error = cam.StartCapture();

	return error;
}

FlyCapture2::Error PGRcam::Stop()
{
	// Stop capturing images
	error = cam.StopCapture();

	// Disconnect the camera
	error = cam.Disconnect();

	return error;
}

FlyCapture2::Image PGRcam::GrabFrame()
{
	// Retrieve an image
	error = cam.RetrieveBuffer(&rawImage);

	//get image timestamp
	timestamp = rawImage.GetTimeStamp();

	// Convert the raw image
	error = rawImage.Convert(PIXEL_FORMAT_MONO8, &convertedImage);

	return convertedImage;
}

Mat PGRcam::convertImagetoMat(Image img)
{
	// convert to OpenCV Mat
	unsigned int rowBytes = (double)img.GetReceivedDataSize() / (double)img.GetRows();
	Mat tframe = Mat(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), rowBytes);

	Mat frame = tframe.clone();

	return frame;
}

void PGRcam::GetImageSize(int &imageWidth, int &imageHeight)
{
		imageWidth = fmt7ImageSettings.width;
		imageHeight = fmt7ImageSettings.height;
}

FlyCapture2::TimeStamp PGRcam::GetTimeStamp()
{
	return timestamp;
}
