#include "stdafx.h"
#include "flycam.h"

Flycam::Flycam()
{
	k_fmt7Mode = MODE_0;
	k_fmt7PixFmt = PIXEL_FORMAT_RAW8;
}

Flycam::~Flycam()
{}

Error Flycam::Connect(PGRGuid guid)
{
	// Connect to a camera
	error = cam.Connect(&guid);

	return error;
}

Error Flycam::SetCameraParameters(int offsetX, int offsetY, int width, int height)
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
	fmt7ImageSettings.offsetX = offsetX;
	fmt7ImageSettings.offsetY = offsetY;
	fmt7ImageSettings.width = width;			// fmt7Info.maxWidth;
	fmt7ImageSettings.height = height;			// fmt7Info.maxHeight;
	fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

	// Validate the settings to make sure that they are valid
	error = cam.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);

	if (!valid)
		printf("Format7 settings are not valid\n");

	// Set the settings to the camera
	error = cam.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);

	return error;
}

Error Flycam::Start()
{
	// Start capturing images
	error = cam.StartCapture();

	return error;
}

Error Flycam::Stop()
{
	// Stop capturing images
	error = cam.StopCapture();

	// Disconnect the camera
	error = cam.Disconnect();

	return error;
}

Mat Flycam::GrabFrame()
{
	// Retrieve an image
	error = cam.RetrieveBuffer(&rawImage);

	//get image timestamp
	timestamp = rawImage.GetTimeStamp();

	// Convert the raw image
	error = rawImage.Convert(PIXEL_FORMAT_MONO8, &convertedImage);

	// convert to OpenCV Mat
	unsigned int rowBytes = (double)convertedImage.GetReceivedDataSize() / (double)convertedImage.GetRows();
	Mat frame = Mat(convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC1, convertedImage.GetData(), rowBytes);

	return frame;
}

void Flycam::GetImageSize(int &imageWidth, int &imageHeight)
{
		imageWidth = fmt7ImageSettings.width;
		imageHeight = fmt7ImageSettings.height;
}
