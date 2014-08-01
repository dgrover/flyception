#include "stdafx.h"
#include "fmfreader.h"

int FmfReader::Open(_TCHAR* fname)
{
	fp = fopen(fname, "rb");

	if(fp == NULL) // Cannot open File
	{
		printf("Cannot open input video file\n");
		return -1;	
	}

	return 1;

}

int FmfReader::Close()
{
	if (fp != NULL)
		fclose(fp);
	else
	{
		printf("File not open\n");
		return -1;
	}

	return 1;
}

int FmfReader::ReadHeader()
{
	fread(&fmfVersion, sizeof(unsigned __int32), 1, fp);
	fread(&SizeY, sizeof(unsigned __int32), 1, fp);
	fread(&SizeX, sizeof(unsigned __int32), 1, fp);
	fread(&bytesPerChunk, sizeof(unsigned __int64), 1, fp);
	fread(&nframes, sizeof(unsigned __int64), 1, fp);

	buf = new char[bytesPerChunk];
			
	maxFramesInFile = (unsigned long int)nframes;

	printf(
		"\n*** VIDEO INFORMATION ***\n"
		"FMF Version: %d\n"
		"Height: %d\n"
		"Width: %d\n"
		"Frame Size: %d\n"
		"Number of Frames: %d\n",
		fmfVersion, 
		SizeY, 
		SizeX, 
		bytesPerChunk-sizeof(double), 
		nframes);

	return 1;
}

int FmfReader::ReadFrame(unsigned long frameIndex)
{
	if((long)frameIndex>=0L && (long)frameIndex < maxFramesInFile)
		fseek (fp, frameIndex*bytesPerChunk + 28 , SEEK_SET );
	else
		return -1; // Cannot grab .. illegal frame number

	fread(buf, sizeof(double), 1, fp);
	fread(buf, bytesPerChunk-sizeof(double), 1, fp);
	
	return 1;	
}

Mat FmfReader::ConvertToCvMat()
{
	Mat frame = Mat(SizeY, SizeX, CV_8UC1, buf, (bytesPerChunk-sizeof(double))/SizeY); //byte size of each row of frame
	
	return frame;
}