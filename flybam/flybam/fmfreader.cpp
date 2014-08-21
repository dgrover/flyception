#include "stdafx.h"
#include "fmfreader.h"

FmfReader::FmfReader()
{}

FmfReader::~FmfReader()
{}

void FmfReader::GetFileExtension(const string &fname)
{
	if (fname.find_last_of(".") != string::npos)
		fext = fname.substr(fname.find_last_of(".") + 1);
	else 
		fext = "";
}

int FmfReader::Open(_TCHAR* fname)
{
	fp = fopen(fname, "rb");

	if(fp == NULL) // Cannot open file
	{
		printf("Cannot open input file\n");
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
	if (fext == "fmf")
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
			bytesPerChunk - sizeof(double),
			nframes);
	}

	return 1;
}

int FmfReader::GetFrameCount()
{
	if (fext == "txt")
	{
		nframes = 0;
		int ch;

		while (!feof(fp))
		{
			ch = fgetc(fp);
			if (ch == '\n')
				nframes++;
		}

		//seek to beginning of file after counting number of lines
		rewind(fp);
	}

	return nframes;
}

void FmfReader::GetImageSize(int &imageWidth, int &imageHeight)
{
	if (fext == "fmf")
	{
		imageWidth = SizeX;
		imageHeight = SizeY;
	}
	else if (fext == "txt")
	{
		imageWidth = 512;
		imageHeight = 512;
	}
}

Mat FmfReader::ReadFrame(unsigned long frameIndex)
{
	if ((long)frameIndex >= 0L && (long)frameIndex < maxFramesInFile)
		fseek(fp, frameIndex*bytesPerChunk + 28, SEEK_SET);
	
	fread(buf, sizeof(double), 1, fp);
	fread(buf, bytesPerChunk - sizeof(double), 1, fp);

	Mat frame = Mat(SizeY, SizeX, CV_8UC1, buf, (bytesPerChunk - sizeof(double)) / SizeY); //byte size of each row of frame
	return frame;
}

int FmfReader::ReadFrame()
{
	if (fscanf(fp, "%f %f\n", &x, &y) == 2)
		return 1;
	else
		return -1;
}
