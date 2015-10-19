#include "stdafx.h"
#include "fvfmfwriter.h"

FVFmfWriter::FVFmfWriter()
{
	fp = NULL;
	flog = NULL;
	ftraj = NULL;

	nframes = 0;
}

int FVFmfWriter::Open()
{
	fp = new FILE;
	flog = new FILE;
	ftraj = new FILE;

	SYSTEMTIME st;
	GetLocalTime(&st);

	sprintf_s(fname, "D:\\fv-%d%02d%02dT%02d%02d%02d.fmf", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
	remove(fname);

	sprintf_s(flogname, "D:\\fv-log-%d%02d%02dT%02d%02d%02d.txt", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
	remove(flogname);

	sprintf_s(ftrajname, "D:\\fv-traj-%d%02d%02dT%02d%02d%02d.txt", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
	remove(ftrajname);
	
	fopen_s(&fp, fname, "wb");

	if(fp == NULL) // Cannot open File
	{
		printf("\nError opening fly-view FMF writer. Recording terminated.");
		return -1;	
	}

	fopen_s(&flog, flogname, "w");
		
	if(flog == NULL)
	{
		printf("\nError creating fly-view log file. Recording terminated.");
		return -1;
	}

	fopen_s(&ftraj, ftrajname, "w");

	if (ftraj == NULL)
	{
		printf("\nError creating fly-view trajectory file. Recording terminated.");
		return -1;
	}

	return 1;

}

int FVFmfWriter::Close()
{
	//seek to location in file where nframes is stored and replace
	fseek (fp, 20, SEEK_SET );	
	_fwrite_nolock(&nframes, sizeof(unsigned __int64), 1, fp);

	fclose(fp);
	fclose(flog);
	fclose(ftraj);

	fp = NULL;
	flog = NULL;
	ftraj = NULL;

	return 1;
}

void FVFmfWriter::InitHeader(unsigned __int32 x, unsigned __int32 y)
{
	//settings for version 1.0 fmf header, take image dimensions as input with number of frames set to zero
	fmfVersion = 1;
	SizeY = y;
	SizeX = x;
	bytesPerChunk = y*x + sizeof(double);
	//bytesPerChunk = y*x;
	nframes = 0;
}


void FVFmfWriter::WriteHeader()
{
	//write FMF header data
	_fwrite_nolock(&fmfVersion, sizeof(unsigned __int32), 1, fp);
	_fwrite_nolock(&SizeY, sizeof(unsigned __int32), 1, fp);
	_fwrite_nolock(&SizeX, sizeof(unsigned __int32), 1, fp);
	_fwrite_nolock(&bytesPerChunk, sizeof(unsigned __int64), 1, fp);
	_fwrite_nolock(&nframes, sizeof(unsigned __int64), 1, fp);
}

//void FmfWriter::WriteFrame(TimeStamp st, Image img)
void FVFmfWriter::WriteFrame(Image img)
{
	//double dst = (double) st.seconds;
	double dst = (double)nframes;

	_fwrite_nolock(&dst, sizeof(double), 1, fp);
	_fwrite_nolock(img.GetData(), img.GetDataSize(), 1, fp);
}

void FVFmfWriter::WriteFrame(Mat img)
{
	double dst = (double)nframes;

	_fwrite_nolock(&dst, sizeof(double), 1, fp);
	_fwrite_nolock(img.data, sizeof(unsigned char), SizeY*SizeX, fp);
}

//void FmfWriter::WriteLog(TimeStamp st)
//{
//	SYSTEMTIME wt;
//	GetLocalTime(&wt);
//
//	fprintf(flog, "Frame %d - System Time [%02d:%02d:%02d:%d] - TimeStamp [%d %d %d]\n", nframes, wt.wHour, wt.wMinute, wt.wSecond, wt.wMilliseconds, st.cycleSeconds, st.cycleCount, st.cycleOffset);
//}

void FVFmfWriter::WriteLog(int st)
{
	fprintf(flog, "Frame %d - TimeStamp %d\n", nframes, st);
}

void FVFmfWriter::WriteTraj(Point2f pt1, Point2f pt2, float body_angle, Point2f galvo_angle)
{
	fprintf(ftraj, "%d %f %f %f %f %f %f %f\n", nframes, pt1.x, pt1.y, pt2.x, pt2.y, body_angle, galvo_angle.x, galvo_angle.y);
}

int FVFmfWriter::IsOpen()
{
	if (fp == NULL) // Cannot open File
		return 0;
	else
		return 1;
}

