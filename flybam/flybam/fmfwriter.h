#ifndef FMFWRITER_H
#define FMFWRITER_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

class FmfWriter
{
	private:
		FILE *fp;
		FILE *flog;
		FILE *ftraj;

		char fname[100];
		char flogname[100];
		char ftrajname[100];

		unsigned __int32 fmfVersion, SizeY, SizeX;
		unsigned __int64 bytesPerChunk;
		char *buf;

		
	public:
		unsigned __int64 nframes;

		FmfWriter();
		
		int Open();
		int Close();

		void InitHeader(unsigned __int32 x, unsigned __int32 y);
		void WriteHeader();
		void WriteFrame(TimeStamp st, Image img);
		void WriteLog(TimeStamp st);
		void WriteTraj(Point2f pt);
		
};

#endif