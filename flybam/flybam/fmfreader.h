#ifndef FMFREADER_H
#define FMFREADER_H

using namespace cv;

class FmfReader
{
	private:
		FILE *fp;

		unsigned __int32 fmfVersion, SizeY, SizeX;
		unsigned __int64 bytesPerChunk, nframes;
		long maxFramesInFile;
		char *buf;

	public:

		FmfReader();
		~FmfReader();

		int Open(_TCHAR *fname);
		int Close();

		int ReadHeader();
		Mat ReadFrame(unsigned long frameIndex);
		
		int GetFrameCount();
		void GetImageSize(int &imageWidth, int &imageHeight);
};

#endif
