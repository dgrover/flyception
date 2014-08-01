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

		int Open(_TCHAR *fname);
		int Close();

		int ReadHeader();
		int ReadFrame(unsigned long frameIndex);
		Mat ConvertToCvMat();

};

#endif
