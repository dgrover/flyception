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

		float x, y;

	public:

		string fext;

		FmfReader();
		~FmfReader();

		void GetFileExtension(const string& fname);

		int Open(_TCHAR *fname);
		int Close();

		int ReadHeader();
		int GetFrameCount();
		int ReadFrame(unsigned long frameIndex);
		Mat ConvertToMat();
};

#endif
