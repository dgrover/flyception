#ifndef FILEREADER_H
#define FILEREADER_H

using namespace cv;

class FileReader
{
	private:
		FILE *fp;

		unsigned __int32 fmfVersion, SizeY, SizeX;
		unsigned __int64 bytesPerChunk, nframes;
		long maxFramesInFile;
		char *buf;

		string fext;

	public:

		FileReader();
		~FileReader();

		void GetFileExtension(const string &fname);

		int Open(_TCHAR *fname);
		int Close();

		int ReadHeader();
		
		Mat ReadFrame();
		Mat ReadFrame(unsigned long frameIndex);
		
		int GetFrameCount();
		void GetImageSize(int &imageWidth, int &imageHeight);
};

#endif
