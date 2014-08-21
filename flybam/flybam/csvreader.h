#ifndef CSVREADER_H
#define CSVREADER_H

class CsvReader
{
  private:
    FILE *fp;
    float x,y;
	std::string n;

  public:

	CsvReader();
    ~CsvReader();

    int Open(_TCHAR *fname);
    int Close();
    int ReadLine();
	int GetFrameCount();
	void ConvertPixelToVoltage(int imageWidth, int imageHeight, int maxVoltage, float64 dataX[], float64 dataY[]);
};

#endif
