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
    void ConvertPixelToVoltage(float64 dataX[], float64 dataY[]);
};

#endif
