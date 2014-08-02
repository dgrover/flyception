#ifndef CSVREADER_H
#define CSVREADER_H

class CsvReader
{
  private:
    FILE *fp;
    float x,y;

  public:

    CsvReader();
    ~CsvReader();

    int Open(_TCHAR *fname);
    int Close();
    int ReadLine();
    void ConvertPixelToVoltage(float64 *data);
};

#endif
