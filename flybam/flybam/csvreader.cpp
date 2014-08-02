#include "stdafx.h"
#include "csvreader.h"

CsvReader::CsvReader()
{}

CsvReader::~CsvReader()
{
  Close();
}

int CsvReader::Open(_TCHAR* fname)
{
  fp = fopen(fname, "rb");

  if(fp == NULL) // Cannot open File
  {
    printf("Cannot open input txt file\n");
    return -1;
  }

  return 1;

}

int CsvReader::Close()
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

int ReadLine()
{

  if (fscanf(fp, "%g,%g\n", &x, &y) == 2)
    return 1;
  else
    return -1;

}

int GetFrameCount()
{
  int n = 0;
  int ch;

  while( !feof(fp) )
  {
    ch = fgetc(fp);
    if(ch == '\n')
        n++;
  }

  //seek to beginning of file after counting number of lines
  rewind(fp);

  return n;
}

void CsvReader::ConvertPixelToVoltage(float64 *data)
{
    int imageWidth = 512;
    int imageHeight = 512;

    int maxVoltage = 3.0;

    data[0] = (x/imageWidth * maxVoltage) - maxVoltage/2;
    data[1] = (y/imageHeight * maxVoltage) - maxVoltage/2;

}
