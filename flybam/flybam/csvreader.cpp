#include "stdafx.h"
#include "csvreader.h"

CsvReader::CsvReader()
{}

CsvReader::~CsvReader()
{}

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

int CsvReader::ReadLine()
{

  if (fscanf(fp, "%s %f %f\n", n, &x, &y) == 3)
	  return 1;
  else
	  return -1;

}

int CsvReader::GetFrameCount()
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

void CsvReader::ConvertPixelToVoltage(int imageWidth, int imageHeight, int maxVoltage, float64 dataX[], float64 dataY[])
{
    dataX[0] = (x/imageWidth * maxVoltage) - maxVoltage/2;
    dataY[0] = (y/imageHeight * maxVoltage) - maxVoltage/2;

}
