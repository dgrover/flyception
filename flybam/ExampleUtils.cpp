// -----------------------------------------------------------------------------------------
// Common Functions for Sapera Examples
// 
//

// Disable the following Visual C++ compiler warning
//    4786 -> 'identifier' : identifier was truncated to 'number' characters in the debug information
#ifdef _MSC_VER
#pragma warning(disable: 4786)
#endif

// Disable deprecated function warnings with Visual Studio 2005
#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(disable: 4995)
#endif

#include "ExampleUtils.h"
#include "stdafx.h"

// Restore deprecated function warnings with Visual Studio 2005
#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(default: 4995)
#endif

#ifndef SAP_CAMERA_SDK
void XferCallback(SapXferCallbackInfo *pInfo)
{
	SapView *pView= (SapView *) pInfo->GetContext();
	pView->Show();
}
#endif   // SAP_CAMERA_SDK

BOOL GetOptionsFromQuestions(char *acqServerName, UINT32 *pAcqDeviceIndex, char *configFileName)
{
   //////// Ask questions to user to select acquisition board/device and config file ////////

   // Get total number of boards in the system
   int serverCount = SapManager::GetServerCount();
	char deviceIndexToPrint[STRING_LENGTH];
	char configFileIndexToPrint[STRING_LENGTH];
   if (serverCount == 0)
   {
      printf("No device found!\n");
      return FALSE;
   }
   
   int devicesToSkip = 0;
   for (int serverIndex = 0; serverIndex < serverCount; serverIndex++)
   {
      if (SapManager::GetResourceCount(serverIndex, SapManager::ResourceAcqDevice) != 0)
      {
         char serverName[CORSERVER_MAX_STRLEN];
         SapManager::GetServerName(serverIndex, serverName, sizeof(serverName));
         if (strstr(serverName,"CameraLink_") > 0)
            devicesToSkip++;
      }
   }

   printf("\nSelect the acquisition server: Press ");
	printf("1");
	CorSnprintf(deviceIndexToPrint, sizeof(deviceIndexToPrint), "%d", serverCount-1-devicesToSkip);
	if (serverCount >= 2)
	{
		printf(" to ");
		printf(deviceIndexToPrint);
	}
	printf(" or 'q' to quit.");
	printf("\n........................................\n");

   // Scan the boards to find those that support acquisition
   BOOL serverFound = FALSE;
   BOOL cameraFound = FALSE;

   for (int serverIndex = 0; serverIndex < serverCount; serverIndex++)
   {
      if (SapManager::GetResourceCount(serverIndex, SapManager::ResourceAcq) != 0)
      {
         char serverName[CORSERVER_MAX_STRLEN];
         SapManager::GetServerName(serverIndex, serverName, sizeof(serverName));
         printf("%d: %s\n", serverIndex, serverName);
         serverFound = TRUE;
      }
	  if (SapManager::GetResourceCount(serverIndex, SapManager::ResourceAcqDevice) != 0)
      {
         char serverName[CORSERVER_MAX_STRLEN];
         SapManager::GetServerName(serverIndex, serverName, sizeof(serverName));
         if (strstr(serverName,"CameraLink_") > 0)
            continue;
         printf("%d: %s\n", serverIndex, serverName);
         cameraFound = TRUE;
      }
   }

   // At least one acquisition server must be available
   if (!serverFound && !cameraFound)
   {
      printf("No acquisition server found!\n");
      return FALSE;
   }

   char key = (char)_getch();
   if (key != 0)
   {
	   if (key == 'q')
			return FALSE;

      int serverNum = key - '0'; // char-to-int conversion
      if ((serverNum >= 1) && (serverNum < serverCount))
      {
         // update board name
         SapManager::GetServerName(serverNum, acqServerName, CORSERVER_MAX_STRLEN);
      }
      else
      {
         printf("Invalid selection!\n");
         return FALSE;
      }
   }
   else
   {
      printf("Invalid selection!\n");
      return FALSE;
   }


// Scan all the acquisition devices on that server and show menu to user
int deviceCount = SapManager::GetResourceCount(acqServerName, SapManager::ResourceAcq);
int cameraCount = SapManager::GetResourceCount(acqServerName, SapManager::ResourceAcqDevice);
int allDeviceCount=0;

   printf("\nSelect the acquisition device: Press ");
	printf("1");
	allDeviceCount = deviceCount+cameraCount;
	CorSnprintf(deviceIndexToPrint, sizeof(deviceIndexToPrint), "%d", allDeviceCount);
	if (allDeviceCount >= 2)
	{
		printf(" to ");
		printf(deviceIndexToPrint);
	}
	printf(" or 'q' to quit.");
	printf("\n........................................\n");


   for (int deviceIndex = 0; deviceIndex < deviceCount; deviceIndex++)
   {
      char deviceName[CORPRM_GETSIZE(CORACQ_PRM_LABEL)];
      SapManager::GetResourceName(acqServerName, SapManager::ResourceAcq, deviceIndex, deviceName, sizeof(deviceName));
      printf("%d: %s\n", deviceIndex+1, deviceName);
   }

   for (int cameraIndex = 0; cameraIndex < cameraCount; cameraIndex++)
   {
      char cameraName[CORPRM_GETSIZE(CORACQ_PRM_LABEL)];
	  SapManager::GetResourceName(acqServerName, SapManager::ResourceAcqDevice, cameraIndex, cameraName, sizeof(cameraName));
	  printf("%d: %s\n", cameraIndex+1, cameraName);
   }


   key = (char)_getch();
   if (key != 0)
   {
	   if (key == 'q')
			return FALSE;
      int deviceNum = key - '0'; // char-to-int conversion
      if ((deviceNum >= 1) && (deviceNum <= deviceCount+cameraCount))
      {
         *pAcqDeviceIndex = deviceNum-1;
      }
      else
      {
         printf("Invalid selection!\n");
         return FALSE;
      }
   }
   else
   {
      printf("Invalid selection!\n");
      return FALSE;
   }

   // List all files in the config directory
   char configPath[MAX_PATH];
	GetEnvironmentVariable("SAPERADIR", configPath, sizeof(configPath));
	CorStrncat(configPath, "\\CamFiles\\User\\", sizeof(configPath));

   char findPath[MAX_PATH];
   CorStrncpy(findPath, configPath, MAX_PATH);
   CorStrncat(findPath, "*.ccf", sizeof(findPath));

	HANDLE fhandle;
	WIN32_FIND_DATA fdata;
	if ((fhandle = FindFirstFile(findPath, &fdata)) == INVALID_HANDLE_VALUE)
	{
		if (cameraCount==0)
		{
		printf("No config file found.\nUse CamExpert to generate a config file before running this example.\n");
		return FALSE;
		}
	}


   int configFileCount = 0;
	do
	{
	  configFileCount++;
	}
	while (FindNextFile(fhandle, &fdata) && configFileCount < MAX_CONFIG_FILES);
	FindClose(fhandle);
	fhandle = FindFirstFile(findPath, &fdata);

    // Try to find the last letter to choose

    int configFileMenuCount = 0;
    char lastCharMenu='x';
    do
	{
		// Use numbers 0 to 9, then lowercase letters if there are more than 10 files
		int configFileMenuShow = configFileMenuCount+1;
		if (configFileMenuCount > 9)
            lastCharMenu = (char)(configFileMenuShow - 10 + 'a');
		//CorStrncpy(configFileNames[configFileCount], fdata.cFileName, sizeof(configFileNames[configFileCount]));
		configFileMenuCount++;
	}
	while (FindNextFile(fhandle, &fdata) && configFileMenuCount < MAX_CONFIG_FILES);
	FindClose(fhandle);


    printf("\nSelect the config file: Press ");
	printf("1");
	CorSnprintf(configFileIndexToPrint, sizeof(configFileIndexToPrint), "%d", configFileCount);
	if (cameraCount == 0)
	{
		if (configFileCount >= 2)
		{
			printf(" to ");
		    if (configFileCount <= 9)
    			printf(configFileIndexToPrint);
            else
                printf("9");
		}

        if (lastCharMenu != 'x')
        {
			    printf(" or 'a'");
			    printf(" to '");
                printf("%c", lastCharMenu);
			    printf("'");
        }
		printf(" or 'q' to quit.");
		printf("\n........................................\n");
		configFileCount = 0;
    }
	else
	{
      if(configFileCount > 1)
      {
         printf(" to ");
         printf("%d",configFileCount);
      }
         
      printf(" or 'q' to quit.");
      printf("\n........................................\n");
      printf("1: No config File.\n");
	}

	
    fhandle = FindFirstFile(findPath, &fdata);							//find first file
   BOOL moreFilesAvailable = TRUE;
 	
	while ((fhandle != INVALID_HANDLE_VALUE && moreFilesAvailable) && configFileCount < MAX_CONFIG_FILES)
	{
		// Use numbers 0 to 9, then lowercase letters if there are more than 10 files
		int configFileShow = configFileCount+1;
		if (configFileCount < 9)
			printf("%d: %s\n", configFileShow, fdata.cFileName);
		else
			printf("%c: %s\n", configFileShow - 10 + 'a', fdata.cFileName);
		CorStrncpy(configFileNames[configFileCount], fdata.cFileName, sizeof(configFileNames[configFileCount]));
		configFileCount++;
		moreFilesAvailable = FindNextFile(fhandle, &fdata);
	}
	 
	 FindClose(fhandle);

   key = (char)_getch();
   if (key != 0)
   {
	   if (key == '1' && cameraCount != 0)
	   {
			CorStrncpy(configFileName, "NoFile",7); 
			return TRUE;
	   }
		   
		if (key == 'q')
			return FALSE;
      // Use numbers 0 to 9, then lowercase letters if there are more than 10 files
      int configNum;
      if (key >= '1' && key <= '9')
         configNum = key - '1'; // char-to-int conversion
      else
         configNum = key - 'a' + 9; // char-to-int conversion

      if ((configNum >= 0) && (configNum <= configFileCount-1))
      {
		  CorStrncpy(configFileName, configPath, sizeof(configPath));
		  CorStrncat(configFileName, configFileNames[configNum], sizeof(configFileNames[configNum]));
      }
      else
      {
         printf("Invalid selection!\n"); 
         return FALSE;
      }
   }
   else
   {
      printf("Invalid selection!\n");
      return FALSE;
   }

   printf("\n");
   return TRUE;
}


BOOL GetCorAcqDeviceOptionsFromQuestions(char *acqServerName, UINT32 *pAcqDeviceIndex, BOOL showGigEOnly)
{
   int serverCount = SapManager::GetServerCount();
	int acqDeviceCount = 0;
   int deviceCount = 0;
   int GenieIndex = 0;
   int deviceIndex = 0;
   std::vector<std::string> v_ServerNames;
	char deviceIndexToPrint[STRING_LENGTH];
   if (serverCount == 0)
   {
      printf("No device found!\n");
      return FALSE;
   }


#ifdef GRAB_CAMERA_LINK
   printf("\nNote:\nOnly CameraLink cameras will work with this example !\nBehavior is undefined for any other devices. \n");
#endif
      printf("\nSelect one of the camera(s) detected: Press ");


	printf("1");
   char serverName[CORSERVER_MAX_STRLEN]; 
   for (int serverAcqIndex = 0; serverAcqIndex < serverCount; serverAcqIndex++)
   {
		if (SapManager::GetResourceCount(serverAcqIndex, SapManager::ResourceAcqDevice) != 0)
      {
         SapManager::GetServerName(serverAcqIndex, serverName, sizeof(serverName));
         acqDeviceCount++;
         if (showGigEOnly && strstr(serverName,"CameraLink_") > 0)
            acqDeviceCount--;
		}
	}

	CorSnprintf(deviceIndexToPrint, sizeof(deviceIndexToPrint), "%d", acqDeviceCount);
	if (acqDeviceCount >= 2)
	{
		printf(" to ");
		printf(deviceIndexToPrint);
	}
	printf(" or 'q' to quit.");
	printf("\n........................................\n");
   BOOL serverFound = FALSE;
     
   for (int serverIndex = 0; serverIndex < serverCount; serverIndex++)
   {
		if (SapManager::GetResourceCount(serverIndex, SapManager::ResourceAcqDevice) != 0)
      {
			SapManager::GetServerName(serverIndex, serverName, sizeof(serverName));
         if (showGigEOnly && strstr(serverName,"CameraLink_") > 0)
            continue;
			printf("%d: %s\n", GenieIndex+1, serverName);
			GenieIndex++;
			serverFound = TRUE;
			deviceCount = GenieIndex;

			char deviceName[CORPRM_GETSIZE(CORACQ_PRM_LABEL)];
			deviceIndex=0;
			SapManager::GetResourceName(serverName, SapManager::ResourceAcqDevice, deviceIndex, deviceName, sizeof(deviceName));
			printf("    %s%s\n", "User defined Name : ", deviceName);
			printf("........................................\n");			
			v_ServerNames.push_back(serverName);
      }
   }

   // At least one acquisition server must be available
   if (!serverFound)
   {
      printf("No camera found!\n");
      return FALSE;
   }

   char key = (char)_getch();
   if (key != 0)
   {
		if (key == 'q')
			return FALSE;
      int serverNum = key - '0'; // char-to-int conversion
      if ((serverNum >= 1) && (serverNum < GenieIndex+1))
      {
		  std::string sServerName = v_ServerNames[serverNum-1];
		  // Get the Acquisition Server name selected.
		  CorStrncpy(acqServerName, sServerName.c_str(),sServerName.size()+1);
		  *pAcqDeviceIndex = 0;
      }
      else
      {
         printf("Invalid selection!\n");
         return FALSE;
      }
   }
   else
   {
      printf("Invalid selection!\n");
      return FALSE;
   }

   printf("\n");
   return TRUE;
}

#ifndef SAP_CAMERA_SDK
BOOL IsMonoBuffer(SapBuffer* Buffers)
{
	SapFormat format = Buffers->GetFormat();

	if(CORDATA_FORMAT_IS_MONO(format))
		return TRUE;
	else
		return FALSE;
}

SapData SetDataValue(SapBuffer* Buffers, DWORD *pPrmIndex)
{
	if(IsMonoBuffer(Buffers) == TRUE)
	{
		SapDataMono mono(128);
		return mono;
	}
	else
	{
		SapDataRGB rgb;
		//SapDataRGB rgb(m_pInfoList->GetValueAt(*pPrmIndex), m_pInfoList->GetValueAt(*pPrmIndex+1), m_pInfoList->GetValueAt(*pPrmIndex+2));
		*pPrmIndex = *pPrmIndex+3;
		return rgb;
	}
}

BOOL GetLUTOptionsFromQuestions(SapBuffer* Buffers, SapLut* m_pLut, char *chAcqLutName)
{
   //////// Ask questions to user to select LUT mode ////////
	DWORD prmIndex = 1;
	char acqLutFileName[STRING_LENGTH];

   printf("\nSelect the LookUpTable mode you want to apply: \n");

   printf("%s: %s\n", "a", "Normal mode");
   printf("%s: %s\n", "b", "Arithmetic mode");
   printf("%s: %s\n", "c", "Binary mode");
   printf("%s: %s\n", "d", "Boolean mode");
   printf("%s: %s\n", "e", "Gamma mode");
   printf("%s: %s\n", "f", "Reverse mode");
   printf("%s: %s\n", "g", "Roll mode");
   printf("%s: %s\n", "h", "Shift mode");
   printf("%s: %s\n", "i", "Slope mode");
   printf("%s: %s\n", "j", "Threshold single mode");
   printf("%s: %s\n", "k", "Threshold double mode");

   char key = (char)_getch();
   if (key != 0)
   {
      if (key == 'a' || key == 'b' || key == 'c' || key == 'd' || key == 'e' || key == 'f' || key == 'g' || key == 'h' || key == 'i' || key == 'j' || key == 'k')
		{

			switch (::tolower(key))
			{
					case 'a':
					{
						m_pLut->Normal();
						CorStrncpy(acqLutFileName, "Normal_Lut_Mode.lut", sizeof("Normal_Lut_Mode.lut"));
						CorStrncpy(chAcqLutName, "Normal Lut", sizeof("Normal Lut"));
						break;
					}
					case 'b':
					{
						int operationMode = 0;//Linear plus offset with clip
						/*
							Others operations available
						*/
						//int operation = 1;//Linear minus offset(absolute)
						//int operation = 2;//Linear minus offset(with clip)
						//int operation = 3;//Linear with lower clip
						//int operation = 4;//Linear with upper clip
						//int operation = 5;//Scale to maximum limit
						
						SapData offSet;
						offSet = SetDataValue(Buffers, &prmIndex);
						m_pLut->Arithmetic((SapLut::ArithmeticOp)operationMode, offSet);
						CorStrncpy(acqLutFileName, "Arithmetic_Lut_Mode.lut", sizeof("Arithmetic_Lut_Mode.lut"));
						CorStrncpy(chAcqLutName, "Arithmetic Lut", sizeof("Arithmetic Lut"));
						break;
					}
					case 'c':
					{
						SapData clipValue;
						clipValue = SetDataValue(Buffers, &prmIndex);
						m_pLut->BinaryPattern(0, clipValue);
						CorStrncpy(acqLutFileName, "Binary_Lut_Mode.lut", sizeof("Binary_Lut_Mode.lut"));
						CorStrncpy(chAcqLutName, "Binary Lut", sizeof("Binary Lut"));
						break;
					}
					case 'd':
					{
						SapData booleanFunction;
						booleanFunction = SetDataValue(Buffers, &prmIndex);
						m_pLut->Boolean((SapLut::BooleanOp)0, booleanFunction);
						/*
							Others operations available
						*/
						// AND
						//m_pLut->Boolean((SapLut::BooleanOp)1, booleanFunction);
						// OR
						//m_pLut->Boolean((SapLut::BooleanOp)2, booleanFunction);
						// XOR
						CorStrncpy(acqLutFileName, "Boolean_Lut_Mode.lut", sizeof("Boolean_Lut_Mode.lut"));
						CorStrncpy(chAcqLutName, "Boolean Lut", sizeof("Boolean Lut"));
						break;
					}
					case 'e':
					{
						int gammaFactor = (int)(2*GAMMA_FACTOR);
						m_pLut->Gamma((float)gammaFactor/GAMMA_FACTOR);
						CorStrncpy(acqLutFileName, "Gamma_Lut_Mode.lut", sizeof("Gamma_Lut_Mode.lut"));
						CorStrncpy(chAcqLutName, "Gamma Lut", sizeof("Gamma Lut"));
						break;
					}
					case 'f':
					{
						m_pLut->Reverse();
						CorStrncpy(acqLutFileName, "Reverse_Lut_Mode.lut", sizeof("Reverse_Lut_Mode.lut"));
						CorStrncpy(chAcqLutName, "Reverse Lut", sizeof("Reverse Lut"));
						break;
					}
					case 'g':
					{
						int numEntries = 128;
						m_pLut->Roll(numEntries);
						CorStrncpy(acqLutFileName, "Roll_Lut_Mode.lut", sizeof("Roll_Lut_Mode.lut"));
						CorStrncpy(chAcqLutName, "Roll Lut", sizeof("Roll Lut"));
						break;
					}
					case 'h':
					{
						int bitsToShift = 3;
						m_pLut->Shift(bitsToShift);
						CorStrncpy(acqLutFileName, "Shift_Lut_Mode.lut", sizeof("Shift_Lut_Mode.lut"));
						CorStrncpy(chAcqLutName, "Shift Lut", sizeof("Shift Lut"));
						break;
					}
					case 'i':
					{
						int startIndex1 = 76;
						int endIndex1 = 179;
						BOOL clipOutSide = FALSE;//TRUE
						SapData minValue;
						SapData maxValue;
						minValue = SetDataValue(Buffers, &prmIndex);
						maxValue = SetDataValue(Buffers, &prmIndex);
						m_pLut->Slope(startIndex1, endIndex1, minValue, maxValue, clipOutSide);
						CorStrncpy(acqLutFileName, "Slope_With_Range_Lut_Mode.lut", sizeof("Slope_With_Range_Lut_Mode.lut"));
						CorStrncpy(chAcqLutName, "Slope With Range Lut", sizeof("Slope With Range Lut"));
						break;
					}
					case 'j':
					{
						SapData treshValue;
						treshValue = SetDataValue(Buffers, &prmIndex);
						m_pLut->Threshold(treshValue);
						CorStrncpy(acqLutFileName, "Threshold_Single_Mode.lut", sizeof("Threshold_Single_Mode.lut"));
						CorStrncpy(chAcqLutName, "Threshold Single Lut", sizeof("Threshold Single Lut"));
						break;
					}
					case 'k':
					{
						SapData treshValue1;
						SapData treshValue2;
						treshValue1 = SetDataValue(Buffers, &prmIndex);
						treshValue2 = SetDataValue(Buffers, &prmIndex);
						m_pLut->Threshold(treshValue1, treshValue2);
						CorStrncpy(acqLutFileName, "Threshold_Double_Mode.lut", sizeof("Threshold_Double_Mode.lut"));
						CorStrncpy(chAcqLutName, "Threshold Double Lut", sizeof("Threshold Double Lut"));
						break;
					}
			}
		}         
      else
      {
         printf("\nInvalid selection!\n");
         return FALSE;
      }
   }
   else
   {
      printf("\nInvalid selection!\n");
      return FALSE;
   }

	m_pLut->Save(acqLutFileName);		// Save LUT to file (can be reloaded in the main demo)
   printf("\n");
   return TRUE;
}
#endif   // SAP_CAMERA_SDK

int GetKeyCharIndex(char key) 
{
   if (key != 0)
   {
			switch (::tolower(key))
			{
					case 'a':
						return 0;
					case 'b':
						return 1;
					case 'c':
						return 2;
					case 'd':
						return 3;
					case 'e':
						return 4;
					case 'f':
						return 5;
					case 'g':
						return 6;
					case 'h':
						return 7;
					case 'i':
						return 8;
					case 'j':
						return 9;
					case 'k':
						return 10;
					case 'l':
						return 11;
					case 'm':
						return 12;
					case 'n':
						return 13;
					case 'o':
						return 14;
					case 'p':
						return 15;
					case 'q':
						return 16;
					case 'r':
						return 17;
					case 's':
						return 18;
					case 't':
						return 19;
					case 'u':
						return 20;
					case 'v':
						return 21;
					case 'w':
						return 22;
					case 'x':
						return 23;
					case 'y':
						return 24;
					case 'z':
						return 25;
			}
		}         
   else
   {
      printf("\nInvalid selection!\n");
      return -1;
   }
return -1;
}


BOOL GetLoadLUTFiles(SapLut* Lut, std::vector<std::string> v_list, BOOL* bLutLoaded)
{
   //////// Ask questions to user to select LUT file ////////
	std::string fileName;
	std::string fullFileName;

	v_list = GetLUTFilesSaved(".", v_list, FALSE);
	int numVect = (int)v_list.size();

	if (numVect!=0)
	{
			printf("\nDo you want to load an existing LUT file? y/n (Yes/No)  \n");
			char keyQuestion = (char)_getch();
			if (keyQuestion != 0)
			{
				if (keyQuestion == 'n')
				{
					*bLutLoaded = FALSE;
					return TRUE;
				}
				else if (keyQuestion == 'y')
					goto select_lut;
				else
				{
					printf("\nInvalid selection!\n");
					return FALSE;
				}
			}
			else
			{
				printf("\nInvalid selection!\n");
				return FALSE;
			}

	select_lut:
		printf("\nSelect the LUT file available: \n");

		v_list = GetLUTFilesSaved(".", v_list, TRUE);

			char key = (char)_getch();
			if (key != 0)
			{
				int fileNum = GetKeyCharIndex(key);
				if ((fileNum != -1) && (fileNum >= 0) && (fileNum < (int)v_list.size() ))
				{
					fileName = v_list[fileNum];
					fullFileName = fileName + ".lut";
					// Load LUT (saved before in the main demo)
					if (Lut->Load(fullFileName.c_str()))
						*bLutLoaded=TRUE;			

				printf("\n");
				if (bLutLoaded)
					printf("%s%s\n", fullFileName.c_str(), " loaded.");

				}         
				else
				{
					printf("Invalid selection!\n");
					return FALSE;
				}
			}
			else
			{
				printf("Invalid selection!\n");
				return FALSE;
			}
	}

return TRUE;
}


#ifndef SAP_CAMERA_SDK
BOOL GetLoadDynamicLUTFiles(SapDynamicLut* DynamicLut, int lutIndex, std::vector<std::string> v_list, BOOL* bLutLoaded)
{
   //////// Ask questions to user to select LUT file ////////
	std::string fileName;
	std::string fullFileName;

	v_list = GetLUTFilesSaved(".", v_list, FALSE);
	int numVect = (int)v_list.size();

	if (numVect!=0)
	{
			printf("\nSelect one of the Dynamic LUT files available: \n");

			// Remove vector content.
			v_list.erase(v_list.begin(), v_list.end());

			v_list = GetLUTFilesSaved(".", v_list, TRUE);

			int listSize = (int)v_list.size();

			char key = (char)_getch();
			if (key != 0)
			{
				int fileNum = GetKeyCharIndex(key);
				if ((fileNum != -1) && (fileNum >= 0) && (fileNum < listSize ))
				{
					fileName = v_list[fileNum];
					fullFileName = fileName + ".lut";
					// Load LUT (saved before in the main demo)
					if (DynamicLut->Load(lutIndex, fullFileName.c_str()))
						*bLutLoaded=TRUE;			

				printf("\n");
				if (bLutLoaded)
					printf("%s%s\n", fullFileName.c_str(), " loaded.");

				}         
				else
				{
					printf("Invalid selection!\n");
					return FALSE;
				}
			}
			else
			{
				printf("Invalid selection!\n");
				return FALSE;
			}
	}
	else
	{
		printf("\nNo Dynamic LUT file available.\n\n");
		return FALSE;
	}

return TRUE;
}
#endif   // SAP_CAMERA_SDK

std::vector<std::string> GetLUTFilesSaved(char Dossier[STRING_LENGTH], std::vector<std::string> pList, BOOL bPrint) 
{ 
HANDLE hFind; 
WIN32_FIND_DATA FindData; 

// Set the directory
SetCurrentDirectory (Dossier); 

// Beginning of search
hFind=FindFirstFile ("*.*", &FindData); 
//plist.empty();
int fileIndex=0;

if (hFind!=INVALID_HANDLE_VALUE) 
{ 
	// Next files
	while (FindNextFile (hFind, &FindData)) 
	{ 
		if (!(FindData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) 
		{ 
		char* file_name = FindData.cFileName;
		char * pch;
      char *nextToken;

		pch = CorStrtok(file_name, ".", &nextToken);

		char* extension_file = pch;
		while (pch != NULL)
		{
			extension_file = pch;

         pch = CorStrtok(NULL, ".", &nextToken);
			if (pch == NULL) 
            break;
		}

         if (strlen(extension_file))
         {
				if(strcmp(extension_file,"lut") == 0)
				{
		         if (bPrint)
					{
						switch(fileIndex)
						{
							case 0:
							{
								printf("%s: %s\n", "a", file_name);
								break;
							}
							case 1:
							{
								printf("%s: %s\n", "b", file_name);
								break;
							}
							case 2:
							{
								printf("%s: %s\n", "c", file_name);
								break;
							}
							case 3:
							{
								printf("%s: %s\n", "d", file_name);
								break;
							}
							case 4:
							{
								printf("%s: %s\n", "e", file_name);
								break;
							}
							case 5:
							{
								printf("%s: %s\n", "f", file_name);
								break;
							}
							case 6:
							{
								printf("%s: %s\n", "g", file_name);
								break;
							}
							case 7:
							{
								printf("%s: %s\n", "h", file_name);
								break;
							}
							case 8:
							{
								printf("%s: %s\n", "i", file_name);
								break;
							}
							case 9:
							{
								printf("%s: %s\n", "j", file_name);
								break;
							}
							case 10:
							{
								printf("%s: %s\n", "k", file_name);
								break;
							}
							case 11:
							{
								printf("%s: %s\n", "l", file_name);
								break;
							}
							case 12:
							{
								printf("%s: %s\n", "m", file_name);
								break;
							}
						//printf("%d: %s\n", fileIndex, file_name);
						}
					}
					fileIndex++;
					pList.push_back(file_name);
				}
			} 

		} 
	}//end while 
}
// End of search
FindClose (hFind); 
return pList;
} 




BOOL GetCorAcquisitionOptionsFromQuestions(char *acqServerName, UINT32 *pAcqDeviceIndex, char *configFileName)
{
   //////// Ask questions to user to select acquisition board/device and config file ////////

   // Get total number of boards in the system
   int serverCount = SapManager::GetServerCount();
	char deviceIndexToPrint[STRING_LENGTH];
	char configFileIndexToPrint[STRING_LENGTH];
   if (serverCount == 0)
   {
      printf("No device found!\n");
      return FALSE;
   }

   printf("\nSelect the acquisition server: Press ");
	printf("1");
	CorSnprintf(deviceIndexToPrint, sizeof(deviceIndexToPrint), "%d", serverCount-1);
	if (serverCount >= 2)
	{
		printf(" to ");
		printf(deviceIndexToPrint);
	}
	printf(" or 'q' to quit.");
	printf("\n........................................\n");

   // Scan the boards to find those that support acquisition
   BOOL serverFound = FALSE;

   for (int serverIndex = 0; serverIndex < serverCount; serverIndex++)
   {
      if (SapManager::GetResourceCount(serverIndex, SapManager::ResourceAcq) != 0)
      {
         char serverName[CORSERVER_MAX_STRLEN];
         SapManager::GetServerName(serverIndex, serverName, sizeof(serverName));
         printf("%d: %s\n", serverIndex, serverName);
         serverFound = TRUE;
      }
   }

   // At least one acquisition server must be available
   if (!serverFound)
   {
      printf("No acquisition server found!\n");
      return FALSE;
   }

   char key = (char)_getch();
   if (key != 0)
   {
	   if (key == 'q')
			return FALSE;

      int serverNum = key - '0'; // char-to-int conversion
      if ((serverNum >= 1) && (serverNum < serverCount))
      {
         // update board name
         SapManager::GetServerName(serverNum, acqServerName, CORSERVER_MAX_STRLEN);
      }
      else
      {
         printf("Invalid selection!\n");
         return FALSE;
      }
   }
   else
   {
      printf("Invalid selection!\n");
      return FALSE;
   }


// Scan all the acquisition devices on that server and show menu to user
int deviceCount = SapManager::GetResourceCount(acqServerName, SapManager::ResourceAcq);
int allDeviceCount=0;
int cameraCount = 0;

#ifndef GRAB_CAMERA_LINK
   printf("\nSelect the acquisition device: Press ");
#else
{
   printf("\nSelect the device you wish to use on this server: Press ");
}
#endif
	printf("1");
	allDeviceCount = deviceCount;
	CorSnprintf(deviceIndexToPrint, sizeof(deviceIndexToPrint), "%d", allDeviceCount);
	if (allDeviceCount >= 2)
	{
		printf(" to ");
		printf(deviceIndexToPrint);
	}
	printf(" or 'q' to quit.");
	printf("\n........................................\n");


   for (int deviceIndex = 0; deviceIndex < deviceCount; deviceIndex++)
   {
      char deviceName[CORPRM_GETSIZE(CORACQ_PRM_LABEL)];
      SapManager::GetResourceName(acqServerName, SapManager::ResourceAcq, deviceIndex, deviceName, sizeof(deviceName));
      printf("%d: %s\n", deviceIndex+1, deviceName);
   }


   key = (char)_getch();
   if (key != 0)
   {
	   if (key == 'q')
			return FALSE;
      int deviceNum = key - '0'; // char-to-int conversion
      if ((deviceNum >= 1) && (deviceNum <= deviceCount))
      {
         *pAcqDeviceIndex = deviceNum-1;
      }
      else
      {
         printf("Invalid selection!\n");
         return FALSE;
      }
   }
   else
   {
      printf("Invalid selection!\n");
      return FALSE;
   }

   // List all files in the config directory
   char configPath[MAX_PATH];
	GetEnvironmentVariable("SAPERADIR", configPath, sizeof(configPath));
	CorStrncat(configPath, "\\CamFiles\\User\\", sizeof(configPath));

   char findPath[MAX_PATH];
   CorStrncpy(findPath, configPath, MAX_PATH);
   CorStrncat(findPath, "*.ccf", sizeof(findPath));


	HANDLE fhandle;
	WIN32_FIND_DATA fdata;
	if ((fhandle = FindFirstFile(findPath, &fdata)) == INVALID_HANDLE_VALUE)
	{
		if (cameraCount==0)
		{
		printf("No config file found.\nUse CamExpert to generate a config file before running this example.\n");
		return FALSE;
		}
	}


   int configFileCount = 0;
	do
	{
	  configFileCount++;
	}
	while (FindNextFile(fhandle, &fdata) && configFileCount < MAX_CONFIG_FILES);
	FindClose(fhandle);
	fhandle = FindFirstFile(findPath, &fdata);

    // Try to find the last letter to choose

    int configFileMenuCount = 0;
    char lastCharMenu='x';
    do
	{
		// Use numbers 0 to 9, then lowercase letters if there are more than 10 files
		int configFileMenuShow = configFileMenuCount+1;
		if (configFileMenuCount > 9)
            lastCharMenu = (char)(configFileMenuShow - 10 + 'a');
		//CorStrncpy(configFileNames[configFileCount], fdata.cFileName, sizeof(configFileNames[configFileCount]));
		configFileMenuCount++;
	}
	while (FindNextFile(fhandle, &fdata) && configFileMenuCount < MAX_CONFIG_FILES);
	FindClose(fhandle);


    printf("\nSelect the config file: Press ");
	printf("1");
	CorSnprintf(configFileIndexToPrint, sizeof(configFileIndexToPrint), "%d", configFileCount);
	if (cameraCount == 0)
	{
		if (configFileCount >= 2)
		{
			printf(" to ");
		    if (configFileCount <= 9)
    			printf(configFileIndexToPrint);
            else
                printf("'9'");
		}

        if (lastCharMenu != 'x')
        {
			    printf(" or 'a'");
			    printf(" to '");
                printf("%c", lastCharMenu);
			    printf("'");
        }
    }

	printf(" or 'q' to quit.");
	printf("\n........................................\n");


   configFileCount = 0;
   fhandle = FindFirstFile(findPath, &fdata);
	 do
	 {
		  // Use numbers 0 to 9, then lowercase letters if there are more than 10 files
		  int configFileShow = configFileCount+1;
		  if (configFileCount < 9)
				printf("%d: %s\n", configFileShow, fdata.cFileName);
		  else
				printf("%c: %s\n", configFileShow - 10 + 'a', fdata.cFileName);
		  CorStrncpy(configFileNames[configFileCount], fdata.cFileName, sizeof(configFileNames[configFileCount]));
		  configFileCount++;
	 }
	 while (FindNextFile(fhandle, &fdata) && configFileCount < MAX_CONFIG_FILES);
	 FindClose(fhandle);

   key = (char)_getch();
   if (key != 0)
   {
	   if (key == '1' && cameraCount != 0)
			return TRUE;
		if (key == 'q')
			return FALSE;
      // Use numbers 0 to 9, then lowercase letters if there are more than 10 files
      int configNum;
      if (key >= '1' && key <= '9')
         configNum = key - '1'; // char-to-int conversion
      else
         configNum = key - 'a' + 10; // char-to-int conversion

      if ((configNum >= 0) && (configNum <= configFileCount))
      {
		  CorStrncpy(configFileName, configPath, sizeof(configPath));
		  CorStrncat(configFileName, configFileNames[configNum], sizeof(configFileNames[configNum]));
      }
      else
      {
         printf("Invalid selection!\n"); 
         return FALSE;
      }
   }
   else
   {
      printf("Invalid selection!\n");
      return FALSE;
   }

   printf("\n");
   return TRUE;
}
