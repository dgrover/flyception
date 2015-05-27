
#include "stdio.h"
#include "conio.h"
#include "sapclassbasic.h"
#include <vector>
#include <string>

// Static Variables
#define GAMMA_FACTOR			10000
#define MAX_CONFIG_FILES   36       // 10 numbers + 26 lowercase letters
#define STRING_LENGTH		256 
static char configFileNames[MAX_CONFIG_FILES][MAX_PATH] = {0};

#define GVSP_PIX_BAYRG8 (0x01000000 | 0x00080000 | 0x0009)

inline double round(double x)
{
	return (x > 0.0 ? x + 0.5 : x - 0.5);
}

// Functions defined to be used in Grab Examples
BOOL GetOptionsFromQuestions(char *acqServerName, UINT32 *pAcqDeviceIndex, char *configFileName);
BOOL GetCorAcqOptionsFromQuestions(char *acqServerName, UINT32 *pAcqDeviceIndex, char *configFileName);
BOOL GetCorAcqDeviceOptionsFromQuestions(char *acqServerName, UINT32 *pAcqDeviceIndex, BOOL showGigEOnly = FALSE);
BOOL GetCorAcquisitionOptionsFromQuestions(char *acqServerName, UINT32 *pAcqDeviceIndex, char *configFileName);

// Functions defined to be used in LUT Examples
BOOL IsMonoBuffer(SapBuffer* Buffers);
SapData SetDataValue(SapBuffer* Buffers, DWORD *pPrmIndex);
BOOL GetLUTOptionsFromQuestions(SapBuffer* Buffers, SapLut* m_pLut, char *chAcqLutName);
BOOL GetLoadLUTFiles(SapLut* Lut, std::vector<std::string> v_list, BOOL* bLutLoaded);
std::vector<std::string> GetLUTFilesSaved(char Dossier[STRING_LENGTH], std::vector<std::string> pList, BOOL bPrint);

#ifndef SAP_CAMERA_SDK
BOOL GetLoadDynamicLUTFiles(SapDynamicLut* DynamicLut, int lutIndex, std::vector<std::string> v_list, BOOL* bLutLoaded);
#endif

// General Functions
int GetKeyCharIndex(char key);

