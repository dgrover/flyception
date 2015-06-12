// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <windows.h>
#include <iostream>
#include <stdio.h>
#include <tchar.h>
#include <vector>
#include <omp.h>
#include <queue>
#include <algorithm>
#include <numeric>
#include <mmsystem.h>

#include "FlyCapture2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <NIDAQmx.h>

#include "pgrcam.h"

#include "fmfreader.h"
#include "fmfwriter.h"

#include "tracker.h"
#include "daq.h"
#include "arduino.h"
#include "utility.h"

// Disable deprecated function warnings with Visual Studio 2005
//#if defined(_MSC_VER) && _MSC_VER >= 1400
//#pragma warning(disable: 4995)
//#endif

#include "conio.h"
#include "sapclassbasic.h"
#include "ExampleUtils.h"

// Restore deprecated function warnings with Visual Studio 2005
//#if defined(_MSC_VER) && _MSC_VER >= 1400
//#pragma warning(default: 4995)
//#endif

#define BASE_HEIGHT 7.175			//in mm
#define GALVO_HEIGHT 65.0			//in mm
#define GALVO_X_MIRROR_ANGLE 15		//in degrees
#define ARENA_RADIUS 20				//in mm
#define TAIL_LENGTH 100

#define SCALEX 0.00075
#define SCALEY 0.00075

#define NFLIES 1
#define NLOSTFRAMES 5
#define MAXRECFRAMES 110000

// TODO: reference additional headers your program requires here
