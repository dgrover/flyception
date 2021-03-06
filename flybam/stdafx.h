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
#include "conio.h"
//#include <mmsystem.h>
#include <concurrent_queue.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "FlyCapture2.h"
#include <NIDAQmx.h>

#include "pgrcam.h"
#include "fmfreader.h"
#include "fvfmfwriter.h"
#include "avfmfwriter.h"
#include "tracker.h"
#include "daq.h"
//#include "arduino.h"
#include "utility.h"

#include "sapclassbasic.h"
#include "ExampleUtils.h"

#define BASE_HEIGHT -7.175			//in mm
#define GALVO_Y_HEIGHT 68.167			//in mm
#define GALVO_XY_DIST 15.174			//in mm
#define GALVO_X_MIRROR_ANGLE 15		//in degrees
#define GALVO_STEP_SIZE 0.0000075		// doubled the step size to maintain similar manual speed control (due to spectre/meltdown processor slowdown)

#define ARENA_X_RADIUS 22.159			//in mm
#define ARENA_Y_RADIUS 20				//in mm

//#define TAIL_LENGTH 100

#define XVOLTPERDEGREE 0.55
#define YVOLTPERDEGREE 0.525

#define XOFFSET -0.25		// x-offset for centering galvo to target (in volts)
#define YOFFSET -0.315		// y-offset for centering galvo to target (in volts)

#define SCALEX 0.0008
#define SCALEY 0.0008

#define NFLIES 1
#define NLOSTFRAMES 5
#define MAXFVRECFRAMES 1000*100
#define MAXAVRECFRAMES 100*100

// TODO: reference additional headers your program requires here
