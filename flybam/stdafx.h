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

#include "FlyCapture2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <NIDAQmx.h>

#include "pgrcam.h"

#include "fmfreader.h"
#include "fmfwriter.h"

#include "tracker.h"
#include "daq.h"

// TODO: reference additional headers your program requires here
