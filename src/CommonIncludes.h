#ifndef COMMONINCLUDES_H
#define COMMONINCLUDES_H

// Silence warnings in wxWidgets
#define _CRT_SECURE_NO_WARNINGS

// Define the windows API level we'd like to use: Windows XP SP2
#define _WIN32_WINNT 0x0502

#include "config.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// Workaround for missing symbols like _wxTheAssertHandler in MinGW
#define wxDEBUG_LEVEL 0

// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
#pragma hdrstop
#endif

#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include "wx/datetime.h"
#include "wx/image.h"
#include "wx/bookctrl.h"
#include "wx/artprov.h"
#include "wx/imaglist.h"
#include "wx/sysopt.h"
#include <wx/aboutdlg.h>
#include <wx/timer.h>
#include <wx/dnd.h>
#include <wx/filename.h>
#include <wx/arrstr.h>
#include <wx/file.h>

#ifdef __DARWIN__
    #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif

#include <wx/glcanvas.h>

#undef _CRT_SECURE_NO_WARNINGS

// X11 defines Status, True and False which leads to loads of problems, let's undefine it
#if defined(Status)
#undef Status
#endif
#if defined(True)
#undef True
#endif
#if defined(False)
#undef False
#endif
#if defined(Success)
#undef Success
#endif

#if defined(ERROR)
#undef ERROR
#endif

#endif
