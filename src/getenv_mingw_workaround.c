/**
 * Copyright (C) 2015 Roman Hiestand
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * The following is a workaround for a problem in MinGW.
 * Somewhere deep in iostreams, the function _pformat_exponent_digits constantly
 * calls getenv("PRINTF_EXPONENT_DIGITS"), which slows down streaming considerably.
 * A fix is in the pipeline (see http://sourceforge.net/p/mingw-w64/mailman/message/33683770/),
 * but not yet released. This is a workaround.
 */

#define VC_EXTRALEAN
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
 
typedef char *(*getenv_ptr)(const char *);
 
char *getenv(const char *varname)
{
	const char comparestr[] = "PRINTF_EXPONENT_DIGITS";
	const int sizeof_comparestr = sizeof(comparestr);
	int i, isdifferent = 0;
	char varc, compc;
	getenv_ptr getenv_ptr_m = NULL;

	if(varname == 0)
		return 0;

	/* Compare strings */
	for(i  = 0; i < sizeof_comparestr; i++)
	{
		varc = varname[i];
		compc = comparestr[i];
		if(varc != compc)
		{
			isdifferent = 1;
			break;
		}
	}

	/* Strings are different -> call getenv from msvcrt.dll */
	if(isdifferent != 0)
	{
		HMODULE hModMSVCRT = GetModuleHandleW(L"msvcrt.dll");
		if(hModMSVCRT != NULL)
		{
			getenv_ptr_m = (getenv_ptr)GetProcAddress(hModMSVCRT, "getenv");
			if(getenv_ptr_m != NULL)
				return (*getenv_ptr_m)(varname);
		}
	}

	return 0;
}

