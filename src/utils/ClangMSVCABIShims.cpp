/**
 * Copyright (C) 2026 Roman Hiestand
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

/*
 * Only built by clang on Windows (see CMakeLists.txt), against MSVC-built DLLs.
 *
 * boost::archive::archive_exception has a virtual base (std::exception), so
 * destroying a complete one goes through its "vbase destructor" (??_D). Boost
 * dllimports the class member by member, and the DLL exports ~archive_exception
 * (??1) but not ??_D: MSVC never imports ??_D, it emits it in every module that
 * needs it. Clang instead imports it along with the destructor, and
 * boost::serialization::throw_exception, which throws archive_exception by
 * value, leaves the link with an undefined
 *   __imp_??_Darchive_exception@archive@boost@@QEAAXXZ
 *
 * So it is defined here, the way MSVC would: the destructor proper, then the
 * virtual base. lld-link points the __imp_ symbol at this local definition
 * (with a "locally defined symbol imported" warning).
 */

#include <exception>
#include <boost/archive/archive_exception.hpp>

// ~archive_exception() itself, through the import library's thunk. Unlike the
// vbase destructor, it does not destroy the virtual base.
extern void r3dArchiveExceptionDtor(boost::archive::archive_exception *pThis)
	__asm__("??1archive_exception@archive@boost@@UEAA@XZ");

extern "C" void r3dArchiveExceptionVBaseDtor(boost::archive::archive_exception *pThis)
	__asm__("??_Darchive_exception@archive@boost@@QEAAXXZ");

extern "C" void r3dArchiveExceptionVBaseDtor(boost::archive::archive_exception *pThis)
{
	// Locate the virtual base while the vbtable pointer is still intact
	std::exception *pBase = static_cast<std::exception *>(pThis);
	r3dArchiveExceptionDtor(pThis);
	pBase->std::exception::~exception();
}
