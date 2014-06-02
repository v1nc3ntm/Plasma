/*==LICENSE==*

CyanWorlds.com Engine - MMOG client, server and tools
Copyright (C) 2011  Cyan Worlds, Inc.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Additional permissions under GNU GPL version 3 section 7

If you modify this Program, or any covered work, by linking or
combining it with any of RAD Game Tools Bink SDK, Autodesk 3ds Max SDK,
NVIDIA PhysX SDK, Microsoft DirectX SDK, OpenSSL library, Independent
JPEG Group JPEG library, Microsoft Windows Media SDK, or Apple QuickTime SDK
(or a modified version of those libraries),
containing parts covered by the terms of the Bink SDK EULA, 3ds Max EULA,
PhysX SDK EULA, DirectX SDK EULA, OpenSSL and SSLeay licenses, IJG
JPEG Library README, Windows Media SDK EULA, or QuickTime SDK EULA, the
licensors of this Program grant you additional
permission to convey the resulting work. Corresponding Source for a
non-source form of such a combination shall include the source code for
the parts of OpenSSL and IJG JPEG Library used as well as that of the covered
work.

You can contact Cyan Worlds, Inc. by email legal@cyan.com
 or by snail mail at:
      Cyan Worlds, Inc.
      14617 N Newport Hwy
      Mead, WA   99021

*==LICENSE==*/

#include "HeadSpin.h"
#include "hsWindows.h"
#include "plCrashBase.h"

#include "plCrashCli.h"
#include "plCrash_Private.h"

#ifdef _MSC_VER
#include <crtdbg.h>

namespace {
    void IInvalidParameter (const wchar_t* expression, const wchar_t* function, const wchar_t* file, unsigned int line, uintptr_t)
    {
        __debugbreak();
    }

    void IPureVirtualCall ()
    {
        __debugbreak();
    }
}

#endif // _MSC_VER

struct plCrashCli::Private {
    static PROCESS_INFORMATION sCrashSrv;
    static HANDLE sLinkH;
    static plCrashMemLink* sLink;
    
    static LONG WINAPI Handler (_EXCEPTION_POINTERS*);
    
    static LPTOP_LEVEL_EXCEPTION_FILTER sOldFilter;
};
PROCESS_INFORMATION plCrashCli::Private::sCrashSrv = { 0 };
HANDLE plCrashCli::Private::sLinkH;
plCrashMemLink* plCrashCli::Private::sLink = nullptr;
LPTOP_LEVEL_EXCEPTION_FILTER plCrashCli::Private::sOldFilter;

plCrashCli::plCrashCli ()
{
    hsAssert(!sLink, "only one instance of plCrashCli is authorized");
    if (sLink)
        return;
    
    char mapname[128];
    snprintf(mapname, arrsize(mapname), "Plasma20CrashHandler-%u", GetCurrentProcessId());
    // Initialize the semas
    plCrashBase::Init(mapname);

    // Initialize the shared memory
    sLinkH = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(plCrashMemLink), mapname);
    hsAssert(sLinkH, "Failed to create plCrashHandler mapping");
    if (!sLinkH)
        return;
    
    // Map the shared memory
    sLink = (plCrashMemLink*)MapViewOfFile(sLinkH, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(plCrashMemLink));
    hsAssert(sLink, "Failed to map plCrashLinkedMem");
    if (!sLink)
        return;
    memset(sLink, 0, sizeof(plCrashMemLink));
    sLink->fClientProcessID = GetCurrentProcessId();

    // Start the plCrashHandler before a crash
    char cmdline[128];
    STARTUPINFOA info = { 0 };
    snprintf(cmdline, arrsize(cmdline), "%s %s", CRASH_HANDLER_EXE, mapname);
    info.cb = sizeof(STARTUPINFOA);
    CreateProcessA(
                   CRASH_HANDLER_EXE, // plCrashHandler.exe
                   cmdline,           // plCrashHandler.exe Plasma20CrashHandler-%u
                   NULL,
                   NULL,
                   FALSE,
                   CREATE_NO_WINDOW, // Don't create any new windows or consoles
                   NULL,
                   NULL,             // Use the directory of the current plClient
                   &info,
                   &sCrashSrv        // Save the CrashSrv handles
    );

    HANDLE curProc = GetCurrentProcess();
    DuplicateHandle(curProc,                  // Handle to the source process
                    curProc,                  // Handle that we want duplicated
                    sCrashSrv.hProcess,       // Handle to target process
                    &fLink->fClientProcess,   // Pointer to Handle to dupliicate to
                    0,                        // Ignored
                    FALSE,
                    DUPLICATE_CLOSE_SOURCE | DUPLICATE_SAME_ACCESS
    );

#ifdef _MSC_VER
    // Sigh... The Visual C++ Runtime likes to throw up dialogs sometimes.
    // The user cares not about dialogs. We just want to get a minidump...
    // See: http://www.altdevblogaday.com/2012/07/20/more-adventures-in-failing-to-crash-properly/
    _set_invalid_parameter_handler(IInvalidParameter); // TODO
    _set_purecall_handler(IPureVirtualCall); // TODO
#endif // _MSC_VER
    
    // install handler
    sOldFilter = SetUnhandledExceptionFilter(Private::Handler);
}

plCrashCli::~plCrashCli ()
{
    // restore old handler
    SetUnhandledExceptionFilter(sOldFilter);
    
    plCrashBase::gCrashed->Signal(); // forces the CrashSrv to exit, if it's still running
    if (sCrashSrv.hProcess)
    {
        TerminateProcess(sCrashSrv.hProcess, 0);
        CloseHandle(sCrashSrv.hProcess);
    }
    if (sCrashSrv.hThread)
        CloseHandle(sCrashSrv.hThread);
    if (sLink)
        UnmapViewOfFile((LPCVOID)sLink);
    if (sLinkH)
        CloseHandle(sLinkH);
    
    plCrashBase::Close();
}

LONG WINAPI plCrashCli::Private::Handler (_EXCEPTION_POINTERS *ExceptionInfo)
{
    // Before we do __ANYTHING__, pass the exception to plCrashHandler
    hsAssert(sLink, "plCrashMemLink is nil");
    if (sLink)
    {
        sLink->fClientThreadID = GetCurrentThreadId();
        sLink->fCrashed = true;
        sLink->fExceptionPtrs  = ExceptionInfo;
    }
    
    plCrashBase::gCrashed->Signal();

    // Now, try to create a nice exception dialog after plCrashHandler is done.
    if (sLink && sLink->fSrvReady) // Don't deadlock... Only wait if the CrashSrv is attached
        plCrashBase::gHandled->Wait();
    
    // Now, try to create a nice exception dialog after plCrashHandler is done.
    HWND parentHwnd = (gClient == nil) ? GetActiveWindow() : gClient->GetWindowHandle();
    DialogBoxParam(gHInst, MAKEINTRESOURCE(IDD_EXCEPTION), parentHwnd, ExceptionDialogProc, NULL);

    // Trickle up the handlers
    return EXCEPTION_EXECUTE_HANDLER;
}
