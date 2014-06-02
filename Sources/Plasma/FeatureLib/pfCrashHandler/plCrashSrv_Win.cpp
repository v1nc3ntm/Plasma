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
#include "plCrashBase.h"

#include "plCrashSrv.h"
#include "plCrash_Private.h"
#include "plProduct.h"
#include "plFileSystem.h"

#include <dbghelp.h>
#include <shlobj.h>

struct plCrashSrv::Private {
    static plCrashMemLink* sLink;
    static HANDLE sLinkH;

    void IHandleCrash();
};
plCrashMemLink* plCrashSrv::Private::sLink = nullptr;
HANDLE plCrashSrv::Private::sLinkH = NULL;

plCrashSrv::plCrashSrv(const char* file)
{
    // Init semas
    plCrashBase::Init(file);

    // Open the linked memory
    sLinkH = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, file);
    hsAssert(sLinkH, "Failed to open plCrashHandler mapping");
    if (!sLinkH)
        return;

    // Try to map it
    fLink = (plCrashMemLink*)MapViewOfFile(sLinkH, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(plCrashMemLink));
    hsAssert(sLink, "Failed to map plCrashMemLink");
}

plCrashSrv::~plCrashSrv()
{
    if (sLink)
        UnmapViewOfFile((LPCVOID)sLink);
    if (sLinkH)
        CloseHandle(sLinkH);
    
    plCrashBase::Close();
}

void plCrashSrv::IHandleCrash()
{
    plFileName dumpPath = plFileName::Join(plFileSystem::GetLogPath(), "crash.dmp");
    HANDLE file = CreateFileW(dumpPath.AsString().ToWchar(),
                              GENERIC_WRITE,
                              0,
                              NULL,
                              CREATE_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL,
                              NULL
    );

    MINIDUMP_EXCEPTION_INFORMATION e;
    e.ClientPointers = TRUE;
    e.ExceptionPointers = sLink->fExceptionPtrs;
    e.ThreadId = sLink->fClientThreadID;
    MiniDumpWriteDump(sLink->fClientProcess, sLink->fClientProcessID, file, MiniDumpNormal, &e, NULL, NULL);
    CloseHandle(file);
}

void plCrashSrv::HandleCrash()
{
    if (!sLink)
        FATAL("plCrashMemLink is nil!");
    sLink->fSrvReady = true; // mark us as ready to receive crashes

    // In Win32 land we have to hackily handle the client process exiting, so we'll wait on both
    // the crashed semaphore and the client process...
    HANDLE hack[2] = { sLink->fClientProcess, plCrashBase::gCrashed->GetHandle() };
    DWORD result = WaitForMultipleObjects(arrsize(hack), hack, FALSE, INFINITE);
    hsAssert(result != WAIT_FAILED, "WaitForMultipleObjects failed");
    
    if (sLink->fCrashed)
        IHandleCrash();
    plCrashBase::gHandled->Signal(); // Tell CrashCli we handled it
}
