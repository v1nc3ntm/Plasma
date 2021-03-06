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
/*****************************************************************************
*
*   $/Plasma20/Sources/Plasma/NucleusLib/pnAsyncCore/pnAcIo.cpp
*   
***/

#include "pnAcSocket.h"
#include "pnAcInt.h"
#include "pnUtils/pnUtils.h"
#include "hsThread.h"
#pragma hdrstop


/****************************************************************************
*
*   ISocketConnHash
*
***/

// socket notification procedures

// connection data format:
//      uint8_t    connType;
//      uint32_t   buildId;    [optional]
//      uint32_t   branchId;   [optional]
//      uint32_t   buildType;  [optional]
//      plUUID  productId;  [optional]
const unsigned kConnHashFlagsIgnore     = 0x01;
const unsigned kConnHashFlagsExactMatch = 0x02;
struct ISocketConnHash {
    unsigned    connType;
    unsigned    buildId;
    unsigned    buildType;
    unsigned    branchId;
    plUUID      productId;
    unsigned    flags;

    unsigned GetHash () const;
    bool operator== (const ISocketConnHash & rhs) const;
};

struct ISocketConnType : ISocketConnHash {
    HASHLINK(ISocketConnType)   hashlink;
    AsyncSocket::FNotifyProc    notifyProc;
};


static hsReaderWriterLock s_notifyProcLock;
static HASHTABLEDECL(
    ISocketConnType,
    ISocketConnHash,
    hashlink
) s_notifyProcs;


//===========================================================================
unsigned ISocketConnHash::GetHash () const {
    CHashValue hash;
    hash.Hash32(connType);
/*
    if (buildId)
        hash.Hash32(buildId);
    if (buildType)
        hash.Hash32(buildType);
    if (branchId)
        hash.Hash32(branchId);
    if (productId != kNilUuid)
        hash.Hash(&productId, sizeof(productId));
*/
    return hash.GetHash();
}

//===========================================================================
bool ISocketConnHash::operator== (const ISocketConnHash & rhs) const {
    ASSERT(flags & kConnHashFlagsIgnore);

    do {
        // Check connType
        if (connType != rhs.connType)
            break;

        // Check buildId
        if (buildId != rhs.buildId) {
            if (rhs.flags & kConnHashFlagsExactMatch)
                break;
            if (buildId)
                break;
        }

        // Check buildType
        if (buildType != rhs.buildType) {
            if (rhs.flags & kConnHashFlagsExactMatch)
                break;
            if (buildType)
                break;
        }

        // Check branchId
        if (branchId != rhs.branchId) {
            if (rhs.flags & kConnHashFlagsExactMatch)
                break;
            if (branchId)
                break;
        }

        // Check productId
        if (productId != rhs.productId) {
            if (rhs.flags & kConnHashFlagsExactMatch)
                break;
            if (productId != kNilUuid)
                break;
        }

        // Success!
        return true;
    } while (false);

    // Failed!
    return false;
}

//===========================================================================
static unsigned GetConnHash (
    ISocketConnHash *   hash,
    const uint8_t          buffer[],
    unsigned            bytes
) {
    if (!bytes)
        return 0;

    if (IS_TEXT_CONNTYPE(buffer[0])) {
        hash->connType  = buffer[0];
        hash->buildId   = 0;
        hash->buildType = 0;
        hash->branchId  = 0;
        hash->productId = kNilUuid;
        hash->flags     = 0;

        // one uint8_t consumed
        return 1;
    }
    else {
        if (bytes < sizeof(AsyncSocket::ConnectPacket))
            return 0;

        const AsyncSocket::ConnectPacket & connect = * (const AsyncSocket::ConnectPacket *) buffer;
        if (connect.hdrBytes < sizeof(connect))
            return 0;
        
        hash->connType  = connect.connType;
        hash->buildId   = connect.buildId;
        hash->buildType = connect.buildType;
        hash->branchId  = connect.branchId;
        hash->productId = connect.productId;
        hash->flags     = 0;

        return connect.hdrBytes;
    }
}


/****************************************************************************
*
*   Public exports
*
***/

//===========================================================================
void AsyncSocket::Register (
    EConnType       connType, 
    FNotifyProc     notifyProc,
    unsigned        buildId,
    unsigned        buildType,
    unsigned        branchId,
    const plUUID&   productId
) {
    ASSERT(connType != kConnTypeNil);
    ASSERT(notifyProc);

    // Perform memory allocation outside lock
    ISocketConnType * ct    = new ISocketConnType;
    ct->notifyProc          = notifyProc;
    ct->connType            = connType;
    ct->buildId             = buildId;
    ct->buildType           = buildType;
    ct->branchId            = branchId;
    ct->productId           = productId;
    ct->flags               = kConnHashFlagsIgnore;

    hsLockForWriting lock(s_notifyProcLock);
    s_notifyProcs.Add(ct);
}

//===========================================================================
void AsyncSocket::Unregister (
    EConnType       connType, 
    FNotifyProc     notifyProc,
    unsigned        buildId,
    unsigned        buildType,
    unsigned        branchId,
    const plUUID&   productId
) {
    ISocketConnHash hash;
    hash.connType   = connType;
    hash.buildId    = buildId;
    hash.buildType  = buildType;
    hash.branchId   = branchId;
    hash.productId  = productId;
    hash.flags      = kConnHashFlagsExactMatch;

    ISocketConnType * scan;
    {
        hsLockForWriting lock(s_notifyProcLock);

        scan = s_notifyProcs.Find(hash);
        for (; scan; scan = s_notifyProcs.FindNext(hash, scan)) {
            if (scan->notifyProc != notifyProc)
                continue;

            // Unlink the object so it can be deleted outside the lock
            s_notifyProcs.Unlink(scan);
            break;
        }
    }

    // perform memory deallocation outside the lock
    delete scan;
}

//===========================================================================
AsyncSocket::FNotifyProc AsyncSocket::FindNotifyProc (
    const uint8_t   buffer[],
    unsigned        bytes,
    unsigned *      bytesProcessed,
    unsigned *      connType,
    unsigned *      buildId,
    unsigned *      buildType,
    unsigned *      branchId,
    plUUID*         productId
) {
    do {
        // Get the connType
        ISocketConnHash hash;
        *bytesProcessed = GetConnHash(&hash, buffer, bytes);
        if (!*bytesProcessed)
            break;

        // Lookup notifyProc based on connType
        FNotifyProc proc;
        {
            hsLockForReading lock(s_notifyProcLock);
            if (const ISocketConnType * scan = s_notifyProcs.Find(hash))
                proc = scan->notifyProc;
            else
                proc = nullptr;
        }
        if (!proc)
            break;

        // Success!
        *connType   = hash.connType;
        *buildId    = hash.buildId;
        *buildType  = hash.buildType;
        *branchId   = hash.branchId;
        *productId  = hash.productId;
        return proc;
    } while (false);

    // Failure!
    PerfAddCounter(kAsyncPerfSocketDisconnectInvalidConnType, 1);
    *bytesProcessed = 0;
    *connType       = 0;
    *buildId        = 0;
    *buildType      = 0;
    *branchId       = 0;
    *productId      = kNilUuid;
    return nil;
}
