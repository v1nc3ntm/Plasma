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
*   $/Plasma20/Sources/Plasma/PubUtilLib/plNetGateKeeperLib/Private/plNglGateKeeper.cpp
*   
***/

#include "../Pch.h"
#include "pnAsyncCore/pnAcTimer.h"
#include "pnAsyncCore/pnAcLog.h"
#include "pnAsyncCore/pnAcDns.h"
#pragma hdrstop

namespace Ngl { namespace GateKeeper {

/*****************************************************************************
*
*   Private
*
***/
    
struct CliGkConn : hsRefCnt {
    CliGkConn ();
    ~CliGkConn ();

    // Reconnection
    AsyncTimer          reconnectTimer;
    unsigned            reconnectStartMs;

    // Ping
    AsyncTimer          pingTimer;
    unsigned            pingSendTimeMs;
    unsigned            lastHeardTimeMs;

    // This function should be called during object construction
    // to initiate connection attempts to the remote host whenever
    // the socket is disconnected.
    void AutoReconnect ();
    bool AutoReconnectEnabled ();
    void StopAutoReconnect (); // call before destruction
    void StartAutoReconnect ();
    void TimerReconnect ();

    // ping
    void AutoPing ();
    void StopAutoPing ();
    void TimerPing ();

    void Send (const uintptr_t fields[], unsigned count);

    std::mutex          critsect;
    LINK(CliGkConn)     link;
    AsyncSocket *       sock;
    NetCli *            cli;
    char                name[MAX_PATH];
    plNetAddress        addr;
    plUUID              token;
    unsigned            seq;
    unsigned            serverChallenge;
    AsyncSocket::Cancel cancelId;
    bool                abandoned;
};


//============================================================================
// PingRequestTrans
//============================================================================
struct PingRequestTrans : NetGateKeeperTrans {
    FNetCliGateKeeperPingRequestCallback    m_callback;
    void *                                  m_param;
    unsigned                                m_pingAtMs;
    unsigned                                m_replyAtMs;
    ARRAY(uint8_t)                             m_payload;
    
    PingRequestTrans (
        FNetCliGateKeeperPingRequestCallback    callback,
        void *                                  param,
        unsigned                                pingAtMs,
        unsigned                                payloadBytes,
        const void *                            payload
    );

    bool Send ();
    void Post ();
    bool Recv (
        const uint8_t  msg[],
        unsigned    bytes
    );
};

//============================================================================
// FileSrvIpAddressRequestTrans
//============================================================================
struct FileSrvIpAddressRequestTrans : NetGateKeeperTrans {
    FNetCliGateKeeperFileSrvIpAddressRequestCallback    m_callback;
    void *                                              m_param;
    wchar_t                                               m_addr[64];
    bool                                                m_isPatcher;
    
    FileSrvIpAddressRequestTrans (
        FNetCliGateKeeperFileSrvIpAddressRequestCallback    callback,
        void *                                              param,
        bool                                                isPatcher
    );

    bool Send ();
    void Post ();
    bool Recv (
        const uint8_t  msg[],
        unsigned    bytes
    );
};

//============================================================================
// AuthSrvIpAddressRequestTrans
//============================================================================
struct AuthSrvIpAddressRequestTrans : NetGateKeeperTrans {
    FNetCliGateKeeperAuthSrvIpAddressRequestCallback    m_callback;
    void *                                              m_param;
    wchar_t                                               m_addr[64];
    
    AuthSrvIpAddressRequestTrans (
        FNetCliGateKeeperAuthSrvIpAddressRequestCallback    callback,
        void *                                              param
    );

    bool Send ();
    void Post ();
    bool Recv (
        const uint8_t  msg[],
        unsigned    bytes
    );
};


/*****************************************************************************
*
*   Private data
*
***/

enum {
    kPerfConnCount,
    kPingDisabled,
    kAutoReconnectDisabled,
    kNumPerf
};

static bool                         s_running;
static std::mutex                   s_critsect;
static LISTDECL(CliGkConn, link)    s_conns;
static CliGkConn *                  s_active;

static std::atomic<long>            s_perf[kNumPerf];



/*****************************************************************************
*
*   Internal functions
*
***/

//===========================================================================
static unsigned GetNonZeroTimeMs () {
    if (unsigned ms = hsTimer::GetMilliSeconds<uint32_t>())
        return ms;
    return 1;
}

//============================================================================
static CliGkConn * GetConnIncRef_CS (const char tag[]) {
    if (CliGkConn * conn = s_active) {
        conn->Ref(tag);
        return conn;
    }
    return nil;
}

//============================================================================
static CliGkConn * GetConnIncRef (const char tag[]) {
    std::lock_guard<std::mutex> lock(s_critsect);
    return GetConnIncRef_CS(tag);
}

//============================================================================
static void UnlinkAndAbandonConn_CS (CliGkConn * conn) {
    s_conns.Unlink(conn);
    conn->abandoned = true;

    conn->StopAutoReconnect();

    if (conn->cancelId)
        conn->cancelId.ConnectCancel();
    else if (conn->sock)
        conn->sock->Disconnect(true);
    else
        conn->UnRef("Lifetime");
}

//============================================================================
static void SendClientRegisterRequest (CliGkConn * conn) {
/*    const uintptr_t msg[] = {
        kCli2GateKeeper_ClientRegisterRequest,
        BuildId(),
    };

    conn->Send(msg, arrsize(msg));*/
}

//============================================================================
static bool ConnEncrypt (ENetError error, void * param) {
    CliGkConn * conn = (CliGkConn *) param;
        
    if (IS_NET_SUCCESS(error)) {

        //SendClientRegisterRequest(conn);

        if (!s_perf[kPingDisabled])
            conn->AutoPing();

        {
            std::lock_guard<std::mutex> lock(s_critsect);
            std::swap(s_active, conn);
        }

    //  AuthConnectedNotifyTrans * trans = new AuthConnectedNotifyTrans;
    //  NetTransSend(trans);
    }

    return IS_NET_SUCCESS(error);
}

//============================================================================
static void NotifyConnSocketConnect (CliGkConn * conn) {

    conn->TransferRef("Connecting", "Connected");
    conn->cli = NetCliConnectAccept(
        conn->sock,
        kNetProtocolCli2GateKeeper,
        false,
        ConnEncrypt,
        0,
        nil,
        conn
    );
}

//============================================================================
static void CheckedReconnect (CliGkConn * conn, ENetError error) {

    unsigned disconnectedMs = GetNonZeroTimeMs() - conn->lastHeardTimeMs;

    // no auto-reconnect or haven't heard from the server in a while?
    if (!conn->AutoReconnectEnabled() || (int)disconnectedMs >= (int)kDisconnectedTimeoutMs) {
        // Cancel all transactions in progress on this connection.
        NetTransCancelByConnId(conn->seq, kNetErrTimeout);
        // conn is dead.
        conn->UnRef("Lifetime");
        ReportNetError(kNetProtocolCli2GateKeeper, error);
    }
    else {
        if (conn->token != kNilUuid)
            // previously encrypted; reconnect quickly
            conn->reconnectStartMs = GetNonZeroTimeMs() + 500;
        else
            // never encrypted; reconnect slowly
            conn->reconnectStartMs = GetNonZeroTimeMs() + kMaxReconnectIntervalMs;

        // clean up the socket and start reconnect
        if (conn->cli)
            NetCliDelete(conn->cli, true);
        conn->cli = nil;
        conn->sock = nil;
        
        conn->StartAutoReconnect();
    }
}

//============================================================================
static void NotifyConnSocketConnectFailed (CliGkConn * conn) {

    {
        std::lock_guard<std::mutex> lock(s_critsect);

        conn->cancelId.Clear();
        s_conns.Unlink(conn);

        if (conn == s_active)
            s_active = nil;
    }
    
    CheckedReconnect(conn, kNetErrConnectFailed);
    
    conn->UnRef("Connecting");
}

//============================================================================
static void NotifyConnSocketDisconnect (CliGkConn * conn) {

    conn->StopAutoPing();

    {
        std::lock_guard<std::mutex> lock(s_critsect);

        conn->cancelId.Clear();
        s_conns.Unlink(conn);
            
        if (conn == s_active)
            s_active = nil;
    }

    // Cancel all transactions in process on this connection.
    NetTransCancelByConnId(conn->seq, kNetErrTimeout);
    conn->UnRef("Connected");
    conn->UnRef("Lifetime");
}

//============================================================================
static bool NotifyConnSocketRead (CliGkConn * conn, AsyncSocket::NotifyRead * read) {
    // TODO: Only dispatch messages from the active auth server
    conn->lastHeardTimeMs = GetNonZeroTimeMs();
    bool result = NetCliDispatch(conn->cli, read->buffer, read->bytes, conn);
    read->bytesProcessed += read->bytes;
    return result;
}

//============================================================================
static bool SocketNotifyCallback (
    AsyncSocket *           sock,
    AsyncSocket::ENotify    code,
    AsyncSocket::Notify *   notify
) {
    bool result = true;
    CliGkConn * conn;

    switch (code) {
        case AsyncSocket::kNotifyConnectSuccess:
            conn = (CliGkConn *) notify->param;
            sock->user = conn;
            bool abandoned;
            {
                std::lock_guard<std::mutex> lock(s_critsect);
                conn->sock      = sock;
                conn->cancelId.Clear();
                abandoned       = conn->abandoned;
            }
            if (abandoned)
                sock->Disconnect(true);
            else
                NotifyConnSocketConnect(conn);
        break;

        case AsyncSocket::kNotifyConnectFailed:
            conn = (CliGkConn *) notify->param;
            NotifyConnSocketConnectFailed(conn);
        break;

        case AsyncSocket::kNotifyDisconnect:
            conn = (CliGkConn *) sock->user;
            NotifyConnSocketDisconnect(conn);
        break;

        case AsyncSocket::kNotifyRead:
            conn = (CliGkConn *) sock->user;
            result = NotifyConnSocketRead(conn, (AsyncSocket::NotifyRead *) notify);
        break;
    }
    
    return result;
}

//============================================================================
static void Connect (
    CliGkConn * conn
) {
    ASSERT(s_running);

    conn->pingSendTimeMs = 0;

    {
        std::lock_guard<std::mutex> lock(s_critsect);

        while (CliGkConn * oldConn = s_conns.Head()) {
            if (oldConn != conn)
                UnlinkAndAbandonConn_CS(oldConn);
            else
                s_conns.Unlink(oldConn);
        }
        s_conns.Link(conn);
    }
    
    Cli2GateKeeper_Connect connect;
    connect.hdr.connType        = kConnTypeCliToGateKeeper;
    connect.hdr.hdrBytes        = sizeof(connect.hdr);
    connect.hdr.buildId         = plProduct::BuildId();
    connect.hdr.buildType       = plProduct::BuildType();
    connect.hdr.branchId        = plProduct::BranchId();
    connect.hdr.productId       = plProduct::UUID();
    connect.data.token          = conn->token;
    connect.data.dataBytes      = sizeof(connect.data);

    conn->cancelId = AsyncSocket::Connect(
        conn->addr,
        SocketNotifyCallback,
        conn,
        &connect,
        sizeof(connect),
        0,
        0
    );
}

//============================================================================
static void Connect (
    const char          name[],
    const plNetAddress& addr
) {
    ASSERT(s_running);
    
    CliGkConn * conn        = new CliGkConn;
    conn->addr              = addr;
    conn->seq               = ConnNextSequence();
    conn->lastHeardTimeMs   = GetNonZeroTimeMs();   // used in connect timeout, and ping timeout
    strncpy(conn->name, name, arrsize(conn->name));

    conn->Ref("Lifetime");
    conn->AutoReconnect();
}

//============================================================================
static void AsyncLookupCallback (
    void *              param,
    const char          name[],
    unsigned            addrCount,
    const plNetAddress  addrs[]
) {
    if (!addrCount) {
        ReportNetError(kNetProtocolCli2GateKeeper, kNetErrNameLookupFailed);
        return;
    }

    for (unsigned i = 0; i < addrCount; ++i) {
        Connect(name, addrs[i]);
    }
}




/*****************************************************************************
*
*   CliGkConn
*
***/

//===========================================================================
static unsigned CliGkConnTimerDestroyed (void * param) {
    CliGkConn * conn = (CliGkConn *) param;
    conn->UnRef("TimerDestroyed");
    return kPosInfinity32;
}

//===========================================================================
static unsigned CliGkConnReconnectTimerProc (void * param) {
    ((CliGkConn *) param)->TimerReconnect();
    return kPosInfinity32;
}

//===========================================================================
static unsigned CliGkConnPingTimerProc (void * param) {
    ((CliGkConn *) param)->TimerPing();
    return kPingIntervalMs;
}

//============================================================================
CliGkConn::CliGkConn ()
    : hsRefCnt(0), reconnectStartMs(0)
    , pingSendTimeMs(0), lastHeardTimeMs(0)
    , sock(nil), cli(nil), seq(0), serverChallenge(0)
    , abandoned(false)
{
    memset(name, 0, sizeof(name));

    ++s_perf[kPerfConnCount];
}

//============================================================================
CliGkConn::~CliGkConn () {
    if (cli)
        NetCliDelete(cli, true);
    --s_perf[kPerfConnCount];
}

//===========================================================================
void CliGkConn::TimerReconnect () {
    ASSERT(!sock);
    ASSERT(!cancelId);
    
    if (!s_running) {
        std::lock_guard<std::mutex> lock(s_critsect);
        UnlinkAndAbandonConn_CS(this);
    }
    else {
        Ref("Connecting");

        // Remember the time we started the reconnect attempt, guarding against
        // TimeGetMs() returning zero (unlikely), as a value of zero indicates
        // a first-time connect condition to StartAutoReconnect()
        reconnectStartMs = GetNonZeroTimeMs();

        Connect(this);
    }
}

//===========================================================================
// This function is called when after a disconnect to start a new connection
void CliGkConn::StartAutoReconnect () {
    std::lock_guard<std::mutex> lock(critsect);

    if (reconnectTimer && !s_perf[kAutoReconnectDisabled]) {
        // Make reconnect attempts at regular intervals. If the last attempt
        // took more than the specified max interval time then reconnect
        // immediately; otherwise wait until the time interval is up again
        // then reconnect.
        unsigned remainingMs = 0;
        if (reconnectStartMs) {
            remainingMs = reconnectStartMs - GetNonZeroTimeMs();
            if ((signed)remainingMs < 0)
                remainingMs = 0;
            LogMsg(kLogPerf, L"GateKeeper auto-reconnecting in %u ms", remainingMs);
        }
        reconnectTimer.Set(remainingMs);
    }
}

//===========================================================================
// This function should be called during object construction
// to initiate connection attempts to the remote host whenever
// the socket is disconnected.
void CliGkConn::AutoReconnect () {
    ASSERT(!reconnectTimer);
    Ref("ReconnectTimer");

    std::lock_guard<std::mutex> lock(critsect);

    reconnectTimer.Create(
        CliGkConnReconnectTimerProc,
        0,  // immediate callback
        this
    );
}

//============================================================================
void CliGkConn::StopAutoReconnect () {
    std::lock_guard<std::mutex> lock(critsect);

    AsyncTimer timer(reconnectTimer);
    if (timer)
        timer.Delete(CliGkConnTimerDestroyed);
}

//============================================================================
bool CliGkConn::AutoReconnectEnabled () {
    
    return reconnectTimer && !s_perf[kAutoReconnectDisabled];
}

//============================================================================
void CliGkConn::AutoPing () {
    ASSERT(!pingTimer);
    Ref("PingTimer");

    std::lock_guard<std::mutex> lock(critsect);

    pingTimer.Create(
        CliGkConnPingTimerProc,
        sock ? 0 : kPosInfinity32,
        this
    );
}

//============================================================================
void CliGkConn::StopAutoPing () {
    std::lock_guard<std::mutex> lock(critsect);

    if (pingTimer)
        pingTimer.Delete(CliGkConnTimerDestroyed);
}

//============================================================================
void CliGkConn::TimerPing () {
    // Send a ping request
    pingSendTimeMs = GetNonZeroTimeMs();

    const uintptr_t msg[] = {
        kCli2GateKeeper_PingRequest,
        pingSendTimeMs,
        0,      // not a transaction
        0,      // no payload
        reinterpret_cast<uintptr_t>(nullptr)
    };

    Send(msg, arrsize(msg));
}

//============================================================================
void CliGkConn::Send (const uintptr_t fields[], unsigned count) {
    std::lock_guard<std::mutex> lock(critsect);

    NetCliSend(cli, fields, count);
    NetCliFlush(cli);
}


/*****************************************************************************
*
*   Cli2GateKeeper message handlers
*
***/

//============================================================================
static bool Recv_PingReply (
    const uint8_t      msg[],
    unsigned        bytes,
    void *
) {
    const GateKeeper2Cli_PingReply & reply = *(const GateKeeper2Cli_PingReply *)msg;

    if (reply.transId)
        NetTransRecv(reply.transId, msg, bytes);

    return true;
}


//============================================================================
static bool Recv_FileSrvIpAddressReply (
    const uint8_t      msg[],
    unsigned        bytes,
    void *
) {
    const GateKeeper2Cli_FileSrvIpAddressReply & reply = *(const GateKeeper2Cli_FileSrvIpAddressReply *)msg;

    if (reply.transId)
        NetTransRecv(reply.transId, msg, bytes);

    return true;
}

//============================================================================
static bool Recv_AuthSrvIpAddressReply (
    const uint8_t      msg[],
    unsigned        bytes,
    void *
) {
    const GateKeeper2Cli_AuthSrvIpAddressReply & reply = *(const GateKeeper2Cli_AuthSrvIpAddressReply *)msg;

    if (reply.transId)
        NetTransRecv(reply.transId, msg, bytes);

    return true;
}


/*****************************************************************************
*
*   Cli2Auth protocol
*
***/

#define MSG(s)  kNetMsg_Cli2GateKeeper_##s
static NetMsgInitSend s_send[] = {
    { MSG(PingRequest)              },
    { MSG(FileSrvIpAddressRequest)  },
    { MSG(AuthSrvIpAddressRequest)  },
};
#undef MSG

#define MSG(s)  kNetMsg_GateKeeper2Cli_##s, Recv_##s
static NetMsgInitRecv s_recv[] = {
    { MSG(PingReply)                },
    { MSG(FileSrvIpAddressReply)    },
    { MSG(AuthSrvIpAddressReply)    },
};
#undef MSG


/*****************************************************************************
*
*   PingRequestTrans
*
***/

//============================================================================
PingRequestTrans::PingRequestTrans (
    FNetCliGateKeeperPingRequestCallback    callback,
    void *                          param,
    unsigned                        pingAtMs,
    unsigned                        payloadBytes,
    const void *                    payload
) : NetGateKeeperTrans(kPingRequestTrans)
,   m_callback(callback)
,   m_param(param)
,   m_pingAtMs(pingAtMs)
{
    m_payload.Set((const uint8_t *)payload, payloadBytes);
}

//============================================================================
bool PingRequestTrans::Send () {

    if (!AcquireConn())
        return false;

    const uintptr_t msg[] = {
        kCli2GateKeeper_PingRequest,
                        m_pingAtMs,
                        m_transId,
                        m_payload.Count(),
        (uintptr_t)  m_payload.Ptr(),
    };
    
    m_conn->Send(msg, arrsize(msg));
    
    return true;
}

//============================================================================
void PingRequestTrans::Post () {

    m_callback(
        m_result,
        m_param,
        m_pingAtMs,
        m_replyAtMs,
        m_payload.Count(),
        m_payload.Ptr()
    );
}

//============================================================================
bool PingRequestTrans::Recv (
    const uint8_t  msg[],
    unsigned    bytes
) {
    const GateKeeper2Cli_PingReply & reply = *(const GateKeeper2Cli_PingReply *)msg;

    m_payload.Set(reply.payload, reply.payloadBytes);
    m_replyAtMs     = hsTimer::GetMilliSeconds<uint32_t>();
    m_result        = kNetSuccess;
    m_state         = kTransStateComplete;

    return true;
}


/*****************************************************************************
*
*   FileSrvIpAddressRequestTrans
*
***/

//============================================================================
FileSrvIpAddressRequestTrans::FileSrvIpAddressRequestTrans (
    FNetCliGateKeeperFileSrvIpAddressRequestCallback    callback,
    void *                                              param,
    bool                                                isPatcher
) : NetGateKeeperTrans(kGkFileSrvIpAddressRequestTrans)
,   m_callback(callback)
,   m_param(param)
,   m_isPatcher(isPatcher)
{
}

//============================================================================
bool FileSrvIpAddressRequestTrans::Send () {

    if (!AcquireConn())
        return false;

    
    const uintptr_t msg[] = {
        kCli2GateKeeper_FileSrvIpAddressRequest,
                        m_transId,
                        (uintptr_t)(m_isPatcher == true ? 1 : 0)
    };
    
    m_conn->Send(msg, arrsize(msg));
    
    return true;
}

//============================================================================
void FileSrvIpAddressRequestTrans::Post () {

    m_callback(
        m_result,
        m_param,
        m_addr
    );
}

//============================================================================
bool FileSrvIpAddressRequestTrans::Recv (
    const uint8_t  msg[],
    unsigned    bytes
) {
    const GateKeeper2Cli_FileSrvIpAddressReply & reply = *(const GateKeeper2Cli_FileSrvIpAddressReply *)msg;

    
    m_result        = kNetSuccess;
    m_state         = kTransStateComplete;
    StrCopy(m_addr, reply.address, 64);

    return true;
}


/*****************************************************************************
*
*   AuthSrvIpAddressRequestTrans
*
***/

//============================================================================
AuthSrvIpAddressRequestTrans::AuthSrvIpAddressRequestTrans (
    FNetCliGateKeeperFileSrvIpAddressRequestCallback    callback,
    void *                                              param
) : NetGateKeeperTrans(kGkAuthSrvIpAddressRequestTrans)
,   m_callback(callback)
,   m_param(param)
{
}

//============================================================================
bool AuthSrvIpAddressRequestTrans::Send () {

    if (!AcquireConn())
        return false;

    const uintptr_t msg[] = {
        kCli2GateKeeper_AuthSrvIpAddressRequest,
                        m_transId,
    };
    
    m_conn->Send(msg, arrsize(msg));
    
    return true;
}

//============================================================================
void AuthSrvIpAddressRequestTrans::Post () {

    m_callback(
        m_result,
        m_param,
        m_addr
    );
}

//============================================================================
bool AuthSrvIpAddressRequestTrans::Recv (
    const uint8_t  msg[],
    unsigned    bytes
) {
    const GateKeeper2Cli_AuthSrvIpAddressReply & reply = *(const GateKeeper2Cli_AuthSrvIpAddressReply *)msg;

    m_result        = kNetSuccess;
    m_state         = kTransStateComplete;
    StrCopy(m_addr, reply.address, 64);

    return true;
}


}   using namespace GateKeeper;


/*****************************************************************************
*
*   NetGateKeeperTrans
*
***/

//============================================================================
NetGateKeeperTrans::NetGateKeeperTrans (ETransType transType)
:   NetTrans(kNetProtocolCli2GateKeeper, transType)
,   m_conn(nil)
{
}

//============================================================================
NetGateKeeperTrans::~NetGateKeeperTrans () {
    ReleaseConn();
}

//============================================================================
bool NetGateKeeperTrans::AcquireConn () {
    if (!m_conn)
        m_conn = GetConnIncRef("AcquireConn");
    return m_conn != nil;
}

//============================================================================
void NetGateKeeperTrans::ReleaseConn () {
    if (m_conn) {
        m_conn->UnRef("AcquireConn");
        m_conn = nil;
    }
}


/*****************************************************************************
*
*   Protected functions
*
***/

//============================================================================
void GateKeeperInitialize () {
    s_running = true;
    NetMsgProtocolRegister(
        kNetProtocolCli2GateKeeper,
        false,
        s_send, arrsize(s_send),
        s_recv, arrsize(s_recv),
        kGateKeeperDhGValue,
        plBigNum(sizeof(kGateKeeperDhXData), kGateKeeperDhXData),
        plBigNum(sizeof(kGateKeeperDhNData), kGateKeeperDhNData)
    );
}

//============================================================================
void GateKeeperDestroy (bool wait) {
    s_running = false;

    NetTransCancelByProtocol(
        kNetProtocolCli2GateKeeper,
        kNetErrRemoteShutdown
    );    
    NetMsgProtocolDestroy(
        kNetProtocolCli2GateKeeper,
        false
    );

    {
        std::lock_guard<std::mutex> lock(s_critsect);
        while (CliGkConn * conn = s_conns.Head())
            UnlinkAndAbandonConn_CS(conn);
        s_active = nil;
    }

    if (!wait)
        return;

    while (s_perf[kPerfConnCount]) {
        NetTransUpdate();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

//============================================================================
bool GateKeeperQueryConnected () {
    std::lock_guard<std::mutex> lock(s_critsect);
    return (s_active && s_active->cli);
}

//============================================================================
unsigned GateKeeperGetConnId () {
    std::lock_guard<std::mutex> lock(s_critsect);
    return (s_active) ? s_active->seq : 0;
}


}   using namespace Ngl;


/*****************************************************************************
*
*   Exported functions
*
***/

//============================================================================
void NetCliGateKeeperStartConnect (
    const char*   gateKeeperAddrList[],
    uint32_t      gateKeeperAddrCount
) {
    gateKeeperAddrCount = min(gateKeeperAddrCount, 1);

    for (unsigned i = 0; i < gateKeeperAddrCount; ++i) {
        // Do we need to lookup the address?
        const char* name = gateKeeperAddrList[i];
        while (unsigned ch = *name) {
            ++name;
            if (!(isdigit(ch) || ch == L'.' || ch == L':')) {
                AsyncDns::LookupName(
                    gateKeeperAddrList[i],
                    kNetDefaultClientPort,
                    AsyncLookupCallback
                );
                break;
            }
        }
        if (!name[0]) {
            plNetAddress addr(gateKeeperAddrList[i], kNetDefaultClientPort);
            Connect(gateKeeperAddrList[i], addr);
        }
    }
}

//============================================================================
void NetCliGateKeeperDisconnect () {
    std::lock_guard<std::mutex> lock(s_critsect);

    while (CliGkConn * conn = s_conns.Head())
        UnlinkAndAbandonConn_CS(conn);
    s_active = nil;
}

//============================================================================
void NetCliGateKeeperPingRequest (
    unsigned                                pingTimeMs,
    unsigned                                payloadBytes,
    const void *                            payload,
    FNetCliGateKeeperPingRequestCallback    callback,
    void *                                  param
) {
    PingRequestTrans * trans = new PingRequestTrans(
        callback,
        param,
        pingTimeMs, 
        payloadBytes,
        payload
    );
    NetTransSend(trans);
}

//============================================================================
void NetCliGateKeeperFileSrvIpAddressRequest (
    FNetCliGateKeeperFileSrvIpAddressRequestCallback    callback,
    void *                                              param,
    bool                                                isPatcher
) {
    FileSrvIpAddressRequestTrans * trans = new FileSrvIpAddressRequestTrans(callback, param, isPatcher);
    NetTransSend(trans);
}

//============================================================================
void NetCliGateKeeperAuthSrvIpAddressRequest (
    FNetCliGateKeeperAuthSrvIpAddressRequestCallback    callback,
    void *                                              param
) {
    AuthSrvIpAddressRequestTrans * trans = new AuthSrvIpAddressRequestTrans(callback, param);
    NetTransSend(trans);
}
