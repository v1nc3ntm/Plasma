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
*   $/Plasma20/Sources/Plasma/PubUtilLib/plNetGameLib/Private/plNglGame.cpp
*   
***/

#include "../Pch.h"
#include "pnAsyncCore/pnAcTimer.h"
#pragma hdrstop

namespace Ngl { namespace Game {
/*****************************************************************************
*
*   Private
*
***/

struct CliGmConn : hsRefCnt {
    LINK(CliGmConn) link;

    std::mutex          critsect;
    AsyncSocket *       sock;
    AsyncSocket::Cancel cancelId;
    NetCli *            cli;
    plNetAddress        addr;
    unsigned            seq;
    bool                abandoned;

    // ping
    AsyncTimer      pingTimer;
    unsigned        pingSendTimeMs;
    unsigned        lastHeardTimeMs;

    CliGmConn ();
    ~CliGmConn ();

    // ping
    void AutoPing ();
    void StopAutoPing ();
    void TimerPing ();

    void Send (const uintptr_t fields[], unsigned count);
};


//============================================================================
// JoinAgeRequestTrans
//============================================================================
struct JoinAgeRequestTrans : NetGameTrans {
    FNetCliGameJoinAgeRequestCallback   m_callback;
    void *                              m_param;
    // sent
    unsigned                            m_ageMcpId;
    plUUID                              m_accountUuid;
    unsigned                            m_playerInt;

    JoinAgeRequestTrans (
        unsigned                            ageMcpId,
        const plUUID&                       accountUuid,
        unsigned                            playerInt,
        FNetCliGameJoinAgeRequestCallback   callback,
        void *                              param
    );

    bool Send ();
    void Post ();
    bool Recv (
        const uint8_t  msg[],
        unsigned    bytes
    );
};

//============================================================================
// RcvdPropagatedBufferTrans
//============================================================================
struct RcvdPropagatedBufferTrans : NetNotifyTrans {

    unsigned        bufferType;
    unsigned        bufferBytes;
    uint8_t *          bufferData;

    RcvdPropagatedBufferTrans () : NetNotifyTrans(kGmRcvdPropagatedBufferTrans) {}
    ~RcvdPropagatedBufferTrans ();
    void Post ();
};

//============================================================================
// RcvdGameMgrMsgTrans
//============================================================================
struct RcvdGameMgrMsgTrans : NetNotifyTrans {

    unsigned        bufferBytes;
    uint8_t *          bufferData;

    RcvdGameMgrMsgTrans () : NetNotifyTrans(kGmRcvdGameMgrMsgTrans) {}
    ~RcvdGameMgrMsgTrans ();
    void Post ();
};


/*****************************************************************************
*
*   Private data
*
***/

enum {
    kPerfConnCount,
    kPingDisabled,
    kNumPerf
};

static bool                             s_running;
static std::mutex                       s_critsect;
static LISTDECL(CliGmConn, link)        s_conns;
static CliGmConn *                      s_active;
static FNetCliGameRecvBufferHandler     s_bufHandler;
static FNetCliGameRecvGameMgrMsgHandler s_gameMgrMsgHandler;
static std::atomic<long>                s_perf[kNumPerf];


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
static CliGmConn * GetConnIncRef_CS (const char tag[]) {
    if (CliGmConn * conn = s_active) {
        if (conn->cli) {
            conn->Ref(tag);
            return conn;
        }
    }
    return nil;
}

//============================================================================
static CliGmConn * GetConnIncRef (const char tag[]) {
    std::lock_guard<std::mutex> lock(s_critsect);
    return GetConnIncRef_CS(tag);
}

//============================================================================
static void UnlinkAndAbandonConn_CS (CliGmConn * conn) {
    s_conns.Unlink(conn);
    conn->abandoned = true;
    if (conn->cancelId)
        conn->cancelId.ConnectCancel();
    else if (conn->sock)
        conn->sock->Disconnect(true);
    else
        conn->UnRef("Lifetime");
}

//============================================================================
static bool ConnEncrypt (ENetError error, void * param) {
    CliGmConn * conn = (CliGmConn *) param;
    if (!s_perf[kPingDisabled])
        conn->AutoPing();

    if (IS_NET_SUCCESS(error)) {
        std::lock_guard<std::mutex> lock(s_critsect);
        std::swap(s_active, conn);
    }

    return IS_NET_SUCCESS(error);
}

//============================================================================
static void NotifyConnSocketConnect (CliGmConn * conn) {

    conn->TransferRef("Connecting", "Connected");
    conn->cli = NetCliConnectAccept(
        conn->sock,
        kNetProtocolCli2Game,
        true,
        ConnEncrypt,
        0,
        nil,
        conn
    );
}

//============================================================================
static void NotifyConnSocketConnectFailed (CliGmConn * conn) {
    bool notify;
    {
        std::lock_guard<std::mutex> lock(s_critsect);

        conn->cancelId.Clear();
        s_conns.Unlink(conn);
        
        notify
            =  s_running
            && !conn->abandoned
            && (!s_active || conn == s_active);
            
        if (conn == s_active)
            s_active = nil;
    }

    NetTransCancelByConnId(conn->seq, kNetErrTimeout);
    conn->UnRef("Connecting");
    conn->UnRef("Lifetime");

    if (notify)
        ReportNetError(kNetProtocolCli2Game, kNetErrConnectFailed);
}

//============================================================================
static void NotifyConnSocketDisconnect (CliGmConn * conn) {
    conn->StopAutoPing();

    bool notify;
    {
        std::lock_guard<std::mutex> lock(s_critsect);

        s_conns.Unlink(conn);

        notify
            =  s_running
            && !conn->abandoned
            && (!s_active || conn == s_active);

        if (conn == s_active)
            s_active = nil;
    }

    // Cancel all transactions in process on this connection.
    NetTransCancelByConnId(conn->seq, kNetErrTimeout);
    conn->UnRef("Connected");
    conn->UnRef("Lifetime");

    if (notify)
        ReportNetError(kNetProtocolCli2Game, kNetErrDisconnected);
}

//============================================================================
static bool NotifyConnSocketRead (CliGmConn * conn, AsyncSocket::NotifyRead * read) {
    // TODO: Only dispatch messages from the active game server
    conn->lastHeardTimeMs = GetNonZeroTimeMs();
    bool result = NetCliDispatch(conn->cli, read->buffer, read->bytes, nil);
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
    CliGmConn * conn;

    switch (code) {
        case AsyncSocket::kNotifyConnectSuccess:
            conn = (CliGmConn *) notify->param;
            sock->user = conn;
            conn->TransferRef("Connecting", "Connected");
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
            conn = (CliGmConn *) notify->param;
            NotifyConnSocketConnectFailed(conn);
        break;

        case AsyncSocket::kNotifyDisconnect:
            conn = (CliGmConn *) sock->user;
            NotifyConnSocketDisconnect(conn);
        break;

        case AsyncSocket::kNotifyRead:
            conn = (CliGmConn *) sock->user;
            result = NotifyConnSocketRead(conn, (AsyncSocket::NotifyRead *) notify);
        break;
    }
    
    return result;
}

//============================================================================
static void Connect (
    const plNetAddress& addr
) {
    CliGmConn * conn = new CliGmConn;
    conn->addr              = addr;
    conn->seq               = ConnNextSequence();
    conn->lastHeardTimeMs   = GetNonZeroTimeMs();

    conn->Ref("Lifetime");
    conn->Ref("Connecting");

    {
        std::lock_guard<std::mutex> lock(s_critsect);
        while (CliGmConn * conn = s_conns.Head())
            UnlinkAndAbandonConn_CS(conn);
        s_conns.Link(conn);
    }

    Cli2Game_Connect connect;
    connect.hdr.connType    = kConnTypeCliToGame;
    connect.hdr.hdrBytes    = sizeof(connect.hdr);
    connect.hdr.buildId     = plProduct::BuildId();
    connect.hdr.buildType   = plProduct::BuildType();
    connect.hdr.branchId    = plProduct::BranchId();
    connect.hdr.productId   = plProduct::UUID();
    connect.data.dataBytes  = sizeof(connect.data);

    conn->cancelId = AsyncSocket::Connect(
        addr,
        SocketNotifyCallback,
        conn,
        &connect,
        sizeof(connect),
        0,
        0
    );
}


/*****************************************************************************
*
*   CliGmConn
*
***/

//===========================================================================
static unsigned CliGmConnTimerDestroyed (void * param) {
    CliGmConn * conn = (CliGmConn *) param;
    conn->UnRef("TimerDestroyed");
    return kPosInfinity32;
}

//===========================================================================
static unsigned CliGmConnPingTimerProc (void * param) {
    ((CliGmConn *) param)->TimerPing();
    return kPingIntervalMs;
}

//============================================================================
CliGmConn::CliGmConn ()
    : hsRefCnt(0), sock(nil), cli(nil)
    , seq(0), abandoned(false)
    , pingSendTimeMs(0), lastHeardTimeMs(0)
{
    ++s_perf[kPerfConnCount];
}

//============================================================================
CliGmConn::~CliGmConn () {
    if (cli)
        NetCliDelete(cli, true);
    --s_perf[kPerfConnCount];
}

//============================================================================
void CliGmConn::AutoPing () {
    ASSERT(!pingTimer);
    Ref("PingTimer");

    std::lock_guard<std::mutex> lock(critsect);

    pingTimer.Create(
        CliGmConnPingTimerProc,
        sock ? 0 : kPosInfinity32,
        this
    );
}

//============================================================================
void CliGmConn::StopAutoPing () {
    std::lock_guard<std::mutex> lock(critsect);

    if (pingTimer)
        pingTimer.Delete(CliGmConnTimerDestroyed);
}

//============================================================================
void CliGmConn::TimerPing () {
    // Send a ping request
    pingSendTimeMs = GetNonZeroTimeMs();

    const uintptr_t msg[] = {
        kCli2Game_PingRequest,
        pingSendTimeMs
    };

    Send(msg, arrsize(msg));
}

//============================================================================
void CliGmConn::Send (const uintptr_t fields[], unsigned count) {
    std::lock_guard<std::mutex> lock(critsect);

    NetCliSend(cli, fields, count);
    NetCliFlush(cli);
}


/*****************************************************************************
*
*   Cli2Game protocol
*
***/

//============================================================================
static bool Recv_PingReply (
    const uint8_t      msg[],
    unsigned        bytes,
    void *          param
) {
    return true;
}

//============================================================================
static bool Recv_JoinAgeReply (
    const uint8_t      msg[],
    unsigned        bytes,
    void *          param
) {
    const Game2Cli_JoinAgeReply & reply = *(const Game2Cli_JoinAgeReply *)msg;
    if (sizeof(reply) != bytes)
        return false;

    NetTransRecv(reply.transId, msg, bytes);

    return true;
}

//============================================================================
static bool Recv_PropagateBuffer (
    const uint8_t      msg[],
    unsigned        bytes,
    void *          param
) {
    const Game2Cli_PropagateBuffer & reply = *(const Game2Cli_PropagateBuffer *)msg;

    RcvdPropagatedBufferTrans * trans = new RcvdPropagatedBufferTrans;
    trans->bufferType   = reply.type;
    trans->bufferBytes  = reply.bytes;
    trans->bufferData   = (uint8_t *)malloc(reply.bytes);
    memcpy(trans->bufferData, reply.buffer, reply.bytes);
    NetTransSend(trans);

    return true;
}

//============================================================================
static bool Recv_GameMgrMsg (
    const uint8_t      msg[],
    unsigned        bytes,
    void *          param
) {
    const Game2Cli_GameMgrMsg & reply = *(const Game2Cli_GameMgrMsg *)msg;

    RcvdGameMgrMsgTrans * trans = new RcvdGameMgrMsgTrans;
    trans->bufferBytes  = reply.bytes;
    trans->bufferData   = (uint8_t *)malloc(reply.bytes);
    memcpy(trans->bufferData, reply.buffer, reply.bytes);
    NetTransSend(trans);

    return true;
}


//============================================================================
// Send/Recv protocol handler init
//============================================================================
#define MSG(s)  kNetMsg_Cli2Game_##s
static NetMsgInitSend s_send[] = {
    { MSG(PingRequest),         },
    { MSG(JoinAgeRequest),      },
    { MSG(PropagateBuffer),     },
    { MSG(GameMgrMsg),          },
};
#undef MSG

#define MSG(s)  kNetMsg_Game2Cli_##s, Recv_##s
static NetMsgInitRecv s_recv[] = {
    { MSG(PingReply)            },
    { MSG(JoinAgeReply),        },
    { MSG(PropagateBuffer),     },
    { MSG(GameMgrMsg),          },
};
#undef MSG


/*****************************************************************************
*
*   JoinAgeRequestTrans
*
***/

//============================================================================
JoinAgeRequestTrans::JoinAgeRequestTrans (
    unsigned                            ageMcpId,
    const plUUID&                       accountUuid,
    unsigned                            playerInt,
    FNetCliGameJoinAgeRequestCallback   callback,
    void *                              param
) : NetGameTrans(kJoinAgeRequestTrans)
,   m_callback(callback)
,   m_param(param)
,   m_ageMcpId(ageMcpId)
,   m_accountUuid(accountUuid)
,   m_playerInt(playerInt)
{
}

//============================================================================
bool JoinAgeRequestTrans::Send () {
    if (!AcquireConn())
        return false;

    const uintptr_t msg[] = {
        kCli2Game_JoinAgeRequest,
                        m_transId,
                        m_ageMcpId,
        (uintptr_t) &m_accountUuid,
                        m_playerInt,
    };

    m_conn->Send(msg, arrsize(msg));
    
    return true;
}

//============================================================================
void JoinAgeRequestTrans::Post () {
    m_callback(
        m_result,
        m_param
    );
}

//============================================================================
bool JoinAgeRequestTrans::Recv (
    const uint8_t  msg[],
    unsigned    bytes
) {
    const Game2Cli_JoinAgeReply & reply = *(const Game2Cli_JoinAgeReply *) msg;
    m_result        = reply.result;
    m_state         = kTransStateComplete;
    return true;
}

/*****************************************************************************
*
*   RcvdPropagatedBufferTrans
*
***/

//============================================================================
RcvdPropagatedBufferTrans::~RcvdPropagatedBufferTrans () {
    free(bufferData);
}

//============================================================================
void RcvdPropagatedBufferTrans::Post () {
    if (s_bufHandler)
        s_bufHandler(bufferType, bufferBytes, bufferData);
}

/*****************************************************************************
*
*   RcvdGameMgrMsgTrans
*
***/

//============================================================================
RcvdGameMgrMsgTrans::~RcvdGameMgrMsgTrans () {
    free(bufferData);
}

//============================================================================
void RcvdGameMgrMsgTrans::Post () {
    if (s_gameMgrMsgHandler)
        s_gameMgrMsgHandler((GameMsgHeader *)bufferData);
}


} using namespace Game;


/*****************************************************************************
*
*   NetGameTrans
*
***/

//============================================================================
NetGameTrans::NetGameTrans (ETransType transType)
:   NetTrans(kNetProtocolCli2Game, transType)
,   m_conn(nil)
{
}

//============================================================================
NetGameTrans::~NetGameTrans () {
    ReleaseConn();
}

//============================================================================
bool NetGameTrans::AcquireConn () {
    if (!m_conn)
        m_conn = GetConnIncRef("AcquireConn");
    return m_conn != nil;
}

//============================================================================
void NetGameTrans::ReleaseConn () {
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
void GameInitialize () {
    s_running = true;
    NetMsgProtocolRegister(
        kNetProtocolCli2Game,
        false,
        s_send, arrsize(s_send),
        s_recv, arrsize(s_recv),
        kGameDhGValue,
        plBigNum(sizeof(kGameDhXData), kGameDhXData),
        plBigNum(sizeof(kGameDhNData), kGameDhNData)
    );
}

//============================================================================
void GameDestroy (bool wait) {
    s_running = false;
    s_bufHandler = nil;
    s_gameMgrMsgHandler = nil;

    NetTransCancelByProtocol(
        kNetProtocolCli2Game,
        kNetErrRemoteShutdown
    );    
    NetMsgProtocolDestroy(
        kNetProtocolCli2Game,
        false
    );
    
    {
        std::lock_guard<std::mutex> lock(s_critsect);
        while (CliGmConn * conn = s_conns.Head())
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
bool GameQueryConnected () {
    std::lock_guard<std::mutex> lock(s_critsect);
    return (s_active && s_active->cli);
}

//============================================================================
unsigned GameGetConnId () {
    std::lock_guard<std::mutex> lock(s_critsect);
    return (s_active) ? s_active->seq : 0;
}

//============================================================================
void GamePingEnable (bool enable) {
    s_perf[kPingDisabled] = !enable;

    std::lock_guard<std::mutex> lock(s_critsect);
    for (;;) {
        if (!s_active)
            break;
        if (enable)
            s_active->AutoPing();
        else
            s_active->StopAutoPing();
        break;
    }
}

} using namespace Ngl;


/*****************************************************************************
*
*   Exported functions
*
***/

//============================================================================
void NetCliGameStartConnect (
    const uint32_t node
) {
    plNetAddress addr(node, kNetDefaultClientPort);
    Connect(addr);
}

//============================================================================
void NetCliGameDisconnect () {
    std::lock_guard<std::mutex> lock(s_critsect);

    while (CliGmConn * conn = s_conns.Head())
        UnlinkAndAbandonConn_CS(conn);
    s_active = nil;
}

//============================================================================
void NetCliGameJoinAgeRequest (
    unsigned                            ageMcpId,
    const plUUID&                       accountUuid,
    unsigned                            playerInt,
    FNetCliGameJoinAgeRequestCallback   callback,
    void *                              param
) {
    JoinAgeRequestTrans * trans = new JoinAgeRequestTrans(
        ageMcpId,
        accountUuid,
        playerInt,
        callback,
        param
    );
    NetTransSend(trans);
}

//============================================================================
void NetCliGameSetRecvBufferHandler (
    FNetCliGameRecvBufferHandler    handler
) {
    s_bufHandler = handler;
}

//============================================================================
void NetCliGamePropagateBuffer (
    unsigned                        type,
    unsigned                        bytes,
    const uint8_t                      buffer[]
) {
    CliGmConn * conn = GetConnIncRef("PropBuffer");
    if (!conn)
        return;

    const uintptr_t msg[] = {
        kCli2Game_PropagateBuffer,
        type,
        bytes,
        (uintptr_t) buffer,
    };

    conn->Send(msg, arrsize(msg));

    conn->UnRef("PropBuffer");
}

//============================================================================
void NetCliGameSetRecvGameMgrMsgHandler (FNetCliGameRecvGameMgrMsgHandler handler) {
    s_gameMgrMsgHandler = handler;
}

//============================================================================
void NetCliGameSendGameMgrMsg (GameMsgHeader * msgHdr) {
    CliGmConn * conn = GetConnIncRef("GameMgrMsg");
    if (!conn)
        return;

    const uintptr_t msg[] = {
        kCli2Game_GameMgrMsg,
                        msgHdr->messageBytes,
        (uintptr_t)  msgHdr,
    };
    
    conn->Send(msg, arrsize(msg));
    
    conn->UnRef("GameMgrMsg");
}
