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
#include "plAvatar/plArmatureMod.h"
#include "plAvatar/plAvatarMgr.h"
#include "plAvatar/plPhysicalControllerCore.h"
#include "plLOSDispatch.h"
#include "plMessage/plLOSHitMsg.h"
#include "plMessage/plLOSRequestMsg.h"
#include "plProfile.h"
#include "plSimulationMgrImpl.h"

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <LinearMath/btVector3.h>

plProfile_CreateTimer("LineOfSight", "Simulation", LineOfSight);

struct plLOSDispatch::Private {
    
    static btDiscreteDynamicsWorld & GetWorld (const plLOSRequestMsg & msg) {
        if (msg.fWorldKey)
            return plSimulationMgrImpl::GetOrCreateWorld(msg.fWorldKey);
        
        plArmatureMod * av = plAvatarMgr::GetInstance()->GetLocalAvatar();
        if (av && av->GetController())
            return plSimulationMgrImpl::GetOrCreateWorld(av->GetController()->GetSubworld());
        
        return plSimulationMgrImpl::GetOrCreateWorld(nullptr);
    }
    
    
    static void DoRayCast (plKey myKey, const plLOSRequestMsg & req, btCollisionWorld::ClosestRayResultCallback & result) {
        btDiscreteDynamicsWorld & world = GetWorld(req);
        
        //result.m_collisionFilterMask  = req.fRequestType; //TODO
        //result.m_collisionFilterGroup = req.fRequestType; //TODO
        world.rayTest(result.m_rayFromWorld, result.m_rayToWorld, result); // TODO: ignore disabled clickable
        
        bool hasHit = result.hasHit();
        
        if (hasHit && req.fCullDB) {
            btCollisionWorld::ClosestRayResultCallback result2(
                result.m_rayFromWorld,
                result.m_hitPointWorld
            );
            //result2.m_collisionFilterMask  = req.fCullDB; //TODO
            //result2.m_collisionFilterGroup = req.fCullDB; //TODO
            world.rayTest(result.m_rayFromWorld, result.m_hitPointWorld, result2);
            
            hasHit = !result2.hasHit(); // TODO: ignore disabled clickable
        }
        
        bool doReport = true;
        switch (req.fReportType)
        {
        case plLOSRequestMsg::kReportHit:       doReport =  hasHit; break;
        case plLOSRequestMsg::kReportMiss:      doReport = !hasHit; break;
        case plLOSRequestMsg::kReportHitOrMiss: doReport =    true; break;
        }
        
        if (doReport)
        {
            plLOSHitMsg * hitMsg = new plLOSHitMsg(myKey, req.GetSender(), nullptr);
            hitMsg->fNoHit = !hasHit;
            //hitMsg->fObj = result.m_collisionObject.GetUserPointer(); // TODO
            hitMsg->fDistance = result.m_rayFromWorld.distance(result.m_hitPointWorld);
            hitMsg->fNormal.fX = result.m_hitNormalWorld.x();
            hitMsg->fNormal.fY = result.m_hitNormalWorld.y();
            hitMsg->fNormal.fZ = result.m_hitNormalWorld.z();
            hitMsg->fHitPoint.fX = result.m_hitPointWorld.x();
            hitMsg->fHitPoint.fY = result.m_hitPointWorld.y();
            hitMsg->fHitPoint.fZ = result.m_hitPointWorld.z();
            hitMsg->fRequestID = req.fRequestID;
            
            hitMsg->Send();
        }
    }
    
    
    struct FirstRayResultCallback : btCollisionWorld::ClosestRayResultCallback {
        
        FirstRayResultCallback(const btVector3 & from, const btVector3 & to)
         : btCollisionWorld::ClosestRayResultCallback(from, to)
        {}
        
        virtual btScalar addSingleResult (
            btCollisionWorld::LocalRayResult &rayResult,
            bool normalInWorldSpace
        ) {
            btCollisionWorld::ClosestRayResultCallback::addSingleResult(
                rayResult,
                normalInWorldSpace
            );
            return 0.f; // TODO: test if it works...
        }
    };
    
};

plLOSDispatch::plLOSDispatch () {}
plLOSDispatch::~plLOSDispatch () {}

bool plLOSDispatch::MsgReceive (plMessage * msg) {
    plLOSRequestMsg * requestMsg = plLOSRequestMsg::ConvertNoRef(msg);
    
    if (!requestMsg)
        return hsKeyedObject::MsgReceive(msg);
    
    plProfile_BeginTiming(LineOfSight);
    
    const btVector3 from(requestMsg->fFrom.fX, requestMsg->fFrom.fY, requestMsg->fFrom.fZ);
    const btVector3 to(  requestMsg->fTo  .fX, requestMsg->fTo  .fY, requestMsg->fTo  .fZ);
    
    switch (requestMsg->fTestType) {
    case plLOSRequestMsg::kTestClosest: {
        btCollisionWorld::ClosestRayResultCallback result(from, to);
        Private::DoRayCast(GetKey(), *requestMsg, result);
        break; }
    case plLOSRequestMsg::kTestAny: {
        Private::FirstRayResultCallback result(from, to);
        Private::DoRayCast(GetKey(), *requestMsg, result);
        break; }
    }
    
    
    plProfile_EndTiming(LineOfSight);
}



