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
#include "plBtDefs.h"
#include "plPhysical/plSimulationMgr.h"
#include "plSimulationMgrImpl.h"

#include "plAvatar/plPhysicalControllerCore.h"

#include <map>

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

using namespace std;

class plSimulationMgrImpl::Private : plSimulationMgrImpl {
public:
    static Private * instance;
    
    bool suspend;
    map<plKey, btDiscreteDynamicsWorld> worlds;
    
    Private () : suspend(true) {}
    
    static inline bool IsSeeking (btBroadphaseProxy * proxy) {
        if (!proxy->m_clientObject) {
            FATAL("btBroadphaseProxy without client object!");
            return false;
        }
        plBtDefs::ObjectData * data = (plBtDefs::ObjectData*)((btCollisionObject*)proxy->m_clientObject)->getUserPointer();
        
        if (!data) {
            FATAL("btCollisionObject in avatar group without ptr to user-data");
            return false;
        }
        
        return data->IsSeeking ();
    }
    
    struct FilterCallback : public btOverlapFilterCallback {
        virtual bool needBroadphaseCollision (btBroadphaseProxy * proxy0, btBroadphaseProxy * proxy1) const {
            // group flags check
            if (!(proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask
               && proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask))
                return false;
            
            // seek mode avatar check
            if (proxy0->m_collisionFilterGroup == btBroadphaseProxy::CharacterFilter
             && proxy1->m_collisionFilterGroup == plBtDefs::kGrpExclude)
                return !IsSeeking(proxy0);
            if (proxy1->m_collisionFilterGroup == btBroadphaseProxy::CharacterFilter
             && proxy0->m_collisionFilterGroup == plBtDefs::kGrpExclude)
                return !IsSeeking(proxy1);
            
            return true;
        }
    };
};

plSimulationMgrImpl::Private * plSimulationMgrImpl::Private::instance = nullptr;

bool plSimulationMgr::Init () {
    hsAssert(!plSimulationMgrImpl::Private::instance, "Initializing the sim when it's already been done");
    if (!plSimulationMgrImpl::Private::instance)
        plSimulationMgrImpl::Private::instance = new plSimulationMgrImpl::Private();
    
    //btGImpactCollisionAlgorithm::registerAlgorithm(&dispatcher);
    
    return true;
}
void plSimulationMgr::Shutdown () {
    hsAssert(plSimulationMgrImpl::Private::instance, "Shutdown the sim without initializing them");
    if (plSimulationMgrImpl::Private::instance) {
        delete plSimulationMgrImpl::Private::instance;
        plSimulationMgrImpl::Private::instance = nullptr;
    }
}

void plSimulationMgr::Advance (float deltaSecs) {
    if (plSimulationMgrImpl::Private::instance->suspend)
        return;
    
    for (auto & it: plSimulationMgrImpl::Private::instance->worlds)
        it.second.stepSimulation(deltaSecs);  //, 60/minFps);
}

void plSimulationMgr::    Suspend () {        plSimulationMgrImpl::Private::instance->suspend = true ; }
void plSimulationMgr::     Resume () {        plSimulationMgrImpl::Private::instance->suspend = false; }
bool plSimulationMgr::IsSuspended () { return plSimulationMgrImpl::Private::instance->suspend;         }

void plSimulationMgr::Log (const char* formatStr, ...) {}
void plSimulationMgr::LogV (const char* formatStr, va_list args) {}
void plSimulationMgr::ClearLog () {}

uint32_t plSimulationMgr::GetStepCount () { return 0; }

////////////////////////////////////////////////////////////////////////

plSimulationMgrImpl::plSimulationMgrImpl () {}

btDiscreteDynamicsWorld & plSimulationMgrImpl::GetOrCreateWorld (plKey worldKey) {
    auto it = Private::instance->worlds.find(worldKey);
    
    if (it != Private::instance->worlds.end())
        return it->second;
    
    // TODO: worlds creator parameters can be shared or not? test shared for now!
    // NOTE: configuration and dispatcher must be OK to be shared!
    
    static btDefaultCollisionConfiguration      configuration;
    static btCollisionDispatcher                dispatcher(&configuration);
    static btDbvtBroadphase                     broadphase;
    static btSequentialImpulseConstraintSolver  solver;
    static Private::FilterCallback              filter;
    
    it = Private::instance->worlds.emplace_hint(
        it,
        piecewise_construct_t(),
        tuple<plKey>(worldKey),
        tuple<btDispatcher *, btBroadphaseInterface *, btConstraintSolver *, btCollisionConfiguration *>(
            &dispatcher, &broadphase, &solver, &configuration
        )
    );
    
    it->second.setGravity(btVector3(0, 0, -32.174049f));
    it->second.getPairCache()->setOverlapFilterCallback(&filter);
    
    return it->second;
}
