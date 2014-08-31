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
#include "plSimulationMgr.h"

#include <btBulletDynamicsCommon.h>
#include <map>

using namespace std;

class plSimulationMgr::Private : plSimulationMgr
{
public:
    static Private * instance;
    
    bool    suspend;
    
    btDefaultCollisionConfiguration     collisionConf
    btCollisionDispatcher               dispatcher;
    btBroadphaseInterface               pairCache;
    btDbvtBroadphase                    constraintSolver;
    btSequentialImpulseConstraintSolver solver;
    
    map<plKey, btDiscreteDynamicsWorld*> scenes;
    
    
    Private  ();
    ~Private ();
};
plSimulationMgr::Private * plSimulationMgr::Private::instance = nullptr;


plSimulationMgr & plSimulationMgr::GetInstance () { return *Private::instance; }

void plSimulationMgr::Init ()
{
    hsAssert(!Private::instance, "Initializing the sim when it's already been done");
    if (!Private::instance)
        Private::instance = new Private();
}

void plSimulationMgr::Shutdown ()
{
    hsAssert(Private::instance, "Shutdown the sim without initializing them");
    
    if (Private::instance)
    {
        hsAssert(Private::instance->scenes.empty(), "Unreleased scenes at shutdown");
        delete Private::instance;
        Private::instance = nullptr;
    }
}

plSimulationMgr::Private::Private ()
  : suspend(true)
    collisionConf(),
    dispatcher(&collisionConf),
    pairCache(),
    constraintSolver()
{
}


btDiscreteDynamicsWorld & plSimulationMgr::GetScene (plKey id)
{
    btDiscreteDynamicsWorld * scene = Private::instance->scenes[id];
    
    if (!scene)
    {
        scene = new btDiscreteDynamicsWorld(&dispatcher, &pairCache, &constraintSolver, &collisionConf);
        scene->setGravity(btVector3(0, 0, -32.174049f));
    }
    
    return *scene;
}

void plSimulationMgr::ReleaseScene (plKey id)
{
    auto it = Private::instance->scenes.find(id);
    hsAssert(it != Private::instance->scenes.end(), "Release unknow scene")
    
    delete *it.second;
    Private::instance->scenes.erase(it);
}

void plSimulationMgr::Advance (float deltaSecs)
{
    if (Private::instance->suspend)
        return;
    
    for (auto scene: Private::instance->scenes)
    {
        scene->stepSimulation(deltaSecs); //, 60/minFps);
    }
}
void plSimulationMgr::Suspend     () {        Private::instance->suspend = true; }
void plSimulationMgr::Resume      () {        Private::instance->suspend = false; }
bool plSimulationMgr::IsSuspended () { return Private::instance->suspend; }


