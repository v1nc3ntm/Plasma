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

#include "plAvatar/plPhysicalControllerCore.h"

#include <BulletCollision/CollisionShapes/btCapsuleShape.h>

class btDiscreteDynamicsWorld;
class btCollisionObject;

class plBtPhysicalControllerCore : public plBtDefs::ObjectData, public plPhysicalControllerCore
{
public:
    
    ////////////////////////////////////////////
    // plBtDefs::ObjectData implementation
    
    virtual plKey               GetObjKey () const;
    virtual plSimDefs::plLOSDB  GetLOSDBs () const;
    virtual bool                IsSeeking () const;
    virtual void OnHit (const ObjectData &, const btVector3 & normal) const;
    
    ////////////////////////////////////////////
    // plPhysicalControllerCore implementation
    
    virtual void Enable (bool enable);
    
    virtual void SetSubworld (plKey world);
    
    virtual void GetState (hsPoint3 & pos, float & zRot);
    virtual void SetState (const hsPoint3 & pos, float zRot);
    
    virtual void SetMovementStrategy (plMovementStrategy * strategy);
    
    virtual void SetGlobalLoc (const hsMatrix44 & l2w);
    
    virtual void GetPositionSim (hsPoint3 & pos);
    
    virtual void Move (hsVector3 displacement, unsigned int collideWith, unsigned int & collisionResults);
    
    virtual void SetLinearVelocitySim (const hsVector3 & linearVel);
    
    virtual int SweepControllerPath (
        const hsPoint3 & startPos, const hsPoint3 & endPos, bool vsDynamics,
        bool vsStatics, uint32_t & vsSimGroups, std::vector<plControllerSweepRecord> & hits
    );
    
    virtual void LeaveAge ();
    
    ////////////////////////////////////////////
    // plSimulationMgrImpl interface
    
    void Apply(float deltaSec);
    void Update(float deltaSec, int nbSteps);
    
    //////////////////////////////////////
    
    class Private;
    
private:
    struct btControler;
    
    plBtPhysicalControllerCore (plKey ownerSO, float height, float radius, bool human);
    virtual ~plBtPhysicalControllerCore ();

    btControler * ctrl;
    btCapsuleShape shape;
    
    ////////////////////////////////////////////
    // plPhysicalControllerCore implementation
    
    virtual void IHandleEnableChanged ();

friend class plPhysicalControllerCore;
};
