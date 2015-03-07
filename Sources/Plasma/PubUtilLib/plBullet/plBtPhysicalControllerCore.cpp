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
#include "plAvatar/plAvDefs.h"
#include "plBtPhysicalControllerCore.h"
#include "plSimulationMgrImpl.h"
#include "pnSceneObject/plCoordinateInterface.h"
#include "pnSceneObject/plSceneObject.h"

#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletDynamics/Character/btKinematicCharacterController.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

namespace
{
    template<class T>
    struct AutoRef {
        
        template<class... Args>
        AutoRef (Args... args) : ptr(new T(args...)) {}
        
        AutoRef () = delete;
        AutoRef (const AutoRef<T> &) = delete;
        AutoRef (AutoRef<T> && o) : ptr(o.ptr) { o.ptr = nullptr; }
        ~AutoRef () { delete ptr; }
        
        operator       T & ()       { return *ptr; }
        operator const T & () const { return *ptr; }
        
    private:
        T * ptr;
    };
}

struct plBtPhysicalControllerCore::btControler {
    virtual btCollisionObject & GetObj () = 0;
    virtual void AddTo (btDiscreteDynamicsWorld & world) = 0;
    virtual void RemoveFrom (btDiscreteDynamicsWorld & world) = 0;
    virtual void SetEnable (bool) = 0;
    virtual void SetVelocity (const hsVector3 &) = 0;
    
    virtual ~btControler () = 0;
};


plBtPhysicalControllerCore::btControler::~btControler () {}

struct plBtPhysicalControllerCore::Private {
    static const int kAvatarMass = 200;
    static const int kZAxis = 2;
    static const int kUpAxis = kZAxis;
    
    static const unsigned short kKinematicMask =
        (1 << plSimDefs::kGroupAvatarKinematic)
      | (1 << plSimDefs::kGroupDynamic)
      | (1 << plSimDefs::kGroupDetector);
    static const unsigned short kAvatarMask =
        (1 << plSimDefs::kGroupStatic)
      | (1 << plSimDefs::kGroupAvatarBlocker)
      | (1 << plSimDefs::kGroupDynamic)
      | (1 << plSimDefs::kGroupDetector)
      | (1 << plSimDefs::kGroupExcludeRegion);
    
    struct btKinematicControler : btControler {
        btPairCachingGhostObject        ghost;
        btKinematicCharacterController  ctrl;
        
        btKinematicControler (plBtDefs::ObjectData * data, hsPoint3 & pos, btCapsuleShape & shape)
         : ctrl(&ghost, &shape, kSlopeLimit, kUpAxis) {
            ghost.setUserPointer(data);
            ghost.setCollisionShape(&shape);
        }
        
        virtual ~btKinematicControler () {}
        
        virtual btCollisionObject & GetObj () { return ghost; }
        
        virtual void AddTo(btDiscreteDynamicsWorld & world) {
            world.addCollisionObject(&ghost, (1 << plSimDefs::kGroupAvatarKinematic), kKinematicMask);
            world.addAction(&ctrl);
        }
        
        virtual void RemoveFrom (btDiscreteDynamicsWorld & world) {
            world.removeCollisionObject(&ghost);
            world.removeAction(&ctrl);
        }
        
        virtual void SetEnable (bool) {}
        virtual void SetVelocity (const hsVector3 &) {}
    };

    struct btDynamicControler : btControler {
        btRigidBody body;
        
        btDynamicControler (plBtDefs::ObjectData * data, hsPoint3 & pos, btCapsuleShape & shape)
         : body(createInfos(pos, shape))
        {
            body.setAngularFactor(0); // disable rotation
            body.setUserPointer (data);
        }
        
        virtual ~btDynamicControler () {}
        
        AutoRef<btRigidBody::btRigidBodyConstructionInfo> createInfos (hsPoint3 & pos, btCapsuleShape & shape) {
            AutoRef<btRigidBody::btRigidBodyConstructionInfo> result(kAvatarMass, nullptr, &shape);
            btRigidBody::btRigidBodyConstructionInfo & info = result;
            
            btVector3 & orig = info.m_startWorldTransform.getOrigin ();
            orig.setX(pos.fX);
            orig.setY(pos.fY);
            orig.setZ(pos.fZ);
            
            // make actor always 'active'
            info.m_linearSleepingThreshold = 0;
            info.m_angularSleepingThreshold = 0;
            
            return result;
        }
        
        virtual btCollisionObject & GetObj () { return body; }
        
        virtual void AddTo (btDiscreteDynamicsWorld & world) {
            world.addRigidBody (&body, (1 << plSimDefs::kGroupAvatarKinematic), kKinematicMask);
        }
        
        virtual void RemoveFrom (btDiscreteDynamicsWorld & world) {
            world.removeRigidBody (&body);
        }
        
        virtual void SetEnable (bool enable) {
            if (!body.getBroadphaseHandle())
                return; // not in a world
            if (enable) {
                body.getBroadphaseHandle()->m_collisionFilterGroup = (1 << plSimDefs::kGroupAvatar);
                body.getBroadphaseHandle()->m_collisionFilterMask  = kAvatarMask;
            } else {
                body.getBroadphaseHandle()->m_collisionFilterGroup = (1 << plSimDefs::kGroupAvatarKinematic);
                body.getBroadphaseHandle()->m_collisionFilterMask  = kKinematicMask;
            }
        }
        
        virtual void SetVelocity (const hsVector3 & vec) {
            body.setLinearVelocity(btVector3(vec.fX, vec.fY, vec.fZ));
        }
    };
};


plPhysicalControllerCore * plPhysicalControllerCore::Create (
    plKey ownerSO, float height, float width, bool human
) {
    float radius = width / 2.0f;
    float realHeight = height - width;
    return new plBtPhysicalControllerCore (ownerSO, realHeight, radius, human);
}

plBtPhysicalControllerCore::plBtPhysicalControllerCore (plKey ownerSO, float height, float radius, bool human)
 : plPhysicalControllerCore(ownerSO, height, radius),
   ctrl (nullptr),
   shape (radius, height)
{
    plSimulationMgrImpl::AddCtrl(fWorldKey, *this);
}

plBtPhysicalControllerCore::~plBtPhysicalControllerCore() {
    plSimulationMgrImpl::RemCtrl(fWorldKey, *this);
    if (ctrl) {
        ctrl->RemoveFrom(plSimulationMgrImpl::GetOrCreateWorld(fWorldKey));
        delete ctrl;
    }
}


plKey               plBtPhysicalControllerCore::GetObjKey () const { return fOwner; }
plSimDefs::plLOSDB  plBtPhysicalControllerCore::GetLOSDBs () const { return fLOSDB; }
bool                plBtPhysicalControllerCore::IsSeeking () const { return fSeeking; }

void plBtPhysicalControllerCore::OnHit (
    const ObjectData &,
    const btVector3 & normal
) const {
    hsVector3 n(normal.x(), normal.y(), normal.z());
    fMovementStrategy->AddContactNormals (n);
}

void plBtPhysicalControllerCore::Enable (bool enable) {
    if (fEnabled != enable)
    {
        fEnabled = enable;
        if (fEnabled)
            // Defer until the next physics update.
            fEnableChanged = true;
        else if (fMovementStrategy && !fMovementStrategy->IsKinematic())
        {
            ctrl->SetEnable (false);
        }
    }
}

void plBtPhysicalControllerCore::IHandleEnableChanged () {
    fEnableChanged = false;
    ctrl->SetEnable (true);
}


void plBtPhysicalControllerCore::SetSubworld (plKey world) {
    if (world != fWorldKey) {
        if (ctrl) {
            ctrl->RemoveFrom(plSimulationMgrImpl::GetOrCreateWorld(fWorldKey));
            ctrl->AddTo(plSimulationMgrImpl::GetOrCreateWorld(world));
        }
        
        plSimulationMgrImpl::RemCtrl(fWorldKey, *this);
        plSimulationMgrImpl::AddCtrl(world, *this);
        
        fWorldKey = world;
    }
}

void plBtPhysicalControllerCore::GetState (hsPoint3 & pos, float & zRot) {
    fLocalRotation.NormalizeIfNeeded();
    fLocalRotation.GetAngleAxis(&zRot, (hsVector3*)&pos);
    
    if (pos.fZ < 0)
        zRot = (2 * float(M_PI)) - zRot; // axis is backwards, so reverse the angle too

    pos = fLocalPosition;
}
void plBtPhysicalControllerCore::SetState (const hsPoint3 & pos, float zRot) {
    
}

void plBtPhysicalControllerCore::SetMovementStrategy (plMovementStrategy * strategy) {
    hsAssert(strategy, "null mouvement strategy!");
    
    if (!fMovementStrategy || fMovementStrategy->IsKinematic() != strategy->IsKinematic())
    {
        btDiscreteDynamicsWorld & world = plSimulationMgrImpl::GetOrCreateWorld(fWorldKey);
        
        if (ctrl) {
            ctrl->RemoveFrom(world);
            delete ctrl;
        }
        
        if (strategy->IsKinematic())
            ctrl = new Private::btKinematicControler(this, fLocalPosition, shape);
        else
            ctrl = new Private::btDynamicControler(this, fLocalPosition, shape);
        
        ctrl->AddTo(world);
    }
    
    fMovementStrategy = strategy;
}

void plBtPhysicalControllerCore::SetGlobalLoc (const hsMatrix44 & l2w) {
}

void plBtPhysicalControllerCore::GetPositionSim (hsPoint3 & pos) {
    auto & orig = ctrl->GetObj().getWorldTransform().getOrigin();
    pos.fX = orig.x();
    pos.fY = orig.y();
    pos.fZ = orig.z();
}

void plBtPhysicalControllerCore::Move (hsVector3 displacement, unsigned int collideWith, unsigned int & collisionResults) {
}

void plBtPhysicalControllerCore::SetLinearVelocitySim (const hsVector3 & vec) {
    ctrl->SetVelocity (vec);
}

int plBtPhysicalControllerCore::SweepControllerPath (
    const hsPoint3 & startPos, const hsPoint3 & endPos, bool vsDynamics,
    bool vsStatics, uint32_t & vsSimGroups, std::vector<plControllerSweepRecord> & hits
) {
	return 0;
}

void plBtPhysicalControllerCore::LeaveAge () { SetSubworld (nullptr); }


void plBtPhysicalControllerCore::Apply (float deltaSec) { IApply(deltaSec); }
void plBtPhysicalControllerCore::Update (float deltaSec, int nbSteps) { IUpdate(deltaSec, nbSteps, 0); } // alpha is the remain time: lest than 1/60s ??? useless?

