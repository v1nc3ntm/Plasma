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

#include <windows.h>
#include <cstdarg>

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
    
    
    void dprintf (const char * fmt, ...) {
        static char buffer[1024];
        va_list args;
        va_start(args, fmt);
        int n = vsnprintf (buffer, sizeof(buffer)-1, fmt, args);
        va_end(args);
        if (n >= 0) {
            if (n >= sizeof(buffer)-1)
                n = sizeof(buffer)-2;
            buffer[n+0] = '\n';
            buffer[n+1] = '\0';
        }
        
        OutputDebugString (buffer);
    }
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
            dprintf ("btKinematicControler (0x%X)", this);
            ghost.setUserPointer(data);
            ghost.setCollisionShape(&shape);
        }
        
        virtual ~btKinematicControler () {
            dprintf ("~btKinematicControler (0x%X)", this);
        }
        
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
            dprintf ("btDynamicControler (0x%X)", this);
            body.setAngularFactor(0); // disable rotation
            body.setUserPointer (data);
        }
        
        virtual ~btDynamicControler () {
            dprintf ("~btDynamicControler (0x%X)", this);
        }
        
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
            body.setGravity(btVector3(0, 0, 0));
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
            body.setLinearVelocity(btVector3(vec.fX, vec.fY,0 * vec.fZ));
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
 : plPhysicalControllerCore(ownerSO, height, radius)
 , ctrl (nullptr)
 , shape (radius, height)
{
    dprintf ("plBtPhysicalControllerCore (0x%X)", this);
    plSimulationMgrImpl::AddCtrl(fWorldKey, *this);
}

plBtPhysicalControllerCore::~plBtPhysicalControllerCore() {
    dprintf ("~plBtPhysicalControllerCore (0x%X, ctrl=0x%X, world=0x%X)", this, ctrl, (void*)fWorldKey);
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
    dprintf ("plBtPhysicalControllerCore::OnHit (0x%X, x=%i, y=%i, z=%i)", this, normal.x(), normal.y(), normal.z());
    hsVector3 n(normal.x(), normal.y(), normal.z());
    fMovementStrategy->AddContactNormals (n);
}

void plBtPhysicalControllerCore::Enable (bool enable) {
    if (fEnabled != enable)
    {
        dprintf ("plBtPhysicalControllerCore::enable (0x%X, enable=%i)", this, enable);
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
    dprintf ("plBtPhysicalControllerCore::IHandleEnableChanged (0x%X)", this);
    fEnableChanged = false;
    ctrl->SetEnable (true);
}


void plBtPhysicalControllerCore::SetSubworld (plKey world) {
    dprintf ("plBtPhysicalControllerCore::SetSubworld (0x%X, world=0x%X, fWorldKey=0x%X)", this, (void*)world, (void*)fWorldKey);
    if (world != fWorldKey) {
        if (ctrl) {
            if (fWorldKey)
                ctrl->RemoveFrom(plSimulationMgrImpl::GetOrCreateWorld(fWorldKey));
            if (world)
                ctrl->AddTo(plSimulationMgrImpl::GetOrCreateWorld(world));
        }
        
        if (fWorldKey)
            plSimulationMgrImpl::RemCtrl(fWorldKey, *this);
        if (world)
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
    dprintf ("plBtPhysicalControllerCore::GetState (0x%X, x=%i, y=%i, z=%i, r=%i))", this, pos.fX, pos.fY, pos.fZ, zRot);
}
void plBtPhysicalControllerCore::SetState (const hsPoint3 & pos, float zRot) {
    dprintf ("plBtPhysicalControllerCore::SetState (0x%X, x=%i, y=%i, z=%i, r=%i))", this, pos.fX, pos.fY, pos.fZ, zRot);
    plSceneObject* so = plSceneObject::ConvertNoRef(fOwner->ObjectIsLoaded());
    if (so) {
        hsQuat worldRot;
        hsVector3 zAxis(kAvatarUp.fX, kAvatarUp.fY, kAvatarUp.fZ);
        worldRot.SetAngleAxis(zRot, zAxis);

        hsMatrix44 l2w, w2l;
        worldRot.MakeMatrix(&l2w);
        l2w.SetTranslate(&pos);

        // Localize new position and rotation to global coords if we're in a subworld
        const plCoordinateInterface* ci = GetSubworldCI();
        if (ci)
        {
            const hsMatrix44& subworldL2W = ci->GetLocalToWorld();
            l2w = subworldL2W * l2w;
        }
        l2w.GetInverse(&w2l);
        so->SetTransform(l2w, w2l);
        so->FlushTransform();
    }
}

void plBtPhysicalControllerCore::SetMovementStrategy (plMovementStrategy * strategy) {
    dprintf ("plBtPhysicalControllerCore::SetMovementStrategy (0x%X, strategy=0x%X))", this, strategy);
    hsAssert(strategy, "null mouvement strategy!");
    
    if (!fMovementStrategy || fMovementStrategy->IsKinematic() != strategy->IsKinematic())
    {
        btDiscreteDynamicsWorld * world;

        if (fWorldKey)
            world = &plSimulationMgrImpl::GetOrCreateWorld(fWorldKey);
        
        if (ctrl) {
            if (fWorldKey)
                ctrl->RemoveFrom(*world);
            delete ctrl;
        }
        
        //if (strategy->IsKinematic())
        //    ctrl = new Private::btKinematicControler(this, fLocalPosition, shape);
        //else
            ctrl = new Private::btDynamicControler(this, fLocalPosition, shape);
        
        if (fWorldKey)
            ctrl->AddTo(*world);
    }
    
    fMovementStrategy = strategy;
}

void plBtPhysicalControllerCore::SetGlobalLoc (const hsMatrix44 & l2w) {
    fLastGlobalLoc = l2w;

    // Update our local position and rotation
    hsPoint3 prevPosition = fLocalPosition;
    const plCoordinateInterface* subworldCI = GetSubworldCI();
    if (subworldCI)
    {
        hsMatrix44 l2s = fPrevSubworldW2L * l2w;

        l2s.GetTranslate(&fLocalPosition);
        fLocalRotation.SetFromMatrix44(l2s);
    }
    else
    {
        l2w.GetTranslate(&fLocalPosition);
        fLocalRotation.SetFromMatrix44(l2w);
    }

    fLastLocalPosition = fLocalPosition;
    dprintf ("plBtPhysicalControllerCore::SetGlobalLoc (0x%X, x=%i, y=%i, z=%i))", this, fLastLocalPosition.fX, fLastLocalPosition.fY, fLastLocalPosition.fZ);

    hsMatrix44 w2l;
    l2w.GetInverse(&w2l);

    // TODO: Update the physical position
    btVector3 & orig = ctrl->GetObj().getWorldTransform().getOrigin();
    orig.setX(fLocalPosition.fX);
    orig.setY(fLocalPosition.fY);
    orig.setZ(fLocalPosition.fZ);
    //    if (fKinematicCCT)
//    {
//        hsVector3 disp(&fLocalPosition, &prevPosition);
//        if (disp.Magnitude() > 2.f)
//        {
//            // Teleport the underlying actor most of the way
//            disp.Normalize();
//            disp *= 0.001f;
//
//            hsPoint3 teleportPos = fLocalPosition - disp;
//            NxVec3 pos(teleportPos.fX, teleportPos.fY, teleportPos.fZ + kPhysZOffset);
//            fActor->setGlobalPosition(pos);
//        }
//
//        NxExtendedVec3 extPos(fLocalPosition.fX, fLocalPosition.fY, fLocalPosition.fZ + kCCTZOffset);
//        fController->setPosition(extPos);
//    }
//    else
//    {
//        NxVec3 pos(fLocalPosition.fX, fLocalPosition.fY, fLocalPosition.fZ + kPhysZOffset);
//        if (fActor->readBodyFlag(NX_BF_KINEMATIC))
//            fActor->moveGlobalPosition(pos);
//        else
//            fActor->setGlobalPosition(pos);
//    }
    
    // TODO: check 
//    ctrl->GetObj().SetWorldTransform(
//        btMatrix3x3(
//            l2w.fMap[0][0], l2w.fMap[0][1], l2w.fMap[0][2],
//            l2w.fMap[1][0], l2w.fMap[1][1], l2w.fMap[1][2],
//            l2w.fMap[2][0], l2w.fMap[2][1], l2w.fMap[2][2]
//        ),
//        btVector3 (l2w.fMap[3][0], l2w.fMap[3][1], l2w.fMap[3][2])
//    );
}

void plBtPhysicalControllerCore::GetPositionSim (hsPoint3 & pos) {
    dprintf ("plBtPhysicalControllerCore::GetPositionSim (0x%X, x=%i, y=%i, z=%i))", this, pos.fX, pos.fY, pos.fZ);
    btVector3 & orig = ctrl->GetObj().getWorldTransform().getOrigin();
    pos.fX = orig.x();
    pos.fY = orig.y();
    pos.fZ = orig.z();
}

void plBtPhysicalControllerCore::Move (hsVector3 displacement, unsigned int collideWith, unsigned int & collisionResults) {
    dprintf ("plBtPhysicalControllerCore::Move (0x%X, x=%i, y=%i, z=%i))", this, displacement.fX, displacement.fY, displacement.fZ);
    btVector3 & orig = ctrl->GetObj().getWorldTransform().getOrigin();
    orig.setX(orig.x() + displacement.fX);
    orig.setY(orig.y() + displacement.fY);
//    orig.setZ(orig.z() + displacement.fZ);
    collisionResults = kBottom;
}

void plBtPhysicalControllerCore::SetLinearVelocitySim (const hsVector3 & vec) {
    dprintf ("plBtPhysicalControllerCore::SetLinearVelocitySim (0x%X, x=%i, y=%i, z=%i))", this, vec.fX, vec.fY, vec.fZ);
    ctrl->SetVelocity (vec);
}

int plBtPhysicalControllerCore::SweepControllerPath (
    const hsPoint3 & startPos, const hsPoint3 & endPos, bool vsDynamics,
    bool vsStatics, uint32_t & vsSimGroups, std::vector<plControllerSweepRecord> & hits
) {
    printf ("plBtPhysicalControllerCore::SweepControllerPath (0x%X, start: x=%i, y=%i, z=%i; end: x=%i, y=%i, z=%i))", this, startPos.fX, startPos.fY, startPos.fZ, endPos.fX, endPos.fY, endPos.fZ);
    // TODO: btGhostObject::convexSweepTest or btDiscreteDynamicsWorld::convexSweepTest
	return 0;
}

void plBtPhysicalControllerCore::LeaveAge () { SetSubworld (nullptr); }


void plBtPhysicalControllerCore::Apply (float deltaSec) { IApply(deltaSec); }
void plBtPhysicalControllerCore::Update (float deltaSec, int nbSteps) { IUpdate(deltaSec, nbSteps, 0); } // alpha is the remain time: lest than 1/60s ??? useless?

