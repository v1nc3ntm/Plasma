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
#include "plBTPhysical.h"
#include "plSimulationMgr.h"

#include "hsBitVector.h"
#include "hsMatrix44.h"
#include "pnMessage/plCorrectionMsg.h"
#include "plPhysical/plPhysicalSndGroup.h"
#include "hsQuat.h"
#include "hsResMgr.h"
#include "pnSceneObject/plSimulationInterface.h"


#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
//#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>

class plBLPhysical::Private
{
public:
    static inline short collisionMask  (plSimDefs::Group group);
    static inline short collisionGroup (plSimDefs::Group group);
    
    plSimDefs::Bounds   bounds;
    plSimDefs::Group    group;
    
    plSimDefs::plLOSDB  losdb;
    uint32_t            reportsOn;
    
    plKey               objectKey, sceneKey, worldKey;
    plPhysicalSndGroup *soundGrp;
    
    hsBitVector         props;
    
    btRigidBody *       obj;
    
    enum MsgTypes {
        kPhysRefSndGroup
    };
};

inline short plBLPhysical::Private::collisionGroup (plSimDefs::Group group)
{
    switch (group)
    {
    case plSimDefs::kGroupStatic:          return 0x0001;
    case plSimDefs::kGroupAvatarBlocker:   return 0x0002;
    case plSimDefs::kGroupDynamicBlocker:  return 0x0004;
    case plSimDefs::kGroupAvatar:          return 0x0008;
    case plSimDefs::kGroupDynamic:         return 0x0010;
    case plSimDefs::kGroupDetector:        return 0x0020;
    case plSimDefs::kGroupLOSOnly:         return 0x0040;
    case plSimDefs::kGroupExcludeRegion:   return 0x0080;
    case plSimDefs::kGroupAvatarKinematic: return 0x0100;
    default:
        hsAssert(false, "collision group not implemented in plBLPhysical");
        return 0x0000;
    }
}

inline short plBLPhysical::Private::collisionMask (plSimDefs::Group group)
{
    switch (group)
    {
    case plSimDefs::kGroupStatic:          return 0x0018; //! (fix) Avatar + Dynamic
    case plSimDefs::kGroupAvatarBlocker:   return 0x0008; //! (fix) Avatar
    case plSimDefs::kGroupDynamicBlocker:  return 0x0010; //! (fix) Dynamic
    case plSimDefs::kGroupAvatar:          return 0x00BB; // (fix) Static + AvatarBlocker + Avatar + Dynamic + Detector + ExcludeRegion
    case plSimDefs::kGroupDynamic:         return 0x01BD; // (dyn) Static + DynamicBlocker + Avatar + Dynamic + Kinematic
    case plSimDefs::kGroupDetector:        return 0x0118; // (fix) Avatar + Dynamic + Kinematic
    case plSimDefs::kGroupLOSOnly:         return 0x0000; // (fix) -
    case plSimDefs::kGroupExcludeRegion:   return 0x0008; // (fix) Avatar (if not in seek mode)
    case plSimDefs::kGroupAvatarKinematic: return 0x0030; // (fix) Dynamic + Detector
    default:
        hsAssert(false, "collision group not implemented in plBLPhysical");
        return 0x0000;
    }
}


plBLPhysical::plBLPhysical ()
 : physic(new Private)
{
}
plBLPhysical::~plBLPhysical ()
{
    delete physic;
}

void plBLPhysical::Read  (hsStream* stream, hsResMgr* mgr)
{
    btRigidBody::btRigidBodyConstructionInfo info(0, nullptr, nullptr);
    
    plPhysical::Read(stream, mgr);
    
    // object physic
    info.m_mass        = stream->ReadLEScalar();
    info.m_friction    = stream->ReadLEScalar();
    info.m_restitution = stream->ReadLEScalar();
    
    // object type
    physic->bounds = (plSimDefs::Bounds)stream->ReadByte();
    physic->group  = (plSimDefs::Group) stream->ReadByte();
    
    // 
    physic->reportsOn =                      stream->ReadLE32();
    physic->losdb     = (plSimDefs::plLOSDB) stream->ReadLE16();
    if (physic->losdb == plSimDefs::kLOSDBSwimRegion)
        // hack for swim regions currently they are labeled as static av blockers
        physic->group = plSimDefs::kGroupMax;
    
    // Identifiers
    physic->objectKey = mgr->ReadKey(stream);
    physic->sceneKey  = mgr->ReadKey(stream);
    physic->worldKey  = mgr->ReadKey(stream);
    
    // sound group
    mgr->ReadKeyNotifyMe(stream, new plGenRefMsg(GetKey(), plRefMsg::kOnCreate, 0, Private::kPhysRefSndGroup), plRefFlags::kActiveRef);
    
    // Object position and rotation in world
    {
        btVector3 pos(
                stream->ReadLEScalar(),
                stream->ReadLEScalar(),
                stream->ReadLEScalar()
        );
        info.m_startWorldTransform = btTransform(
            btQuaternion(
                stream->ReadLEScalar(),
                stream->ReadLEScalar(),
                stream->ReadLEScalar(),
                stream->ReadLEScalar()
            ),
            pos
        );
    }
    
    // properties
    physic->props.Read(stream);
    
    hsPoint3 offset;
    uint32_t count; 
    
    // Object shape
    switch (physic->bounds)
    {
    case plSimDefs::kBoxBounds:
        info.m_collisionShape = new btBoxShape(
            btVector3(
                stream->ReadLEScalar(),
                stream->ReadLEScalar(),
                stream->ReadLEScalar()
            )
        );
        offset.Read(stream); // offset?
        break;
    case plSimDefs::kSphereBounds:
        info.m_collisionShape = new btSphereShape(stream->ReadLEScalar());
        offset.Read(stream); // offset?
        break;
    case plSimDefs::kHullBounds:
        btConvexHullShape * hull; hull = new btConvexHullShape;
        count = stream->ReadLE32();
        
        for (int i = 0; i < count; i++)
            hull->addPoint(
                btVector3(
                    stream->ReadLEScalar(),
                    stream->ReadLEScalar(),
                    stream->ReadLEScalar()
                )
            );
        info.m_collisionShape = hull;
        
        break;
    case plSimDefs::kProxyBounds:
    case plSimDefs::kExplicitBounds:
        btCompoundShape * shape; shape = new btCompoundShape();
        
        uint32_t ptCount; ptCount = stream->ReadLE32();
        btVector3 * pts; pts = new btVector3[ptCount];
        for (int i = 0; i < ptCount; i++)
            pts[i] = btVector3(
                stream->ReadLEScalar(),
                stream->ReadLEScalar(),
                stream->ReadLEScalar()
            );
        
        count = stream->ReadLE32();
        for (int i = 0; i < count; i++)
        {
            uint16_t a, b, c;
            a = stream->ReadLE16(); hsAssert(a < ptCount, "invalid face vertex index");
            b = stream->ReadLE16(); hsAssert(b < ptCount, "invalid face vertex index");
            c = stream->ReadLE16(); hsAssert(c < ptCount, "invalid face vertex index");
            
            shape->addChildShape(
                btTransform(),
                new btTriangleShape(pts[a], pts[b], pts[c])
            );
        }
        delete[] pts;
        
        info.m_collisionShape = shape;
        
        break;
    default:
        hsAssert(false, "bounds type not supported");
    }
    
    physic->obj = new btRigidBody(info);
    plSimulationMgr::GetScene(physic->worldKey).addRigidBody(
        physic->obj,
        Private::collisionGroup(physic->group),
        Private::collisionMask(physic->group)
    );
    
    if (physic->props.IsBitSet(plSimulationInterface::kStartInactive))
        physic->obj->forceActivationState(DISABLE_SIMULATION);
    
    // TODO: ...
    
}
void plBLPhysical::Write (hsStream* stream, hsResMgr* mgr)
{
    plPhysical::Write(stream, mgr);
    
    
    stream->WriteLEScalar(1.f / physic->obj->getInvMass());
    stream->WriteLEScalar(physic->obj->getFriction());
    stream->WriteLEScalar(physic->obj->getRestitution());
    stream->WriteByte(physic->bounds);
    stream->WriteByte(physic->group);
    stream->WriteLE32(physic->reportsOn);
    stream->WriteLE16(physic->losdb);
    mgr->WriteKey(stream, physic->objectKey);
    mgr->WriteKey(stream, physic->sceneKey);
    mgr->WriteKey(stream, physic->worldKey);
    mgr->WriteKey(stream, physic->soundGrp);
    
    {   const btTransform & t = physic->obj->getWorldTransform();
        {   const btVector3 & pos = t.getOrigin();
            stream->WriteLEScalar(pos.x());
            stream->WriteLEScalar(pos.y());
            stream->WriteLEScalar(pos.z());
        }
        {   btQuaternion rot;
            t.getBasis().getRotation(rot);
            stream->WriteLEScalar(rot.x());
            stream->WriteLEScalar(rot.y());
            stream->WriteLEScalar(rot.z());
            stream->WriteLEScalar(rot.w());
        }
    }
    
    physic->props.Write(stream);
    
    hsAssert(false, "TODO");
    switch (physic->bounds)
    {
    case plSimDefs::kBoxBounds:
        //TODO
        break;
    case plSimDefs::kSphereBounds:
        //TODO
        break;
    case plSimDefs::kHullBounds:
        //TODO
        break;
    case plSimDefs::kProxyBounds:
    case plSimDefs::kExplicitBounds:
        //TODO
        break;
    default:
        hsAssert(false, "bounds type not supported");
    }
}



float plBLPhysical::GetMass ()        { return 1.f / physic->obj->getInvMass(); }
int   plBLPhysical::GetGroup () const { return physic->group; }

void     plBLPhysical::   AddLOSDB  (uint16_t flag) { hsSetBits(  (int &)physic->losdb, flag); }
void     plBLPhysical::RemoveLOSDB  (uint16_t flag) { hsClearBits((int &)physic->losdb, flag); } 
bool     plBLPhysical::  IsInLOSDB  (uint16_t flag) { hsCheckBits(       physic->losdb, flag); }
uint16_t plBLPhysical::GetAllLOSDBs () { return physic->losdb; }


plKey plBLPhysical::GetObjectKey () const { return physic->objectKey; }
void  plBLPhysical::SetObjectKey (plKey key) { physic->objectKey = key; }

plKey plBLPhysical::GetSceneNode () const { return physic->sceneKey; }
void  plBLPhysical::SetSceneNode (plKey node) {
    plSimulationMgr::GetScene (physic->sceneKey).removeRigidBody(physic->obj);
    physic->sceneKey = node;
    plSimulationMgr::GetScene(node).addRigidBody(
        physic->obj,
        Private::collisionGroup(physic->group),
        Private::collisionMask(physic->group)
    );
}

plKey                plBLPhysical::GetWorldKey ()   const { return physic->worldKey; }
plPhysicalSndGroup * plBLPhysical::GetSoundGroup () const { return physic->soundGrp; }


void plBLPhysical::GetPositionSim (hsPoint3 & pos) const
{
    btVector3 & o = physic->obj->getWorldTransform().getOrigin();
    pos.fX = o.x();
    pos.fY = o.y();
    pos.fZ = o.z();
}

void plBLPhysical::GetTransform (hsMatrix44& l2w, hsMatrix44& w2l)
{
    const btTransform & t = physic->obj->getWorldTransform();
    const btMatrix3x3 & r = t.getBasis();
    const btVector3   & o = t.getOrigin();
    
    // make 4x4 matrix from translation and 3x3 rotation matrix
    w2l.fMap[0][0] = r[0].x();
    w2l.fMap[1][0] = r[1].x();
    w2l.fMap[2][0] = r[2].x();
    w2l.fMap[3][0] = o   .x();
    
    w2l.fMap[0][1] = r[0].y();
    w2l.fMap[1][1] = r[1].y();
    w2l.fMap[2][1] = r[2].y();
    w2l.fMap[3][1] = o   .y();
    
    w2l.fMap[0][2] = r[0].z();
    w2l.fMap[1][2] = r[1].z();
    w2l.fMap[2][2] = r[2].z();
    w2l.fMap[3][2] = o   .z();
    
    w2l.fMap[0][3] = 0;
    w2l.fMap[1][3] = 0;
    w2l.fMap[2][3] = 0;
    w2l.fMap[3][3] = 1;
    
    w2l.GetInverse(&l2w);
}
void plBLPhysical::SetTransform (const hsMatrix44& l2w, const hsMatrix44& w2l, bool force)
{
    btTransform & t = physic->obj->getWorldTransform();
    btMatrix3x3 & r = t.getBasis();
    btVector3   & o = t.getOrigin();
    
    hsAssert(w2l.fMap[0][3] == 0, "invalid transform matrix");
    hsAssert(w2l.fMap[1][3] == 0, "invalid transform matrix");
    hsAssert(w2l.fMap[2][3] == 0, "invalid transform matrix");
    hsAssert(w2l.fMap[3][3] == 1, "invalid transform matrix");
    
    hsAssert(w2l.fMap[0][0] + w2l.fMap[1][0] + w2l.fMap[2][0] == 1.f, "scaling transform not supported by bullet");
    hsAssert(w2l.fMap[0][1] + w2l.fMap[1][1] + w2l.fMap[2][1] == 1.f, "scaling transform not supported by bullet");
    hsAssert(w2l.fMap[0][2] + w2l.fMap[1][2] + w2l.fMap[2][2] == 1.f, "scaling transform not supported by bullet");
    
    r[0].setX(w2l.fMap[0][0]);
    r[1].setX(w2l.fMap[1][0]);
    r[2].setX(w2l.fMap[2][0]);
    o   .setX(w2l.fMap[3][0]);
    
    r[0].setY(w2l.fMap[0][1]);
    r[1].setY(w2l.fMap[1][1]);
    r[2].setY(w2l.fMap[2][1]);
    o   .setY(w2l.fMap[3][1]);
    
    r[0].setZ(w2l.fMap[0][2]);
    r[1].setZ(w2l.fMap[1][2]);
    r[2].setZ(w2l.fMap[2][2]);
    o   .setZ(w2l.fMap[3][2]);
}

bool         plBLPhysical::GetProperty (int prop) const
{
    return physic->props.IsBitSet(prop);
}
plPhysical & plBLPhysical::SetProperty (int prop, bool b)
{
    switch (prop)
    {
    case plSimulationInterface::kDisable:
        // TODO: test if it do what I want...
        physic->obj->forceActivationState(
            b ? DISABLE_SIMULATION : ACTIVE_TAG
        );
        break;
    case plSimulationInterface::kPinned:
        // TODO: test if it work...
        if (b)
            physic->obj->setFlags(
                physic->obj->getFlags() |  btCollisionObject::CF_STATIC_OBJECT
            );
        else
            physic->obj->setFlags(
                physic->obj->getFlags() & ~btCollisionObject::CF_STATIC_OBJECT
            );
        break;
    case plSimulationInterface::kPassive:           // in SendNewLocation()
    case plSimulationInterface::kPhysAnim:          // TODO?
    case plSimulationInterface::kStartInactive:     // in Read()
    case plSimulationInterface::kNoSynchronize:     // TODO?
    case plSimulationInterface::kNoOwnershipChange: // TODO?
    case plSimulationInterface::kAvAnimPushable:    // TODO?
        break;
    default:
        hsAssert(false, "unknow property");
    }
    
    physic->props.SetBit(prop, b);
}


bool plBLPhysical::GetLinearVelocitySim (hsVector3 & vel) const
{
    const btVector3 & vec = physic->obj->getLinearVelocity();
    vel.fX = vec.x();
    vel.fY = vec.y();
    vel.fZ = vec.z();
    return true;
}
void plBLPhysical::SetLinearVelocitySim (const hsVector3 & vec)
{
    physic->obj->setLinearVelocity(btVector3(vec.fX, vec.fY, vec.fZ));
}
bool plBLPhysical::GetAngularVelocitySim (hsVector3 & vel) const
{
    const btVector3 & vec = physic->obj->getAngularVelocity();
    vel.fX = vec.x();
    vel.fY = vec.y();
    vel.fZ = vec.z();
    return true;
}
void plBLPhysical::SetAngularVelocitySim (const hsVector3 & vec)
{
    physic->obj->setAngularVelocity(btVector3(vec.fX, vec.fY, vec.fZ));
}
void plBLPhysical::ClearLinearVelocity ()
{
    physic->obj->setLinearVelocity(btVector3(0, 0, 0));
}

void plBLPhysical::GetSyncState (hsPoint3 & pos, hsQuat & rot, hsVector3 & linV, hsVector3 & angV)
{
    GetPositionSim(pos);
    
    btQuaternion q;
    physic->obj->getWorldTransform().getBasis().getRotation(q);
    rot.fX = q.x();
    rot.fY = q.y();
    rot.fZ = q.z();
    rot.fW = q.w();
    
    GetLinearVelocitySim(linV);
    GetAngularVelocitySim(angV);
}
void plBLPhysical::SetSyncState (hsPoint3 * pos, hsQuat * rot, hsVector3 * linV, hsVector3 * angV)
{
    if (pos || rot)
    {
        btTransform & t = physic->obj->getWorldTransform();
        
        if (pos)
        {
            btVector3 & o = t.getOrigin();
            o.setX(pos->fX);
            o.setY(pos->fY);
            o.setZ(pos->fZ);
        }
        if (rot)
            t.setRotation(btQuaternion(rot->fX, rot->fY, rot->fZ, rot->fW));
    }
    
    if (linV)  SetLinearVelocitySim(*linV);
    if (angV) SetAngularVelocitySim(*angV);
}

void plBLPhysical::SetHitForce (const hsVector3& force, const hsPoint3& pos)
{
    physic->obj->applyForce( // FIXME: Force or Impulse?
        btVector3(force.fX, force.fY, force.fZ),
        btVector3(  pos.fX,   pos.fY,   pos.fZ)
    );
}

void plBLPhysical::SendNewLocation (bool synchTransform, bool forceUpdate)
{
    // TODO:
    // we only send if:
    // - the body is active or forceUpdate is on
    // - the mass is non-zero
    // - the physical is not passive
    
    if (!physic->props.IsBitSet(plSimulationInterface::kPassive))
    {
        hsMatrix44 l2w, w2l;
        
        GetTransform(l2w, w2l);
        (new plCorrectionMsg(physic->objectKey, l2w, w2l, synchTransform))->Send();
    }
}

void plBLPhysical::ExcludeRegionHack (bool cleared)
{
//    physic->obj->set
    hsAssert(false, "TODO");
}

plDrawableSpans * plBLPhysical::CreateProxy (hsGMaterial* mat, hsTArray<uint32_t> & idx, plDrawableSpans * addTo)
{
    
    hsAssert(false, "TODO?");
    return addTo;
}


bool plBLPhysical::DoReportOn (plSimDefs::Group group) const { return hsCheckBits(physic->reportsOn, 1<<group); }

bool plBLPhysical::HandleRefMsg (plGenRefMsg * refMsg) {
//    plKey refKey = refMsg->GetRef()->GetKey();
//    plString refKeyName = refKey ? refKey->GetName() : "MISSING";
    
    switch (refMsg->fType) {
    case Private::kPhysRefSndGroup:
        switch (refMsg->GetContext())
        {
        case plRefMsg::kOnCreate:
//        case plRefMsg::kOnRequest:
            physic->soundGrp = plPhysicalSndGroup::ConvertNoRef(refMsg->GetRef());
            break;

//        case plRefMsg::kOnDestroy:
//            fSndGroup = nullptr;
//            break;
        default:
            hsAssert(false, "unhandled refMsg context");
        }
    default:
        hsAssert(false, "unhandled refMsg type");
    }
}


