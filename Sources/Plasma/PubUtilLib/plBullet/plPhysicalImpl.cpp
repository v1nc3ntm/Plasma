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
#include "plPhysicalImpl.h"
#include "plSimulationMgrImpl.h"

#include "hsBitVector.h"
#include "hsGeometry3.h"
#include "hsMatrix44.h"
#include "hsQuat.h"
#include "hsResMgr.h"
#include "plPhysical/plPhysicalSndGroup.h"
#include "plPhysical/plSimDefs.h"
#include "plMessage/plAngularVelocityMsg.h"
#include "plMessage/plLinearVelocityMsg.h"
#include "pnMessage/plCorrectionMsg.h"
#include "pnMessage/plRefMsg.h"
#include "pnMessage/plSimulationMsg.h"
#include "pnSceneObject/plSimulationInterface.h"

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btVector3.h>

class plPhysicalImpl::Private : plBtDefs::ObjectData {
public:
    btRigidBody *       obj;
    plSimDefs::Bounds   bounds;
    plSimDefs::Group    group;
    uint32_t            reportsOn; // bit mask of groups who collide are repported by this sensor (group must be kGroupDetector to take effect)
    plSimDefs::plLOSDB  losdb;
    plKey               objKey, sceneKey, worldKey;
    plPhysicalSndGroup* sndGroup;
    hsBitVector         props;
    
    bool                hit;
    hsVector3           force;
    hsPoint3            pos;
    
    Private () : hit(false) {}
    ~Private () {
        if (obj)
            plSimulationMgrImpl::GetOrCreateWorld(worldKey).removeRigidBody(obj);
    }
    
    virtual plKey               GetObjKey () const { return objKey; }
    virtual plSimDefs::plLOSDB  GetLOSDBs () const { return losdb; }
    virtual bool                IsSeeking () const { FATAL("mustn't be called: error in collision filters"); return false; }
    
    bool HandleRefMsg (plGenRefMsg * refMsg) {
        uint8_t     refCtxt = refMsg->GetContext();
        plKey        refKey = refMsg->GetRef()->GetKey();

        plString refKeyName = refKey ? refKey->GetName() : "MISSING";

       switch (refCtxt)
        {
        case plRefMsg::kOnCreate:
        case plRefMsg::kOnRequest:
            sndGroup = plPhysicalSndGroup::ConvertNoRef(refMsg->GetRef());
            break;

        case plRefMsg::kOnDestroy:
            sndGroup = nullptr;
            break;
        }

        return true;
    }
    
    static unsigned short ConvertMask (plSimDefs::Group group, unsigned short reportsOn) {
        switch (group) {
        case plSimDefs::kGroupStatic:           return (1 << plSimDefs::kGroupAvatar)
                                                     | (1 << plSimDefs::kGroupDynamic);
        case plSimDefs::kGroupAvatarBlocker:    return (1 << plSimDefs::kGroupAvatar);
        case plSimDefs::kGroupDynamicBlocker:   return (1 << plSimDefs::kGroupDynamic);
//        case plSimDefs::kGroupAvatar:           return (1 << plSimDefs::kGroupStatic)
//                                                     | (1 << plSimDefs::kGroupAvatarBlocker)
//                                                     | (1 << plSimDefs::kGroupDynamic)
//                                                     | (1 << plSimDefs::kGroupDetector)
//                                                     | (1 << plSimDefs::kGroupExcludeRegion);
        case plSimDefs::kGroupDynamic:          return (1 << plSimDefs::kGroupStatic)
                                                     | (1 << plSimDefs::kGroupDynamicBlocker)
                                                     | (1 << plSimDefs::kGroupAvatar)
                                                     | (1 << plSimDefs::kGroupDynamic)
                                                     | (1 << plSimDefs::kGroupDetector)
                                                     | (1 << plSimDefs::kGroupAvatarKinematic);
        case plSimDefs::kGroupDetector:         return reportsOn;
        case plSimDefs::kGroupLOSOnly:          return 0;
        case plSimDefs::kGroupExcludeRegion:    return (1 << plSimDefs::kGroupAvatar);
//        case plSimDefs::kGroupAvatarKinematic:  return (1 << plSimDefs::kGroupDynamic)
//                                                     | (1 << plSimDefs::kGroupDetector);
        case plSimDefs::kGroupMax:              break;
        }
        
        FATAL("Invalid group for no-avatar physical");
        return 0;
    }
    
};

plPhysicalImpl::plPhysicalImpl () : physic(new Private) {}
plPhysicalImpl::~plPhysicalImpl () { delete physic; }


void plPhysicalImpl::Read (hsStream * stream, hsResMgr * mgr)
{
    plPhysical::Read(stream, mgr);
    
    // Physical properties
    float mass         = stream->ReadLEScalar();
    btRigidBody::btRigidBodyConstructionInfo info(mass, nullptr, nullptr);
    info.m_friction    = stream->ReadLEScalar();
    info.m_restitution = stream->ReadLEScalar();
    
    // Type
    physic->bounds = (plSimDefs::Bounds)stream->ReadByte();
    physic->group  = (plSimDefs::Group) stream->ReadByte();
    
    // group mask for collision reports
    physic->reportsOn = stream->ReadLE32();
    
    // Line Of Sight (data base?)
    physic->losdb     = (plSimDefs::plLOSDB) stream->ReadLE16();
    if (physic->losdb == plSimDefs::kLOSDBSwimRegion)
        // hack for swim regions currently they are labeled as static av blockers
        physic->group = plSimDefs::kGroupMax;
    
    // Id
    physic->objKey = mgr->ReadKey(stream);
    
    // Parents id
    physic->sceneKey = mgr->ReadKey(stream);
    physic->worldKey = mgr->ReadKey(stream);
    
    // Sound group
    mgr->ReadKeyNotifyMe(stream, new plGenRefMsg(
        GetKey(), plRefMsg::kOnCreate, 0, 0
    ), plRefFlags::kActiveRef);
    
    // Position
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
    
    // Properties
    physic->props.Read(stream);
    
    // Shape
    switch (physic->bounds) {
    case plSimDefs::kBoxBounds: {
        info.m_collisionShape = new btBoxShape(
            btVector3(
                stream->ReadLEScalar(),
                stream->ReadLEScalar(),
                stream->ReadLEScalar()
            )
        );
        hsPoint3 offset; offset.Read(stream); // offset?
        break;
    }
    case plSimDefs::kSphereBounds: {
        info.m_collisionShape = new btSphereShape(stream->ReadLEScalar());
        hsPoint3 offset; offset.Read(stream); // offset?
        break;
    }
    case plSimDefs::kHullBounds: {
        btConvexHullShape * hull = new btConvexHullShape;
        uint32_t count = stream->ReadLE32();
        
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
    }
    case plSimDefs::kProxyBounds:
    case plSimDefs::kExplicitBounds: {
        btCompoundShape * shape = new btCompoundShape();
        stream->Skip(28); // skip "NXS\x01MESH" 0 8 0,001 255 0
        uint32_t ptCount = stream->ReadLE32();
        uint32_t count = stream->ReadLE32();
        
        btVector3 * pts = new btVector3[ptCount];
        for (int i = 0; i < ptCount; i++)
            pts[i] = btVector3(
                stream->ReadLEScalar(),
                stream->ReadLEScalar(),
                stream->ReadLEScalar()
            );
        
        for (int i = 0; i < count; i++)
        {
            uint8_t a, b, c;
            a = stream->ReadByte(); hsAssert(a < ptCount, "invalid face vertex index");
            b = stream->ReadByte(); hsAssert(b < ptCount, "invalid face vertex index");
            c = stream->ReadByte(); hsAssert(c < ptCount, "invalid face vertex index");
            
            shape->addChildShape(
                btTransform(),
                new btTriangleShape(pts[a], pts[b], pts[c])
            );
        }
        delete[] pts;
        info.m_collisionShape = shape;
        break;
    }
    default:
        hsAssert(false, "bounds type not supported");
    }
    
    ////////////////////////////////////////////////////
    
    physic->obj = new btRigidBody(info);
    physic->obj->setUserPointer((plBtDefs::ObjectData*)physic);
    
    if (physic->props.IsBitSet(plSimulationInterface::kStartInactive))
        physic->obj->forceActivationState(DISABLE_SIMULATION);
    
    if (physic->group == plSimDefs::kGroupDetector)
        physic->obj->setCollisionFlags(
            physic->obj->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE
        );
    
    
    plSimulationMgrImpl::GetOrCreateWorld(physic->worldKey).addRigidBody(
        physic->obj, (1 << physic->group), Private::ConvertMask(physic->group, physic->reportsOn)
    );
}

void plPhysicalImpl::Write (hsStream * stream, hsResMgr * mgr) {
    plPhysical::Write(stream, mgr);
    
    stream->WriteLEScalar(1.f / physic->obj->getInvMass());
    stream->WriteLEScalar(physic->obj->getFriction());
    stream->WriteLEScalar(physic->obj->getRestitution());
    stream->WriteByte(physic->bounds);
    stream->WriteByte(physic->group);
    stream->WriteLE32(physic->reportsOn);
    stream->WriteLE16(physic->losdb);
    mgr->WriteKey(stream, physic->objKey);
    mgr->WriteKey(stream, physic->sceneKey);
    mgr->WriteKey(stream, physic->worldKey);
    mgr->WriteKey(stream, physic->sndGroup);
    {
        const btTransform & t = physic->obj->getWorldTransform();
        const btVector3 & pos = t.getOrigin();
        
        stream->WriteLEScalar(pos.x());
        stream->WriteLEScalar(pos.y());
        stream->WriteLEScalar(pos.z());
        
        btQuaternion rot;
        t.getBasis().getRotation(rot);
        stream->WriteLEScalar(rot.x());
        stream->WriteLEScalar(rot.y());
        stream->WriteLEScalar(rot.z());
        stream->WriteLEScalar(rot.w());
    }
    
    physic->props.Write(stream);
    
    //TODO: write shape
    hsAssert(false, "TODO");
}

bool plPhysicalImpl::MsgReceive (plMessage * msg) {
    if (plGenRefMsg * refM = plGenRefMsg::ConvertNoRef(msg))
    {
        return physic->HandleRefMsg(refM);
    }
    else if (plSimulationMsg::ConvertNoRef(msg))
    {
        if(plLinearVelocityMsg * velMsg = plLinearVelocityMsg::ConvertNoRef(msg))
        {
            SetLinearVelocitySim(velMsg->Velocity());
            return true;
        }
        
        if(plAngularVelocityMsg * angvelMsg = plAngularVelocityMsg::ConvertNoRef(msg))
        {
            SetAngularVelocitySim(angvelMsg->AngularVelocity());
            return true;
        }

        
        return false;
    }
    // couldn't find a local handler: pass to the base
    else
        return plPhysical::MsgReceive(msg);
}



bool plPhysicalImpl::GetProperty (int prop) const
{ return physic->props.IsBitSet(prop) != 0; }

plPhysical & plPhysicalImpl::SetProperty (int prop, bool b) {
    switch (prop) {
    case plSimulationInterface::kDisable:
        // TODO: test
        physic->obj->forceActivationState(
            b ? DISABLE_SIMULATION : ACTIVE_TAG
        );
        break;
        
    case plSimulationInterface::kPinned:
        // TODO: test
        if (b)
            physic->obj->setFlags(
                physic->obj->getFlags() |  btCollisionObject::CF_STATIC_OBJECT
            );
        else
            physic->obj->setFlags(
                physic->obj->getFlags() & ~btCollisionObject::CF_STATIC_OBJECT
            );
        break;
        
    case plSimulationInterface::kPassive:           //TODO
    case plSimulationInterface::kPhysAnim:          //TODO
    case plSimulationInterface::kStartInactive:     //TODO
    case plSimulationInterface::kNoSynchronize:     //TODO
    case plSimulationInterface::kNoOwnershipChange: //TODO
    case plSimulationInterface::kAvAnimPushable:    //TODO
        break;
        
    default: hsAssert(false, "unknow property");
    }
    
    physic->props.SetBit(prop, b);
    
    return *this;
}

plKey plPhysicalImpl::GetObjectKey () const    { return physic->objKey; }
void  plPhysicalImpl::SetObjectKey (plKey key) { physic->objKey = key; }

plKey plPhysicalImpl::GetSceneNode () const     { return physic->sceneKey; }
void  plPhysicalImpl::SetSceneNode (plKey node) { physic->sceneKey = node; }

bool plPhysicalImpl::GetLinearVelocitySim (hsVector3 & vel) const {
    const btVector3 & vec = physic->obj->getLinearVelocity();
    vel.fX = vec.x();
    vel.fY = vec.y();
    vel.fZ = vec.z();
    return true;
}
void plPhysicalImpl::SetLinearVelocitySim (const hsVector3 & vel)
{ physic->obj->setLinearVelocity(btVector3(vel.fX, vel.fY, vel.fZ)); }
void plPhysicalImpl::ClearLinearVelocity ()
{ physic->obj->setLinearVelocity(btVector3(0, 0, 0)); }

bool plPhysicalImpl::GetAngularVelocitySim (hsVector3 & vel) const {
    const btVector3 & vec = physic->obj->getAngularVelocity();
    vel.fX = vec.x();
    vel.fY = vec.y();
    vel.fZ = vec.z();
    return true;
}
void plPhysicalImpl::SetAngularVelocitySim (const hsVector3 & vel)
{ physic->obj->setAngularVelocity(btVector3(vel.fX, vel.fY, vel.fZ)); }

void plPhysicalImpl::GetTransform (hsMatrix44 & l2w, hsMatrix44 & w2l) {
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
void plPhysicalImpl::SetTransform (const hsMatrix44& l2w, const hsMatrix44& w2l, bool force) {
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

int plPhysicalImpl::GetGroup () const { return physic->group; }

uint16_t plPhysicalImpl::GetAllLOSDBs ()             { return                   physic->losdb ; }
bool     plPhysicalImpl::  IsInLOSDB (uint16_t flag) { return hsCheckBits(flag, physic->losdb); }
void     plPhysicalImpl::   AddLOSDB (uint16_t flag) {          hsSetBits(flag, physic->losdb); }
void     plPhysicalImpl::RemoveLOSDB (uint16_t flag) {        hsClearBits(flag, physic->losdb); }

plKey plPhysicalImpl::GetWorldKey () const { return physic->worldKey; }

plPhysicalSndGroup * plPhysicalImpl::GetSoundGroup () const { return physic->sndGroup; }

void plPhysicalImpl::GetPositionSim (hsPoint3 & pos) const {
    btVector3 & o = physic->obj->getWorldTransform().getOrigin();
    pos.fX = o.x();
    pos.fY = o.y();
    pos.fZ = o.z();
}

void plPhysicalImpl::SendNewLocation (bool synchTransform, bool isSynchUpdate) {
    // TODO:
    // we only send if:
    // - the body is active or forceUpdate is on
    // - the mass is non-zero
    // - the physical is not passive
    
    if (!physic->props.IsBitSet(plSimulationInterface::kPassive))
    {
        hsMatrix44 l2w, w2l;
        
        GetTransform(l2w, w2l);
        (new plCorrectionMsg(physic->objKey, l2w, w2l, synchTransform))->Send();
    }
}

void plPhysicalImpl::SetHitForce (const hsVector3& force, const hsPoint3& pos) {
    physic->hit = true;
    physic->force = force;
    physic->pos   = pos;
}
void plPhysicalImpl::ApplyHitForce () {
    if (physic->hit)
        physic->obj->applyForce( // FIXME: Force or Impulse?
            btVector3(physic->force.fX, physic->force.fY, physic->force.fZ),
            btVector3(physic->  pos.fX, physic->  pos.fY, physic->  pos.fZ)
        );
}
void plPhysicalImpl::ResetHitForce () {
    physic->hit = false;
}

void plPhysicalImpl::GetSyncState (hsPoint3& pos, hsQuat& rot, hsVector3& linV, hsVector3& angV) {
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
void plPhysicalImpl::SetSyncState (hsPoint3* pos, hsQuat* rot, hsVector3* linV, hsVector3* angV) {
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

void plPhysicalImpl::ExcludeRegionHack (bool cleared) {
    hsAssert(false, "TODO");
}

plDrawableSpans * plPhysicalImpl::CreateProxy (hsGMaterial* mat, hsTArray<uint32_t>& idx, plDrawableSpans* addTo) {
    hsAssert(false, "TODO");
    
    return addTo;
}

float plPhysicalImpl::GetMass () { return 1.f / physic->obj->getInvMass(); }

