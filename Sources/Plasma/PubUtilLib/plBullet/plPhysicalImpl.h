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
#ifndef plPhysicalImpl_h_inc
#define plPhysicalImpl_h_inc

#include "pnFactory/plCreatable.h"
#include "plPhysical.h"

class plPhysicalImpl : public plPhysical
{
public:
    plPhysicalImpl ();
    virtual ~plPhysicalImpl ();

    CLASSNAME_REGISTER(plPhysicalImpl);
    GETINTERFACE_ANY(plPhysicalImpl, plPhysical);

    virtual void Write (hsStream * s, hsResMgr * mgr);
    virtual void  Read (hsStream * s, hsResMgr * mgr);

    virtual bool MsgReceive (plMessage * msg);

    //
    // From plPhysical
    //
    virtual bool         GetProperty (int prop) const;
    virtual plPhysical & SetProperty (int prop, bool b);

    virtual plKey GetObjectKey () const;
    virtual void  SetObjectKey (plKey key);

    virtual plKey GetSceneNode () const;
    virtual void  SetSceneNode (plKey node);

    virtual bool GetLinearVelocitySim (hsVector3& vel) const;
    virtual void SetLinearVelocitySim (const hsVector3& vel);
    virtual void  ClearLinearVelocity ();

    virtual bool GetAngularVelocitySim (hsVector3& vel) const;
    virtual void SetAngularVelocitySim (const hsVector3& vel);

    virtual void GetTransform (hsMatrix44& l2w, hsMatrix44& w2l);
    virtual void SetTransform (const hsMatrix44& l2w, const hsMatrix44& w2l, bool force=false);

    virtual int GetGroup () const;

    virtual uint16_t GetAllLOSDBs ();
    virtual bool       IsInLOSDB (uint16_t flag);
    virtual void        AddLOSDB (uint16_t flag);
    virtual void     RemoveLOSDB (uint16_t flag);

    virtual plKey GetWorldKey () const;

    virtual plPhysicalSndGroup * GetSoundGroup () const;

    virtual void GetPositionSim (hsPoint3 & pos) const;

    virtual void SendNewLocation (bool synchTransform = false, bool isSynchUpdate = false);

    virtual void SetHitForce (const hsVector3& force, const hsPoint3& pos);
    virtual void ApplyHitForce (); // TODO: remove?
    virtual void ResetHitForce ();

    virtual void GetSyncState (hsPoint3& pos, hsQuat& rot, hsVector3& linV, hsVector3& angV);
    virtual void SetSyncState (hsPoint3* pos, hsQuat* rot, hsVector3* linV, hsVector3* angV);

    virtual void ExcludeRegionHack (bool cleared);

    virtual plDrawableSpans * CreateProxy (hsGMaterial* mat, hsTArray<uint32_t>& idx, plDrawableSpans* addTo);

    virtual float GetMass ();
    
    class Private;
    Private * physic;
};

#endif
