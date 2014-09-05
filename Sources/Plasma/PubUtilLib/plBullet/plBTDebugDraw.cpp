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

#include "plBTDebugDraw.h"


// primitives
void plBTDebugDraw::drawLine (
    const btVector3 & from,
    const btVector3 & to,
    const btVector3 & color
) {
    // TODO
}
void plBTDebugDraw::drawLine (
    const btVector3 & from,
    const btVector3 & to,
    const btVector3 & colorFrom,
    const btVector3 & colorTo
) {
    // TODO
}

void plBTDebugDraw::drawPlane (
    const btVector3 &   normal,
    btScalar            planeConst,
    const btTransform & transform,
    const btVector3 &   color
) {
    // TODO
}

void plBTDebugDraw::drawBox (
    const btVector3 & min,
    const btVector3 & max,
    const btVector3 & color
) {
    // TODO
}
void plBTDebugDraw::drawBox (
    const btVector3 &   min,
    const btVector3 &   max,
    const btTransform & trans,
    const btVector3 &   color
) {
    // TODO
}

void plBTDebugDraw::drawTriangle (
    const btVector3 & v0,
    const btVector3 & v1,
    const btVector3 & v2,
    const btVector3 & n0,
    const btVector3 & n1,
    const btVector3 & n2,
    const btVector3 & color,
    btScalar alpha
) {
    // TODO
}

void plBTDebugDraw::drawTriangle (
    const btVector3 & v0,
    const btVector3 & v1,
    const btVector3 & v2,
    const btVector3 & color,
    btScalar alpha
) {
    // TODO
}

void plBTDebugDraw::drawCylinder (
    btScalar            radius,
    btScalar            halfHeight,
    int                 upAxis,
    const btTransform & transform,
    const btVector3 &   color
) {
    // TODO
}
void plBTDebugDraw::drawCone (
    btScalar            radius,
    btScalar            height,
    int                 upAxis,
    const btTransform & transform,
    const btVector3 &   color
) {
    // TODO
}
void plBTDebugDraw::drawCapsule (
    btScalar            radius,
    btScalar            halfHeight,
    int                 upAxis,
    const btTransform & transform,
    const btVector3 &   color
) {
    // TODO
}

void plBTDebugDraw::drawSphere (
    btScalar            radius,
    const btTransform & transform,
    const btVector3 &   color
) {
    // TODO
}
void plBTDebugDraw::drawSphere (
    const btVector3 & center,
    btScalar          radius,
    const btVector3 & color
) {
    // TODO
}

    
// advanced primitives
void plBTDebugDraw::drawArc (
    const btVector3 & center,
    const btVector3 & normal,
    const btVector3 & axis,
    btScalar          radiusA,
    btScalar          radiusB,
    btScalar          minAngle,
    btScalar          maxAngle,
    const btVector3 & color,
    bool              drawSect,
    btScalar          stepDegrees
) {
    // TODO
}
void plBTDebugDraw::drawSpherePatch (
    const btVector3 & center,
    const btVector3 & up,
    const btVector3 & axis,
    btScalar          radius,
    btScalar          minTh,
    btScalar          maxTh,
    btScalar          minPs,
    btScalar          maxPs,
    const btVector3 & color,
    btScalar          stepDegrees
) {
    // TODO
}
void plBTDebugDraw::draw3dText      (
    const btVector3 & location,
    const char *      text
) {
    // TODO
}

// semantic primitives
void plBTDebugDraw::drawAabb (
    const btVector3 & from,
    const btVector3 & to,
    const btVector3 & color
) {
    // TODO
}
void plBTDebugDraw::drawContactPoint (
    const btVector3 & pointOnB,
    const btVector3 & normalOnB,
    btScalar          distance,
    int               lifeTime,
    const btVector3 & color
) {
    // TODO
}
void plBTDebugDraw::drawTransform (
    const btTransform & transform,
    btScalar            orthoLen
) {
    // TODO
}

// 
void plBTDebugDraw::setDebugMode (int mode);
int  plBTDebugDraw::getDebugMode () const;
void plBTDebugDraw::reportErrorWarning (const char * msg);
