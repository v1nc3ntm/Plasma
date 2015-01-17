#include "plPhysical/plSimDefs.h"
#include "pnKeyedObject/plKey.h"

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

class plPhysicalControllerCore;

struct plBtDefs {
    
    static const unsigned short kGrpExclude = 0x1000;
    
    struct ObjectData {
        virtual plKey               GetObjKey () const = 0;
        virtual plSimDefs::plLOSDB  GetLOSDBs () const = 0;
        virtual bool                IsSeeking () const = 0;
    };
};

