#include "plPhysical/plSimDefs.h"
#include "pnKeyedObject/plKey.h"

class plPhysicalControllerCore;
class btVector3;

struct plBtDefs {
    
    static const unsigned short kGrpExclude = 0x1000;
    
    struct ObjectData {
        virtual plKey               GetObjKey () const = 0;
        virtual plSimDefs::plLOSDB  GetLOSDBs () const = 0;
        virtual bool                IsSeeking () const = 0;
        virtual void OnHit (const ObjectData &, const btVector3 & normal) const = 0;
    };
};

