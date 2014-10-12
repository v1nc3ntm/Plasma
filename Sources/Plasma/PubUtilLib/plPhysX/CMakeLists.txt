include_directories("../../CoreLib")
include_directories("../../NucleusLib/inc")
include_directories("../../NucleusLib")
include_directories("../../PubUtilLib")

include_directories(${PHYSX_INCLUDE_DIRS})

set(plPhysX_SOURCES
    plLOSDispatch.cpp
    plPXConvert.cpp
    plPXPhysical.cpp
    plPXPhysicalControllerCore.cpp
    plSimulationMgrImpl.cpp
)

set(plPhysX_HEADERS
    plLOSDispatch.h
    plPhysXCreatable.h
    plPXConvert.h
    plPXPhysical.h
    plPXPhysicalControllerCore.h
    plPXStream.h
    plSimulationMgrImpl.h
)

add_library(plPhysX STATIC ${plPhysX_SOURCES} ${plPhysX_HEADERS})

source_group("Source Files" FILES ${plPhysX_SOURCES})
source_group("Header Files" FILES ${plPhysX_HEADERS})
