include_directories("../../CoreLib")
include_directories("../../NucleusLib/inc")
include_directories("../../NucleusLib")
include_directories("../../PubUtilLib")

include_directories(${BULLET_INCLUDE_DIRS})

set(plBullet_SOURCES
    plPhysicalImpl.cpp
    plSimulationMgrImpl.cpp
)

set(plBullet_HEADERS
    plPhysicalImpl.h
    plSimulationMgrImpl.h
)

if (NOT USE_PHYSX)
    add_library(plBullet STATIC ${plBullet_SOURCES} ${plBullet_HEADERS})
endif ()

source_group("Source Files" FILES ${plBullet_SOURCES})
source_group("Header Files" FILES ${plBullet_HEADERS})
