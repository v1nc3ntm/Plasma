include_directories("../../CoreLib")
include_directories("../../NucleusLib/inc")
include_directories("../../NucleusLib")
include_directories("../../PubUtilLib")

include_directories(${BULLET_INCLUDE_DIRS})

set(plBullet_SOURCES
    plBTPhysical.cpp
    plSimulationMgr.cpp
)

set(plBullet_HEADERS
    plBTPhysical.h
    plSimulationMgr.h
)

add_library(plBullet STATIC ${plBullet_SOURCES} ${plBullet_HEADERS})

source_group("Source Files" FILES ${plBullet_SOURCES})
source_group("Header Files" FILES ${plBullet_HEADERS})
