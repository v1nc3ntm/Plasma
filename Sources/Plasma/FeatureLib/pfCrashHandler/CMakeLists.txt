include_directories(../../CoreLib)
include_directories(../../NucleusLib)
include_directories(../../PubUtilLib)

set(pfCrashHandler_HEADERS
    plCrashCli.h
    plCrashSrv.h
)

set(pfCrashHandler_win_HEADERS
    plCrash_WinPrivate.h
)

set(pfCrashHandler_win_SOURCES
    plCrashCli_Win.cpp
    plCrash_WinPrivate.cpp
    plCrashSrv_Win.cpp
)

set(pfCrashHandler_all_SOURCES
    plCrash_Null.cpp
)

if(WIN32)
    add_library(pfCrashHandler STATIC
        ${pfCrashHandler_win_SOURCES}
        ${pfCrashHandler_win_HEADERS}
        ${pfCrashHandler_HEADERS}
    )
else()
    message(WARNING "pfCrashHandler not implemented for this platform")
    add_library(pfCrashHandler STATIC
        ${pfCrashHandler_all_SOURCES}
        ${pfCrashHandler_HEADERS}
    )
endif()

source_group("Default Source Files" FILES ${pfCrashHandler_all_SOURCES})
source_group("Windows Source Files" FILES ${pfCrashHandler_win_SOURCES})
source_group("Windows Header Files" FILES ${pfCrashHandler_win_HEADERS})
source_group("Header Files" FILES ${pfCrashHandler_HEADERS})
