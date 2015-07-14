if(DirectX_INCLUDE_DIR)
    set(DirectX_FIND_QUIETLY TRUE)
endif()

# Figure out the arch for the path suffixes
if(CMAKE_SIZEOF_VOID_P STREQUAL "8")
    set(_dxarch "x64")
else()
    set(_dxarch "x86")
endif()


find_path(DirectX_INCLUDE_DIR d3dx9.h
          PATHS "$ENV{DXSDK_DIR}/Include"
)

find_library(DirectX_d3d9 NAMES d3d9
             PATHS "$ENV{DXSDK_DIR}/Lib/${_dxarch}"
)

find_library(DirectX_d3dx9 NAMES d3dx9
             PATHS "$ENV{DXSDK_DIR}/Lib/${_dxarch}"
)

find_library(DirectX_dinput8 NAMES dinput8
             PATHS "$ENV{DXSDK_DIR}/Lib/${_dxarch}"
)

find_library(DirectX_dsound NAMES dsound
             PATHS "$ENV{DXSDK_DIR}/Lib/${_dxarch}"
)

find_library(DirectX_dxguid NAMES dxguid
             PATHS "$ENV{DXSDK_DIR}/Lib/${_dxarch}"
)

find_library(DirectX_dxerr NAMES dxerr9 DxErr
             PATHS "$ENV{DXSDK_DIR}/Lib/${_dxarch}"
)

set(DirectX_LIBRARIES
    ${DirectX_d3d9}
    ${DirectX_d3dx9}
    ${DirectX_dinput8}
    ${DirectX_dsound}
    ${DirectX_dxguid}
    ${DirectX_dxerr}
)


if(DirectX_INCLUDE_DIR AND DirectX_d3d9 AND DirectX_d3dx9 AND DirectX_dinput8
                       AND DirectX_dsound AND DirectX_dxguid AND DirectX_dxerr)
    set(DirectX_FOUND TRUE)
    
    if(DirectX_dxerr MATCHES ".*dxerr9.*")
        if(NOT DirectX_FIND_QUIETLY)
            message(STATUS "Found old DirectX SDK: ${DirectX_INCLUDE_DIR}")
        endif()
        add_definitions(-DDX_OLD_SDK)
    elseif(NOT DirectX_FIND_QUIETLY)
        message(STATUS "Found DirectX SDK: ${DirectX_INCLUDE_DIR}")
    endif()
elseif(DirectX_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find DirectX SDK")
endif()
