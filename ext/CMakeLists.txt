# FetchContent command is available with cmake >= 3.11
include(FetchContent)

# -----------------------------------------------------------------------------
# helper funtion to download a 'fetch content declare'
# -----------------------------------------------------------------------------
macro(fetch what)
    FetchContent_GetProperties("${what}")
    if(NOT ${${what}_POPULATED})
        message(STATUS "fetching ${what} ...")
        FetchContent_Populate(${what})
    endif()
    mark_as_advanced(${${what}_SOURCE_DIR})
endmacro()


#------------------------------------------------------------------------------
# json
FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.2
)
fetch(json)
add_subdirectory(${json_SOURCE_DIR} json)

# tbb
FetchContent_Declare(
        tbb
        GIT_REPOSITORY https://github.com/wjakob/tbb.git
        GIT_TAG tbb44u4
)
fetch(tbb)
add_subdirectory(${tbb_SOURCE_DIR} tbb)
#------------------------------------------------------------------------------
# libigl
FetchContent_Declare(
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG        v2.4.0
)
fetch(libigl)
add_subdirectory(${libigl_SOURCE_DIR} libigl)
