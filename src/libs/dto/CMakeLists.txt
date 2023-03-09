cmake_minimum_required(VERSION 3.5)

project(dto)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
    GLOB
    CRL_SOURCES #
    "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h" #
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

option(CXX "enable C++ compilation" ON)
find_package(KNITRO REQUIRED)
#find_package(CHOLMOD REQUIRED)

# basic dependencies
list(
    APPEND
    CRL_TARGET_DEPENDENCIES #
    "frame"
    "search"
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
     PUBLIC "${KNITRO_INCLUDE_DIRS}"
     #PUBLIC "${CHOLMOD_INCLUDES}"
)

# target link libs
list(
    APPEND
    CRL_TARGET_LINK_LIBS #
    PUBLIC
    "frame"
    PUBLIC
    "search"
    PUBLIC
    ${Boost_LIBRARIES}
    PUBLIC
    ${KNITRO_LIBRARIES}
    #PUBLIC
    #${CHOLMOD_LIBRARIES}
)

# compile definitions
list(APPEND 
    CRL_COMPILE_DEFINITIONS #
     ""
)

# create target
create_crl_library(
    ${CRL_TARGET_NAME}
    "${CRL_SOURCES}" #
    "${CRL_TARGET_DEPENDENCIES}" #
    "${CRL_TARGET_INCLUDE_DIRS}" #
    "${CRL_TARGET_LINK_LIBS}" #
    "${CRL_COMPILE_DEFINITIONS}"
)
