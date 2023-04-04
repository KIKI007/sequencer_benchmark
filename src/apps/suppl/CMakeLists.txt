cmake_minimum_required(VERSION 3.5)

project(supplApp)

file(GLOB CRL_SOURCES #
     "${CMAKE_CURRENT_SOURCE_DIR}/*.h" #
     "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" #
)

#find_package(GUROBI REQUIRED)

list(
    APPEND
    CRL_TARGET_DEPENDENCIES #
    "crl::frame" #
    "crl::search"
    "crl::dto"
    "crl::algorithms"
)

list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
     # PUBLIC "${GUROBI_INCLUDE_DIRS}"
)

list(APPEND CRL_TARGET_LINK_LIBS #
    PUBLIC
    "crl::frame" #
    "crl::search"
    "crl::dto"
    "crl::algorithms"
)

list(APPEND CRL_COMPILE_DEFINITIONS #
     PUBLIC "CRL_DATA_FOLDER=${CRL_DATA_FOLDER}" #
     PUBLIC "ROBOCRAFT_DATA_FOLDER=${ROBOCRAFT_DATA_FOLDER}" #
)

create_crl_app(
    ${PROJECT_NAME}
    "${CRL_SOURCES}" #
    "${CRL_TARGET_DEPENDENCIES}" #
    "${CRL_TARGET_INCLUDE_DIRS}" #
    "${CRL_TARGET_LINK_LIBS}" #
    "${CRL_COMPILE_DEFINITIONS}"
)
