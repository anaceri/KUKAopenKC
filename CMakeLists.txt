project(Kuka)
cmake_minimum_required(VERSION 2.8)

file(GLOB_RECURSE SRCS src/*.cpp)
file(GLOB_RECURSE HDRS src/*.h src/*.hpp)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DCBF_NDEBUG")

# libopenkc
set(OPENKCFRI_ROOT_DIR OPENKCFRI_ROOT_DIR CACHE PATH "path to the openkc libs")

set(CORE_LIBS "-L${OPENKCFRI_ROOT_DIR}/lib -Wl,-rpath=${OPENKCFRI_ROOT_DIR}/lib -lopenkcfri -lorocos-kdl -lcbf -pthread")
set(CORE_INCLUDES "${OPENKCFRI_ROOT_DIR}/include")
include_directories(${CORE_INCLUDES})
include_directories("~/SWLib/eigen3/")
include_directories("/usr/local/include/cbf0.2")
include_directories("/usr/local/include")
include_directories("~/KukaCtrlSrc/src")

aux_source_directory(src SRC_LIST)
add_executable(${PROJECT_NAME} app/main.cpp ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${CORE_LIBS})
