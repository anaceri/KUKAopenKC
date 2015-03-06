project(Kuka)
cmake_minimum_required(VERSION 2.8)

file(GLOB_RECURSE SRCS src/*.cpp)
file(GLOB_RECURSE HDRS src/*.h src/*.hpp)
#Djallil setup
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DCBF_NDEBUG -DDJALLIL_CONF")
#grasp lab setup
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DCBF_NDEBUG")

# libopenkc
set(CORE_ROOT_DIR CORE_ROOT_DIR CACHE PATH "path to the core libs")
set(EIGEN_DIR EIGEN_DIR CACHE PATH "path to the eigen libs")
set(KUKA_CONTROLLER_DIR KUKA_CONTROLLER_DIR CACHE PATH "path to the kuka controller libs")

set(CORE_LIBS "-L${CORE_ROOT_DIR}/lib -Wl,-rpath=${CORE_ROOT_DIR}/lib -lopenkcfri -lorocos-kdl -lcbf -pthread")
set(CORE_INCLUDES "${CORE_ROOT_DIR}/include")
set(CBF_INCLUDES "${CORE_INCLUDES}/cbf0.2")
set(EIGEN3_INCLUDES "${EIGEN_DIR}")
include_directories("${CORE_INCLUDES}")
include_directories("${CBF_INCLUDES}")
include_directories("${EIGEN3_INCLUDES}")
include_directories("${KUKA_CONTROLLER_DIR}/src")

aux_source_directory(src SRC_LIST)
add_executable(${PROJECT_NAME} app/kukamove.cpp ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${CORE_LIBS})
