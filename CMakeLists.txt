SET(PROJECT_NAME PhotometricDatasetCalibration)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

set(BUILD_TYPE Release)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS_RELEASE} ${SSE_FLAGS} -std=c++0x /openmp /Ox /fp:strict"
)

include_directories(
	${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)


add_executable(responseCalib src/main_responseCalib.cpp src/EquiUndistorter.cpp src/PhotometricUndistorter.cpp)
target_link_libraries(responseCalib ${OpenCV_LIBS})

add_executable(playDataset src/main_playbackDataset.cpp src/EquiUndistorter.cpp src/PhotometricUndistorter.cpp)
target_link_libraries(playDataset ${OpenCV_LIBS})

find_package(Aruco)

IF(Aruco_FOUND)
	include_directories(
		${EIGEN3_INCLUDE_DIR}
		${Boost_INCLUDE_DIRS}
		${ARUCO_INCLUDE_DIRS}
	)
	add_executable(vignetteCalib src/main_vignetteCalib.cpp src/EquiUndistorter.cpp src/PhotometricUndistorter.cpp)
	target_link_libraries(vignetteCalib ${OpenCV_LIBS} ${ARUCO_LIBRARIES})
ELSE()
	message("================ aruco not found. not compiling vignetteCalib. ========================")
ENDIF()















