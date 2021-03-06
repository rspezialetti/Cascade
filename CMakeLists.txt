cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(CASCADE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")

#finde OPENCV
find_package( OpenCV REQUIRED )

#Find OpennI
find_package(OpenNI2)
if(OpenNI2_FOUND)
	include_directories(${OpenNI2_INCLUDE_DIRS})
	link_directories(${OpenNI2_LIBRARY})
	add_definitions(${OpeNI2_DEFINITIONS})
endif(OpenNI2_FOUND)

#find NITE
find_package(Nite2)
if(Nite2_FOUND)
	include_directories(${NITE2_INCLUDE_DIRS})
	link_directories(${NITE2_LIBRARY})
	add_definitions(${NITE2_DEFINITIONS})
endif(Nite2_FOUND)

#finde BOOST
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
	link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)

set(HEADER	 ${PROJECT_SOURCE_DIR}/include/calibration.h
			 ${PROJECT_SOURCE_DIR}/include/utils.h
			 ${PROJECT_SOURCE_DIR}/include/UDPClient.hpp
			 ${PROJECT_SOURCE_DIR}/include/coordinates_mapper.h)

set(SOURCES  ${PROJECT_SOURCE_DIR}/src/main_cascade.cpp
			 ${PROJECT_SOURCE_DIR}/src/calibration.cpp
			  ${PROJECT_SOURCE_DIR}/src/utils.cpp
			  ${PROJECT_SOURCE_DIR}/src/coordinates_mapper.cpp )

add_executable(cascade ${SOURCES} ${HEADER})
include_directories(${PROJECT_SOURCE_DIR}/include/)
target_link_libraries(cascade ${OpenNI2_LIBRARY})
target_link_libraries(cascade ${NITE2_LIBRARY})
target_link_libraries(cascade ${OpenCV_LIBS})
target_link_libraries(cascade ${Boost_LIBRARY_DIRS})