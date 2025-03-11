cmake_minimum_required(VERSION 3.16)
project(tros_base)

#---1. setting complie environment---#
SET(CMAKE_CXX_FLAGS "-g -O3 -Wall -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O3 -Wall -W -fprofile-arcs -ftest-coverage")
SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")

add_compile_options(-std=c++17)

#---2. find 3rd packages---#
find_package(glog 0.6.0 REQUIRED)
message(STATUS "glog include dirs: ${glog_INCLUDE_DIRS}")
message(STATUS "glog libraries: ${glog_LIBRARIES}")

find_package(GTest REQUIRED)
message(STATUS "GTest include dirs: ${GTEST_INCLUDE_DIRS}")
message(STATUS "GTest libraries: ${GTEST_LIBRARIES}")

find_package(yaml-cpp REQUIRED)
message(STATUS "yaml-cpp include dirs: ${yaml-cpp_INCLUDE_DIRS}")
message(STATUS "yaml-cpp libraries: ${yaml-cpp_LIBRARIES}")

find_package(TBB REQUIRED)
message(STATUS "TBB include dirs: ${TBB_INCLUDE_DIRS}")
message(STATUS "TBB libraries: ${TBB_LIBRARIES}")

find_package(Eigen3 3.3.0 REQUIRED)
message(STATUS "Eigen3 include dirs: ${EIGEN3_INCLUDE_DIR}")

find_package(Doxygen)
if(DOXYGEN_FOUND)
  message(STATUS "Doxygen executable: ${DOXYGEN_EXECUTABLE}")
endif()

find_package(Protobuf REQUIRED)
message(STATUS "Protobuf include dirs: ${Protobuf_INCLUDE_DIRS}")
message(STATUS "Protobuf libraries: ${Protobuf_LIBRARIES}")

find_package(PkgConfig)
pkg_check_modules(ZMQ REQUIRED libzmq)
message(STATUS "ZeroMQ include dirs: ${ZMQ_INCLUDE_DIRS}")
message(STATUS "ZeroMQ libraries: ${ZMQ_LIBRARIES}")

#---3. set include directories and lib---#

#---3.1 generate pre-generate files---#
file(GLOB_RECURSE PROTO_FILES ${CMAKE_SOURCE_DIR}/include/*.proto)
message("Proto files: ${PROTO_FILES}")
foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(FILE_NAME ${PROTO_FILE} NAME_WE)  # Get the file name without extension
    get_filename_component(DIR_NAME ${PROTO_FILE} DIRECTORY)  # Get the directory of the .proto file
    message("file name: ${FILE_NAME}")
    message("dir name: ${DIR_NAME}")

    set(OUTPUT_INCLUDE_DIR ${DIR_NAME})
    string(REPLACE "include/tros_base" "src" OUTPUT_SRC_DIR ${DIR_NAME})
    message("include files: ${OUTPUT_INCLUDE_DIR}")
    message("src files: ${OUTPUT_SRC_DIR}")


    # Generate the protobuf files
    execute_process(
        COMMAND protoc --cpp_out=${OUTPUT_INCLUDE_DIR} -I ${DIR_NAME} ${PROTO_FILE}
        RESULT_VARIABLE PROTOC_RESULT
        OUTPUT_VARIABLE PROTOC_OUTPUT
        ERROR_VARIABLE PROTOC_ERROR
    )

    list(APPEND PROTO_SRCS ${OUTPUT_INCLUDE_DIR}/${FILE_NAME}.pb.cc)
endforeach()

#---3.2 set include directories and lib---#
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${ZMQ_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)
set(LIBS
  glog::glog
  yaml-cpp
  TBB::tbb
  pthread
  Eigen3::Eigen
  ${ZMQ_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  /usr/local/lib/libzmqpp.so
)

set(TEST_LIB
  gtest_main
  gtest
  ${LIBS}
)

#---4. add shared lib---#
file(GLOB_RECURSE SRCS src/*.cc)
add_library(
  ${PROJECT_NAME} SHARED
  ${SRCS}
  ${PROTO_SRCS}
)
target_link_libraries(${PROJECT_NAME} ${LIBS})

#---5. build unit tests---#
enable_testing()

function(AddTest dir_path use_gtest)
  file(GLOB_RECURSE TEST_CC_FILES ${dir_path}/*.cc)
  foreach(ABS_PATH ${TEST_CC_FILES})
    get_filename_component(EXE_NAME ${ABS_PATH} NAME_WE)
    add_executable(${EXE_NAME} ${ABS_PATH})
    if(${use_gtest})
      target_link_libraries(${EXE_NAME} ${TEST_LIB} ${PROJECT_NAME})
      gtest_discover_tests(${EXE_NAME})
    else()
      target_link_libraries(${EXE_NAME} ${LIBS} ${PROJECT_NAME})
    endif()
  endforeach()
endfunction(AddTest)

AddTest(tests ON)

#---6. build mock tests---#
AddTest(mock OFF)

#---7. add generate doxygen doc command---#
set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_GENERATE_MAN NO)
doxygen_add_docs(
    doxygen
    ${PROJECT_SOURCE_DIR}
    COMMENT "Generate doxygen pages"
)

#---8. add easy command---#
add_custom_target(coverage
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/coverage
    COMMAND make -j 8
    COMMAND make test
    COMMAND lcov --capture --directory . --output-file ${CMAKE_BINARY_DIR}/coverage/coverage.info
    COMMAND lcov --remove ${CMAKE_BINARY_DIR}/coverage/coverage.info '/usr/*' --output-file ${CMAKE_BINARY_DIR}/coverage/coverage.info
    COMMAND lcov --remove ${CMAKE_BINARY_DIR}/coverage/coverage.info '${CMAKE_SOURCE_DIR}/include/tros_base/proto/*' --output-file ${CMAKE_BINARY_DIR}/coverage/coverage.info
    COMMAND genhtml --prefix ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR}/coverage/coverage.info --output-directory ${CMAKE_BINARY_DIR}/coverage_report
    COMMENT "Generating code coverage report"
)

#---8. pack to build/---#
add_custom_target(copy_include ALL
     COMMAND ${CMAKE_COMMAND} -E copy_directory
             ${CMAKE_SOURCE_DIR}/include/ ${CMAKE_BINARY_DIR}/include/
)

install(TARGETS tros_base EXPORT tros_baseTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(EXPORT tros_baseTargets
    FILE tros_baseConfig.cmake
    NAMESPACE tros_base::
    DESTINATION lib/cmake/tros_base
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ 
        DESTINATION include
)