cmake_minimum_required(VERSION 3.16)
project(SoapyPlutoSky CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(SoapySDR CONFIG REQUIRED)
find_package(Threads REQUIRED)
# gRPC's package configuration finds protobuf itself, in config mode where
# protobuf installs one (Homebrew) and through FindProtobuf otherwise.
find_package(gRPC CONFIG REQUIRED)
if(NOT TARGET protobuf::libprotobuf)
    find_package(Protobuf REQUIRED)
endif()
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN)
    message(FATAL_ERROR "grpc_cpp_plugin not found")
endif()
if(TARGET protobuf::protoc)
    set(PLUTOSKY_PROTOC $<TARGET_FILE:protobuf::protoc>)
else()
    set(PLUTOSKY_PROTOC ${Protobuf_PROTOC_EXECUTABLE})
endif()

# The control service client, generated from the daemon's proto.
set(PLUTOSKY_PROTO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../proto"
    CACHE PATH "Directory holding plutosky/v1/control.proto")
set(PLUTOSKY_PROTO "${PLUTOSKY_PROTO_ROOT}/plutosky/v1/control.proto")
set(PLUTOSKY_GENERATED "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(PLUTOSKY_PROTO_SOURCES
    "${PLUTOSKY_GENERATED}/plutosky/v1/control.pb.cc"
    "${PLUTOSKY_GENERATED}/plutosky/v1/control.grpc.pb.cc")
add_custom_command(
    OUTPUT ${PLUTOSKY_PROTO_SOURCES}
        "${PLUTOSKY_GENERATED}/plutosky/v1/control.pb.h"
        "${PLUTOSKY_GENERATED}/plutosky/v1/control.grpc.pb.h"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PLUTOSKY_GENERATED}"
    COMMAND ${PLUTOSKY_PROTOC}
        --cpp_out=${PLUTOSKY_GENERATED}
        --grpc_out=${PLUTOSKY_GENERATED}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        -I ${PLUTOSKY_PROTO_ROOT}
        ${PLUTOSKY_PROTO}
    DEPENDS ${PLUTOSKY_PROTO}
    COMMENT "Generating the control service client")

add_compile_options(-Wall -Wextra)

set(PLUTOSKY_VECTORS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../tools/vectors"
    CACHE PATH "Directory holding the shared wire-format vectors")

# Protocol and wire-format code with no SoapySDR dependency, so the tests and
# the probe tool can link it directly.
add_library(plutosky_core STATIC
    src/Json.cpp
    src/Vita.cpp
    src/Convert.cpp
    src/Control.cpp
    ${PLUTOSKY_PROTO_SOURCES})
set_target_properties(plutosky_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(plutosky_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${PLUTOSKY_GENERATED}")
target_link_libraries(plutosky_core PUBLIC Threads::Threads gRPC::grpc++ protobuf::libprotobuf)

SOAPY_SDR_MODULE_UTIL(
    TARGET plutosky
    SOURCES
        src/Device.cpp
        src/Streaming.cpp
        src/Registration.cpp
    LIBRARIES plutosky_core)

add_executable(plutosky_probe tools/plutosky_probe.cpp)
target_link_libraries(plutosky_probe PRIVATE plutosky_core SoapySDR)
install(TARGETS plutosky_probe RUNTIME DESTINATION bin)

enable_testing()
add_subdirectory(tests)
