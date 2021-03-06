cmake_minimum_required(VERSION 3.13)

project(osimplay VERSION 0.0.1)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_CXX_EXTENSIONS OFF)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BUILD_TYPE}-opensim-install/lib/cmake")

find_package(OpenSim REQUIRED)

add_library(common INTERFACE)
target_link_libraries(common INTERFACE
    osimLepton
    osimCommon
    osimSimulation
    osimActuators
    osimAnalyses
    osimTools
    SimTKcommon
    SimTKmath
    SimTKsimbody
)
target_compile_options(common INTERFACE
    # disable MSVC permissiveness. Forces MSVC to obey C++ standard
    # - specifically, this enables the use of 'and', 'not', etc.
    $<$<CXX_COMPILER_ID:MSVC>:/permissive->
)
target_compile_features(common INTERFACE
    cxx_std_20
)


add_executable(OpenSimPartyDemoCable
    src/OpenSimPartyDemoCable.cpp
    src/ak_wrapcylinder.hpp
)
target_link_libraries(OpenSimPartyDemoCable PUBLIC common)


add_executable(expt_wrap src/expt_wrap.cpp)
target_link_libraries(expt_wrap PUBLIC common)

add_executable(size_of_objects src/size_of_objects.cpp)
target_link_libraries(size_of_objects PUBLIC common)

add_executable(study_simbody_4_pendulum src/study_simbody_4_pendulum.cpp)
target_link_libraries(study_simbody_4_pendulum PUBLIC common)

add_executable(opensim_wrapping src/opensim_wrapping.cpp)
target_link_libraries(opensim_wrapping PRIVATE common)

add_executable(simbody_wrapping src/simbody_wrapping.cpp)
target_link_libraries(simbody_wrapping PRIVATE common)

add_executable(fd src/fd.cpp)
target_link_libraries(fd PRIVATE common)

add_executable(joris2 src/joris2.cpp)
target_link_libraries(joris2 PRIVATE common)

add_executable(joris2_simbody src/joris2_simbody.cpp)
target_link_libraries(joris2_simbody PRIVATE common)

# BUILD LIB PATHS:
#
# in Windows, copy libraries to the exe dir, because Windows specifically
# searches in the exe's dir for libraries. see:
#
# https://stackoverflow.com/questions/44909846/cmake-exe-cant-find-dll
if (WIN32)
    file(GLOB OPENSIM_LIBS LIST_DIRECTORIES FALSE "${OpenSim_ROOT_DIR}/bin/*.dll")
    foreach(OPENSIM_LIB ${OPENSIM_LIBS})
        add_custom_command(
            TARGET common
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENSIM_LIB}" "$<TARGET_FILE_DIR:osim-snippets>")
    endforeach()
endif()
