cmake_minimum_required(VERSION 3.2...3.27)
project(git-version-tracking
    LANGUAGES CXX)

# Define the two required variables before including
# the source code for watching a git repository.
set(PRE_CONFIGURE_FILE "git.cpp.in")
set(POST_CONFIGURE_FILE "${CMAKE_CURRENT_BINARY_DIR}/git.cpp")
include(git-version-tracking.cmake)

# Create a library out of the compiled post-configure file.
#
# Note that the include is a system include. This was done
# so downstream projects don't suffer from warnings on a
# 3rdparty library.
add_library(${PROJECT_NAME} STATIC ${POST_CONFIGURE_FILE})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_dependencies(${PROJECT_NAME} check_git)
