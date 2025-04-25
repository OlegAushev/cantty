cmake_minimum_required(VERSION 3.12)
project(cantty VERSION 0.1)

################################################################################
# Threads
# set(CMAKE_THREAD_PREFER_PTHREAD ON)
# set(THREADS_PREFER_PTHREAD_FLAG ON)
# find_package(Threads REQUIRED)

################################################################################
# Headers and Sources
set(app_incdirs
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)

file(GLOB_RECURSE app_sources CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.c
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

################################################################################
# FTXUI
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED TRUE)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG v6.0.2
)
FetchContent_MakeAvailable(ftxui)

################################################################################
# Dear ImGUI
# find_package(glfw3 REQUIRED)
# find_package(OpenGL REQUIRED)

# file(GLOB imgui_sources CONFIGURE_DEPENDS
#     ${PROJECT_SOURCE_DIR}/include/imgui/*.cpp
#     ${PROJECT_SOURCE_DIR}/include/implot/*.cpp
#     ${PROJECT_SOURCE_DIR}/include/imguifiledialog/*.cpp
# )
# file(GLOB imgui_backend_sources CONFIGURE_DEPENDS
#     ${PROJECT_SOURCE_DIR}/include/imgui/backends/*glfw*.cpp
#     ${PROJECT_SOURCE_DIR}/include/imgui/backends/*opengl3*.cpp
# )

# set(imgui_incdirs
#     ${PROJECT_SOURCE_DIR}/include/imgui
#     ${PROJECT_SOURCE_DIR}/include/implot
# )

################################################################################
# Executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(executable ${CMAKE_PROJECT_NAME})

add_executable(${executable}
    ${app_sources}
)

target_compile_definitions(${executable} PRIVATE
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(${executable} PRIVATE
    ${app_incdirs}
)

target_link_libraries(${executable} PRIVATE
    PRIVATE ftxui::screen
    PRIVATE ftxui::dom
    PRIVATE ftxui::component
    atomic
    # Threads::Threads
    glfw
    ${OPENGL_LIBRARIES}
)

################################################################################
# Compiler Options and Features
target_compile_options(${executable} PRIVATE
    $<$<CONFIG:Debug>:-Og -ggdb>
    $<$<CONFIG:Release>:-O3 -g0>
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wconversion
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wuseless-cast
    -Wdouble-promotion
    -Wformat=2
    -Wimplicit-fallthrough
    -fdiagnostics-color=always
)

target_compile_features(${executable} PRIVATE
    c_std_11
    cxx_std_23
)

################################################################################
# Git Version Tracking
add_subdirectory(cmake/git-version-tracking)

target_link_directories(${executable} PRIVATE
    ${CMAKE_BINARY_DIR}/cmake/git-version-tracking
)

target_link_libraries(${executable} PRIVATE
    git-version-tracking
)

################################################################################
# Misc Options
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
