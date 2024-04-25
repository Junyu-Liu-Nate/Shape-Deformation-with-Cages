cmake_minimum_required(VERSION 3.16)

# Sets project name
project(green_coordinates LANGUAGES CXX C)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Sets C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specifies required Qt components
find_package(Qt6 REQUIRED COMPONENTS Core)
find_package(Qt6 REQUIRED COMPONENTS Concurrent)
find_package(Qt6 REQUIRED COMPONENTS Xml)
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(Qt6 REQUIRED COMPONENTS OpenGL)
find_package(Qt6 REQUIRED COMPONENTS OpenGLWidgets)
find_package(Qt6 REQUIRED COMPONENTS Gui)

# Allows you to include files from within those directories, without prefixing their filepaths
include_directories(src)
include_directories(libs)

# Specifies .cpp and .h files to be passed to the compiler
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mainwindow.cpp
    src/cage3d.cpp
    src/glwidget3d.cpp
    src/graphics/camera.cpp
    src/graphics/graphicsdebug.cpp
    src/graphics/meshloader.cpp
    src/graphics/shader.cpp
    src/graphics/shape.cpp

    src/mainwindow.h
    src/cage3d.h
    src/glwidget3d.h
    src/graphics/camera.h
    src/graphics/graphicsdebug.h
    src/graphics/meshloader.h
    src/graphics/shader.h
    src/graphics/shape.h

    util/tiny_obj_loader.h
    util/unsupportedeigenthing/OpenGLSupport
    src/mesh_struct/halfedgemesh.h src/mesh_struct/halfedgemesh.cpp
    src/coordinates/greencoordinates3d.h src/coordinates/greencoordinates3d.cpp
    src/mesh_struct/halfedgemeshobject.h src/mesh_struct/halfedgemeshobject.cpp
    src/object3d.h src/object3d.cpp
    src/coordinates/mvc3d.h src/coordinates/mvc3d.cpp
    src/coordinates/greencoordinates2d.h src/coordinates/greencoordinates2d.cpp
    src/object2d.h src/object2d.cpp
    src/cage2d.h src/cage2d.cpp
<<<<<<< HEAD
    src/coordinates/mvc2d.h src/coordinates/mvc2d.cpp
=======

    src/coordinates/gchigherorder2d.h src/coordinates/gchigherorder2d.cpp
    src/mesh_struct/margincage2d.h src/mesh_struct/margincage2d.cpp
    src/coordinates/mvc2d.h src/coordinates/mvc2d.cpp

    src/glwidget2d.h src/glwidget2d.cpp

>>>>>>> 3D-Green-Coord
)

# GLEW: this creates its library and allows you to `#include "GL/glew.h"`
add_library(StaticGLEW STATIC glew/src/glew.c)
include_directories(${PROJECT_NAME} PRIVATE glew/include)

# Specifies libraries to be linked (Qt components, glew, etc)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Concurrent
    Qt::Core
    Qt::Gui
    Qt::OpenGL
    Qt::OpenGLWidgets
    Qt::Widgets
    Qt::Xml
    StaticGLEW
)

# This allows you to `#include "Eigen/..."`
target_include_directories(${PROJECT_NAME} PRIVATE
    Eigen
)

# Specifies other files
qt6_add_resources(${PROJECT_NAME} "Resources"
    PREFIX
        "/"
    FILES
        resources/shaders/shader.frag
        resources/shaders/shader.vert
        resources/shaders/anchorPoint.vert
        resources/shaders/anchorPoint.geom
        resources/shaders/anchorPoint.frag
        resources/shaders/texture.frag
        resources/shaders/texture.vert
)

# GLEW: this provides support for Windows (including 64-bit)
if (WIN32)
  add_compile_definitions(GLEW_STATIC)
  target_link_libraries(${PROJECT_NAME} PRIVATE
    opengl32
    glu32
  )
endif()

# Set this flag to silence warnings on Windows
if (MSVC OR MSYS OR MINGW)
  set(CMAKE_CXX_FLAGS "-Wno-volatile")
endif()

# Set this flag to silence warnings on MacOS
if (APPLE)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-volatile")
endif()
