cmake_minimum_required(VERSION 3.30)

project(GameEngine CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OGRE_BUILD_RENDERSYSTEM_GL True)

include(FetchContent)
FetchContent_Declare(
  ogre
  GIT_REPOSITORY        https://github.com/OGRECave/ogre
  GIT_TAG               v14.2.6
  CONFIGURE_COMMAND     "cmake -GNinja -DOGRE_BUILD_RENDERSYSTEM_GL=True"
  OVERRIDE_FIND_PACKAGE
)
FetchContent_Declare(
  tomlplusplus
  GIT_REPOSITORY        https://github.com/marzer/tomlplusplus.git
  GIT_TAG               v3.4.0
  OVERRIDE_FIND_PACKAGE
)
FetchContent_Declare(
  CLI11
  GIT_REPOSITORY        https://github.com/CLIUtils/CLI11.git
  GIT_TAG               v2.4.2
  OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(ogre tomlplusplus CLI11)

add_executable(game src/main.cc)
target_link_libraries(game PRIVATE
  OgreMain
  OgreBites
  OgreRTShaderSystem
  RenderSystem_GL
  tomlplusplus::tomlplusplus
  CLI11)
