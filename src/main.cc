#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "CLI/CLI.hpp"
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreRTShaderSystem.h"
#include "OgreRoot.h"
#include "toml++/toml.hpp"

namespace {

class AssetLibrary {
public:
  struct AssetInfo {
    std::filesystem::path absolute_path;
    std::filesystem::path resource_relative_path;
  };

  static std::optional<AssetLibrary>
  FromTOML(const std::filesystem::path &config_path) {
    toml::table tbl;
    try {
      tbl = toml::parse_file(std::string(config_path));
    } catch (const toml::parse_error &err) {
      std::cerr << "Parsing failed: " << err << std::endl;
      return std::nullopt;
    }

    const toml::table *objects_table = tbl["assets"].as<toml::table>();
    if (!objects_table) {
      std::cerr << "Could not find any assets in the config." << std::endl;
      return std::nullopt;
    }

    AssetLibrary lib;
    lib.asset_dir_ = config_path.parent_path();
    for (const auto &object : *objects_table) {
      const toml::key &object_name = object.first;
      std::cout << "Loading object: " << object_name << std::endl;
      const toml::table *objinfo = object.second.as<toml::table>();
      if (!objinfo) {
        std::cerr << "Error: No object info found." << std::endl;
        continue;
      }
      if (!lib.ParseObjectInfoFromTOML(object_name, *objinfo)) {
        std::cerr << "Error: Failed to parse asset object information for "
                     "object name: "
                  << object_name << std::endl;
        return std::nullopt;
      }
    }
    return lib;
  }

  const AssetInfo *GetAsset(std::string_view asset_name) const {
    const auto &asset =
        assets_.find(std::string(asset_name.data(), asset_name.size()));
    if (asset == assets_.end()) {
      return nullptr;
    }
    return &(asset->second);
  }

  std::filesystem::path GetAssetDirectory() const { return asset_dir_; }

private:
  bool ParseObjectInfoFromTOML(std::string_view object_key,
                               const toml::table &obj_info) {
    std::optional<std::string_view> object_path =
        obj_info["path"].value<std::string_view>();
    if (!object_path.has_value()) {
      std::cerr << "No path found for object: " << object_key << std::endl;
      return false;
    }

    AssetInfo info;
    info.absolute_path = asset_dir_ / object_path.value();
    info.resource_relative_path = object_path.value();

    if (!std::filesystem::exists(info.absolute_path)) {
      std::cerr << "Error: Asset at path " << info.absolute_path
                << " does not exist." << std::endl;
      return false;
    }
    if (std::filesystem::is_directory(info.absolute_path)) {
      std::cerr << "Error: Asset at path " << info.absolute_path
                << "is a directory." << std::endl;
      return false;
    }
    assets_.insert({std::string(object_key), std::move(info)});
    return true;
  }

  std::unordered_map<std::string, AssetInfo> assets_;
  std::filesystem::path asset_dir_;
};

using ::Ogre::Root;
using ::Ogre::SceneManager;
using ::OgreBites::ApplicationContext;

// Function attribute macro for identifying which functions are overloaded from
// the base Ogre class. The base Ogre classes do not use virtual overloads,
// so this should improve readability.
#define OGRE_OVERLOAD

class Scene {
public:
  Scene(const std::filesystem::path &config) : config_(config) {}

  void Load(OgreBites::ApplicationContext *appctx,
            const AssetLibrary &assetlib);

private:
  void ParseEntityInfoFromTOML(const toml::table &entity_info,
                               Ogre::SceneManager &scene,
                               const AssetLibrary &assetlib);

  std::filesystem::path config_;
};

#define TOML_PARSE_VECTOR3(TABLE, KEY, OUTVEC, DEFAULT)                        \
  do {                                                                         \
    double parts[3];                                                           \
    parts[0] = DEFAULT[0];                                                     \
    parts[1] = DEFAULT[1];                                                     \
    parts[2] = DEFAULT[2];                                                     \
    const toml::array *arr = TABLE[KEY].as<toml::array>();                     \
    if (arr) {                                                                 \
      for (int i = 0; i < 3; ++i) {                                            \
        const toml::node *component = arr->get(i);                             \
        if (component == nullptr)                                              \
          break;                                                               \
        const std::optional<double> value = component->value<double>();        \
        if (!value.has_value())                                                \
          continue;                                                            \
        parts[i] = *value;                                                     \
      }                                                                        \
    }                                                                          \
    OUTVEC = Ogre::Vector3(parts[0], parts[1], parts[2]);                      \
  } while (false);

void Scene::ParseEntityInfoFromTOML(const toml::table &entity_info,
                                    Ogre::SceneManager &scene,
                                    const AssetLibrary &assetlib) {
  std::cout << "Inside ParseEntityInfoFromTOML" << std::endl;
  Ogre::Vector3 position(0, 0, 0);
  Ogre::Vector3 scale(1, 1, 1);

  TOML_PARSE_VECTOR3(entity_info, "position", position, Ogre::Vector3(0, 0, 0));
  TOML_PARSE_VECTOR3(entity_info, "scale", scale, Ogre::Vector3(1, 1, 1));

  std::optional<std::string_view> mesh_name =
      entity_info["mesh"].value<std::string_view>();
  if (!mesh_name.has_value()) {
    std::cerr << "Error: Could not get mesh name from entity info."
              << std::endl;
    return;
  }

  const AssetLibrary::AssetInfo *asset_info = assetlib.GetAsset(*mesh_name);
  if (!asset_info) {
    std::cerr << "Error: Could not find asset from asset library with name: "
              << *mesh_name << std::endl;
    return;
  }
  std::filesystem::path resource_relative_path_as_fspath(
      asset_info->resource_relative_path);
  std::cout << "Resource relative path: " << asset_info->resource_relative_path
            << ", filename: " << resource_relative_path_as_fspath.filename();

  Ogre::Entity *entity =
      scene.createEntity(resource_relative_path_as_fspath.filename());
  Ogre::SceneNode *node = scene.getRootSceneNode()->createChildSceneNode();
  node->setPosition(position);
  node->setScale(scale);
  node->attachObject(entity);
}

void Scene::Load(OgreBites::ApplicationContext *appctx,
                 const AssetLibrary &assetlib) {
  Root *root = appctx->getRoot();
  SceneManager *scn_manager = root->createSceneManager();

  Ogre::RTShader::ShaderGenerator *shadergen =
      Ogre::RTShader::ShaderGenerator::getSingletonPtr();
  shadergen->addSceneManager(scn_manager);

  using ColorValue = Ogre::ColourValue;
  scn_manager->setAmbientLight(ColorValue(0.5, 0.5, 0.5));

  Ogre::Light *light = scn_manager->createLight("MainLight");
  Ogre::SceneNode *light_node =
      scn_manager->getRootSceneNode()->createChildSceneNode();
  light_node->attachObject(light);

  light_node->setPosition(20, 80, 50);

  // TODO: Specify the camera information in the scene config.
  Ogre::SceneNode *cam_node =
      scn_manager->getRootSceneNode()->createChildSceneNode();
  // camera
  {
    Ogre::Camera *cam = scn_manager->createCamera("myCam");
    cam->setNearClipDistance(5);
    cam->setAutoAspectRatio(true);
    cam_node->attachObject(cam);
    cam_node->setPosition(0, 0, 140);
    appctx->getRenderWindow()->addViewport(cam);
  }

  std::cout << "Reading scene config from file: " << config_ << std::endl;
  toml::table scene_tbl;
  try {
    scene_tbl = toml::parse_file(std::string(config_));
  } catch (const toml::parse_error &err) {
    std::cerr << "Error: Failed to parse scene toml: " << err << std::endl;
    return;
  }

  const toml::table *entities_table = scene_tbl["entities"].as<toml::table>();
  if (!entities_table) {
    std::cerr << "Could not find any entities in the config." << std::endl;
    return;
  }

  for (const auto &object : *entities_table) {
    const toml::key &object_name = object.first;
    std::cout << "Loading object: " << object_name << std::endl;
    const toml::table *objinfo = object.second.as<toml::table>();
    if (!objinfo) {
      std::cerr << "Error: No object info found." << std::endl;
      continue;
    }
    ParseEntityInfoFromTOML(*objinfo, *scn_manager, assetlib);
  }
  std::cout << "Finished loading entities for scene." << std::endl;
}

enum ConfigStrictness { UNKNOWN = 0, REQUIRED, OPTIONAL };

#define LOAD_TOML_OPTION2(TOML_FIELD, OUTFIELD, TYPE, STRICTNESS)              \
  {                                                                            \
    bool __should_set_value = true;                                            \
    std::optional<TYPE> value = TOML_FIELD.value<TYPE>();                      \
    switch (STRICTNESS) {                                                      \
    case ConfigStrictness::REQUIRED: {                                         \
      if (!value.has_value()) {                                                \
        std::cerr << "Failed to parse toml settings: " << #TOML_FIELD          \
                  << std::endl;                                                \
        return std::nullopt;                                                   \
      }                                                                        \
    } break;                                                                   \
    case ConfigStrictness::OPTIONAL: {                                         \
      if (!value.has_value()) {                                                \
        std::cout << "Ignoring unset field: " << #TOML_FIELD << std::endl;     \
        __should_set_value = false;                                            \
      }                                                                        \
    } break;                                                                   \
    }                                                                          \
    if (__should_set_value)                                                    \
      OUTFIELD = value.value();                                                \
  }

#define LOAD_TOML_OPTION(TOML_FIELD, OUTFIELD, TYPE)                           \
  LOAD_TOML_OPTION2(TOML_FIELD, OUTFIELD, TYPE, ConfigStrictness::REQUIRED)

class GameConfig {

public:
  static std::optional<GameConfig>
  FromTOML(const std::filesystem::path &config_path) {
    GameConfig cfg;

    toml::table tbl;
    try {
      tbl = toml::parse_file(std::string(config_path));
    } catch (const toml::parse_error &err) {
      std::cerr << "Parsing failed: " << err << std::endl;
      return std::nullopt;
    }

    // clang-format off
    LOAD_TOML_OPTION (tbl["game-info"]["title"],       cfg.game_name_,       std::string_view);
    LOAD_TOML_OPTION (tbl["game-info"]["start_scene"], cfg.starting_scene_,  std::string_view);
    LOAD_TOML_OPTION2(tbl["window"]["width"],          cfg.width_,           int,               ConfigStrictness::OPTIONAL);
    LOAD_TOML_OPTION2(tbl["window"]["height"],         cfg.height_,          int,               ConfigStrictness::OPTIONAL);
    LOAD_TOML_OPTION2(tbl["window"]["fullscreen"],     cfg.fullscreen_,      bool,              ConfigStrictness::OPTIONAL);
    // clang-format on
    return cfg;
  }

  std::string_view Name() const { return game_name_; }
  std::string_view StartingScene() const { return starting_scene_; }
  bool IsValid() const;
  int Width() const { return width_; }
  int Height() const { return height_; }
  bool Fullscreen() const { return fullscreen_; }

private:
  std::string game_name_;
  std::string starting_scene_;
  int width_ = 1080;
  int height_ = 720;
  bool fullscreen_ = false;
};

class Game : public ApplicationContext {
public:
  Game(const GameConfig &cfg, const AssetLibrary &assetlib,
       std::filesystem::path project_path)
      : ApplicationContext(std::string(cfg.Name().data(), cfg.Name().size())),
        project_path_(project_path), cfg_(cfg), assetlib_(assetlib) {}
  virtual ~Game() = default;

  OGRE_OVERLOAD void createRoot();
  OGRE_OVERLOAD void initApp();
  OGRE_OVERLOAD void setup();

private:
  void ParseScenes();
  void LoadStartingScene();

  std::filesystem::path project_path_;
  std::unordered_map<std::string, Scene> scenes_;
  const GameConfig &cfg_;
  const AssetLibrary &assetlib_;
};

void Game::ParseScenes() {
  const std::filesystem::path scenes_path = project_path_ / "scenes";
  if (!std::filesystem::exists(scenes_path)) {
    std::cerr << "Failed to find scenes directlry for project." << std::endl;
    return;
  }

  for (const auto &entry : std::filesystem::directory_iterator(scenes_path)) {
    // TODO: Parse the scene.
    const std::filesystem::path &path = entry.path();
    if (!std::filesystem::is_directory(path)) {
      continue;
    }
    const std::filesystem::path &scene_config_path = path / "scene.toml";
    if (!std::filesystem::exists(scene_config_path) ||
        std::filesystem::is_directory(scene_config_path)) {
      continue;
    }

    const std::string &scene_name = path.filename();
    assert(!scene_name.empty());
    if (scenes_.find(scene_name) != scenes_.end()) {
      std::cerr << "WARN: duplicate scene name: " << scene_name;
      continue;
    }
    Scene scene(scene_config_path);
    scenes_.insert({scene_name, scene});
    std::cout << "Added new scene: " << scene_name << std::endl;
  }
}

void Game::LoadStartingScene() {
  const std::string starting_scene(cfg_.StartingScene().data(),
                                   cfg_.StartingScene().size());
  std::cout << "Loading starting scene: " << starting_scene << std::endl;
  auto starting_scene_itr = scenes_.find(starting_scene);
  if (starting_scene_itr == scenes_.end()) {
    std::cerr << "Error: starting scene does not exist: " << starting_scene;
    return;
  }
  starting_scene_itr->second.Load(this, assetlib_);
}

void Game::createRoot() {
  if (mRoot) {
    return;
  }
  ApplicationContext::createRoot();
}

void Game::initApp() {
  createRoot();
  std::cout << "Loading GL renderer." << std::endl;
  getRoot()->loadPlugin("/home/mob/Projects/roguelike/build/_deps/ogre-build/"
                        "lib/RenderSystem_GL");
  std::cout << "Finished loading GL renderer." << std::endl;
  ApplicationContext::initApp();
}

void Game::setup() {
  // -- Block context: start
  // Copy of ApplicationContext::setup() to set window.
  mRoot->initialise(false);
  createWindow(mAppName, cfg_.Width(), cfg_.Height());
  if (cfg_.Fullscreen()) {
    getRenderWindow()->setFullscreen(cfg_.Fullscreen(), cfg_.Width(),
                                     cfg_.Height());
  }
  locateResources();
  initialiseRTShaderSystem();
  loadResources();
  mRoot->addFrameListener(this);
  // -- Block context: end

  // Load Resources
  Ogre::ResourceGroupManager &resg_manager =
      Ogre::ResourceGroupManager::getSingleton();
  // TODO: Remove commented out code.
  resg_manager.addResourceLocation(assetlib_.GetAssetDirectory(), "FileSystem");

  ParseScenes();
  LoadStartingScene();
}

void RunGameProject(const std::filesystem::path project_path) {
  // Parse the project toml file.
  const std::filesystem::path project_toml_path = project_path / "project.toml";
  if (!std::filesystem::exists(project_toml_path)) {
    std::cerr << "Error: project.toml not found at " << project_toml_path
              << std::endl;
    return;
  }

  std::optional<GameConfig> game_config =
      GameConfig::FromTOML(project_toml_path);
  if (!game_config.has_value()) {
    std::cerr << "Error: Failed to parse game config." << std::endl;
    return;
  }

  const std::filesystem::path asset_library_path =
      project_path / "assets" / "assets.toml";
  std::optional<AssetLibrary> asset_lib =
      AssetLibrary::FromTOML(asset_library_path);
  if (!asset_lib.has_value()) {
    std::cerr << "Error: Could not parse asset library config." << std::endl;
    return;
  }

  try {
    auto game = Game(game_config.value(), asset_lib.value(), project_path);
    game.initApp();
    game.getRoot()->startRendering();
    game.closeApp();
  } catch (const std::exception &e) {
    std::cerr << "Error occurred during game execution: " << e.what()
              << std::endl;
    return;
  }
}

} // namespace

int main(int argc, char **argv) {
  CLI::App app("Game Engine");

  std::string project_path = ".";
  app.add_option("-p", project_path, "Path to the game project.");
  CLI11_PARSE(app, argc, argv);

  // TODO: If an absolute path is provided, this should not be done.
  // This might not even be necessary, should let the filesystem api determine
  // if it's a relative path or an absolute path.
  const std::filesystem::path current_path = std::filesystem::current_path();
  const std::filesystem::path full_project_path = current_path / project_path;
  if (!std::filesystem::exists(full_project_path)) {
    std::cerr << "Error: Invalid project path. Path does not exist."
              << std::endl;
    return 1;
  }
  if (!std::filesystem::is_directory(full_project_path)) {
    std::cerr << "Error: Project path is not a directory." << std::endl;
    return 1;
  }
  RunGameProject(full_project_path);
  return 0;
}
