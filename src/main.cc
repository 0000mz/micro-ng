#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
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
// clang-format off
#include "fmt/core.h"
#define SPDLOG_FMT_EXTERNAL
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

namespace {

class streamable_logger_t {
  spdlog::level::level_enum log_level_;
  std::ostringstream log_stream_;

public:
  explicit streamable_logger_t(const spdlog::level::level_enum log_level)
      : log_level_(log_level) {}

  template <typename T> streamable_logger_t &operator<<(const T &value) {
    log_stream_ << value;
    return *this;
  }

  ~streamable_logger_t() { spdlog::log(log_level_, "{}", log_stream_.str()); }
};

// loguru
#define LOG_S(log_level) streamable_logger_t(spdlog::level::log_level)
#define LOG(log_level) LOG_S(log_level)

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
      LOG(err) << "Parsing failed: " << err;
      return std::nullopt;
    }

    const toml::table *objects_table = tbl["assets"].as<toml::table>();
    if (!objects_table) {
      LOG(err) << "Could not find any assets in the config.";
      return std::nullopt;
    }

    AssetLibrary lib;
    lib.asset_dir_ = config_path.parent_path();
    for (const auto &object : *objects_table) {
      const toml::key &object_name = object.first;
      SPDLOG_INFO(fmt::format("Loading object: {}", std::string(object_name)));
      const toml::table *objinfo = object.second.as<toml::table>();
      if (!objinfo) {
        LOG(err) << "Error: No object info found.";
        continue;
      }
      if (!lib.ParseObjectInfoFromTOML(object_name, *objinfo)) {
        LOG(err) << "Error: Failed to parse asset object information for "
                    "object name: "
                 << object_name;
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
      LOG(err) << "No path found for object: " << object_key;
      return false;
    }

    AssetInfo info;
    info.absolute_path = asset_dir_ / object_path.value();
    info.resource_relative_path = object_path.value();

    if (!std::filesystem::exists(info.absolute_path)) {
      LOG(err) << "Error: Asset at path " << info.absolute_path
               << " does not exist.";
      return false;
    }
    if (std::filesystem::is_directory(info.absolute_path)) {
      LOG(err) << "Error: Asset at path " << info.absolute_path
               << "is a directory.";
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
  void ParseCameraInfoFromTOML(std::string_view camera_name,
                               const toml::table &camera_info,
                               Ogre::SceneManager &scene,
                               OgreBites::ApplicationContext &appctx);

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

void Scene::ParseCameraInfoFromTOML(std::string_view camera_name,
                                    const toml::table &camera_info,
                                    Ogre::SceneManager &scene,
                                    OgreBites::ApplicationContext &appctx) {
  Ogre::Vector3 position;
  TOML_PARSE_VECTOR3(camera_info, "position", position, Ogre::Vector3(0, 0, 0));
  std::optional<bool> auto_aspect_ratio =
      camera_info["auto_aspect_ratio"].value<bool>();
  std::optional<double> near_clip_distance =
      camera_info["near_clip_distance"].value<double>();

  Ogre::SceneNode *cam_node = scene.getRootSceneNode()->createChildSceneNode();
  Ogre::Camera *cam =
      scene.createCamera(std::string(camera_name.data(), camera_name.size()));
  cam->setNearClipDistance(near_clip_distance.has_value() ? *near_clip_distance
                                                          : 1);
  cam->setAutoAspectRatio(auto_aspect_ratio.has_value() ? *auto_aspect_ratio
                                                        : false);
  cam_node->attachObject(cam);
  cam_node->setPosition(position);
  appctx.getRenderWindow()->addViewport(cam);
}

void Scene::ParseEntityInfoFromTOML(const toml::table &entity_info,
                                    Ogre::SceneManager &scene,
                                    const AssetLibrary &assetlib) {
  Ogre::Vector3 position, scale;

  TOML_PARSE_VECTOR3(entity_info, "position", position, Ogre::Vector3(0, 0, 0));
  TOML_PARSE_VECTOR3(entity_info, "scale", scale, Ogre::Vector3(1, 1, 1));

  std::optional<std::string_view> mesh_name =
      entity_info["mesh"].value<std::string_view>();
  if (!mesh_name.has_value()) {
    LOG(err) << "Error: Could not get mesh name from entity info.";
    return;
  }

  const AssetLibrary::AssetInfo *asset_info = assetlib.GetAsset(*mesh_name);
  if (!asset_info) {
    LOG(err) << "Error: Could not find asset from asset library with name: "
             << *mesh_name;
    return;
  }
  std::filesystem::path resource_relative_path_as_fspath(
      asset_info->resource_relative_path);
  SPDLOG_INFO(
      fmt::format("Resource relative path: {}, filename: {}",
                  std::string(asset_info->resource_relative_path),
                  std::string(resource_relative_path_as_fspath.filename())));

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

  SPDLOG_INFO(
      fmt::format("Reading scene config from file: {}", std::string(config_)));
  toml::table scene_tbl;
  try {
    scene_tbl = toml::parse_file(std::string(config_));
  } catch (const toml::parse_error &err) {
    LOG(err) << "Error: Failed to parse scene toml: " << err;
    return;
  }

  const toml::table *cameras_table = scene_tbl["cameras"].as<toml::table>();
  if (!cameras_table) {
    LOG(err) << "Could not find any cameras in the scene config.";
    return;
  }
  for (const auto &camera : *cameras_table) {
    const toml::key &camera_name = camera.first;
    SPDLOG_INFO(fmt::format("Loading camera: {}", std::string(camera_name)));
    const toml::table *camerainfo = camera.second.as<toml::table>();
    if (!camerainfo) {
      LOG(err) << "Error: No camera info found.";
      continue;
    }
    ParseCameraInfoFromTOML(camera_name, *camerainfo, *scn_manager, *appctx);
  }

  const toml::table *entities_table = scene_tbl["entities"].as<toml::table>();
  if (!entities_table) {
    LOG(err) << "Could not find any entities in the scene config.";
    return;
  }

  for (const auto &object : *entities_table) {
    const toml::key &object_name = object.first;
    SPDLOG_INFO(fmt::format("Loading entity: {}", std::string(object_name)));
    const toml::table *objinfo = object.second.as<toml::table>();
    if (!objinfo) {
      LOG(err) << "Error: No object info found.";
      continue;
    }
    ParseEntityInfoFromTOML(*objinfo, *scn_manager, assetlib);
  }
  SPDLOG_INFO("Finished loading entities for scene.");
}

enum ConfigStrictness { UNKNOWN = 0, REQUIRED, OPTIONAL };

#define LOAD_TOML_OPTION2(TOML_FIELD, OUTFIELD, TYPE, STRICTNESS)              \
  {                                                                            \
    bool __should_set_value = true;                                            \
    std::optional<TYPE> value = TOML_FIELD.value<TYPE>();                      \
    switch (STRICTNESS) {                                                      \
    case ConfigStrictness::REQUIRED: {                                         \
      if (!value.has_value()) {                                                \
        LOG(err) << "Failed to parse toml settings: " << #TOML_FIELD;          \
        return std::nullopt;                                                   \
      }                                                                        \
    } break;                                                                   \
    case ConfigStrictness::OPTIONAL: {                                         \
      if (!value.has_value()) {                                                \
        SPDLOG_INFO("Ignoring unset field: " #TOML_FIELD);                     \
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
      LOG(err) << "Parsing failed: " << err;
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
    LOG(err) << "Failed to find scenes directlry for project.";
    return;
  }

  for (const auto &entry : std::filesystem::directory_iterator(scenes_path)) {
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
      LOG(warn) << "duplicate scene name: " << scene_name;
      continue;
    }
    Scene scene(scene_config_path);
    scenes_.insert({scene_name, scene});
    SPDLOG_INFO(fmt::format("Adding new scene: {}", scene_name));
  }
}

void Game::LoadStartingScene() {
  const std::string starting_scene(cfg_.StartingScene().data(),
                                   cfg_.StartingScene().size());
  SPDLOG_INFO(fmt::format("Loading starting scene: {}", starting_scene));
  auto starting_scene_itr = scenes_.find(starting_scene);
  if (starting_scene_itr == scenes_.end()) {
    LOG(err) << "Error: starting scene does not exist: " << starting_scene;
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
  SPDLOG_INFO("Loading GL renderer.");
  getRoot()->loadPlugin("/home/mob/Projects/roguelike/build/_deps/ogre-build/"
                        "lib/RenderSystem_GL");
  SPDLOG_INFO("Finished loading GL renderer.");
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
  resg_manager.addResourceLocation(assetlib_.GetAssetDirectory(), "FileSystem");

  ParseScenes();
  LoadStartingScene();
}

void RunGameProject(const std::filesystem::path project_path) {
  // Parse the project toml file.
  const std::filesystem::path project_toml_path = project_path / "project.toml";
  if (!std::filesystem::exists(project_toml_path)) {
    LOG(err) << "Error: project.toml not found at " << project_toml_path;
    return;
  }

  std::optional<GameConfig> game_config =
      GameConfig::FromTOML(project_toml_path);
  if (!game_config.has_value()) {
    LOG(err) << "Error: Failed to parse game config.";
    return;
  }

  const std::filesystem::path asset_library_path =
      project_path / "assets" / "assets.toml";
  std::optional<AssetLibrary> asset_lib =
      AssetLibrary::FromTOML(asset_library_path);
  if (!asset_lib.has_value()) {
    LOG(err) << "Error: Could not parse asset library config.";
    return;
  }

  try {
    auto game = Game(game_config.value(), asset_lib.value(), project_path);
    game.initApp();
    game.getRoot()->startRendering();
    game.closeApp();
  } catch (const std::exception &e) {
    LOG(err) << "Error occurred during game execution: " << e.what();
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
    LOG(err) << "Error: Invalid project path. Path does not exist.";
    return 1;
  }
  if (!std::filesystem::is_directory(full_project_path)) {
    LOG(err) << "Error: Project path is not a directory.";
    return 1;
  }
  RunGameProject(full_project_path);
  return 0;
}
