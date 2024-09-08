#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>

#include "CLI/CLI.hpp"
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreRTShaderSystem.h"
#include "OgreRoot.h"
#include "toml++/toml.hpp"

namespace {
// FIX: Do not hardcode this.
const char kBaseProjectPath[] = "/home/mob/Projects/roguelike";

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

  void Load(OgreBites::ApplicationContext *appctx);

private:
  std::filesystem::path config_;
};

void Scene::Load(OgreBites::ApplicationContext *appctx) {
  // TODO: Remove the rest of this function.
  Ogre::ResourceGroupManager &resg_manager =
      Ogre::ResourceGroupManager::getSingleton();
  resg_manager.addResourceLocation(kBaseProjectPath, "FileSystem");

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

  {
    // TODO: download the ogre mesh for this to work.
    Ogre::Entity *ogre_entity = scn_manager->createEntity("ogrehead.mesh");
    Ogre::SceneNode *ogre_node =
        scn_manager->getRootSceneNode()->createChildSceneNode();
    ogre_node->attachObject(ogre_entity);
    cam_node->setPosition(0, 47, 222);
  }

  {
    Ogre::Entity *ogre_entity2 = scn_manager->createEntity("ogrehead.mesh");
    Ogre::SceneNode *ogre_node2 =
        scn_manager->getRootSceneNode()->createChildSceneNode(
            Ogre::Vector3(84, 48, 0));
    ogre_node2->attachObject(ogre_entity2);
  }

  {
    Ogre::Entity *ogre_entity3 = scn_manager->createEntity("ogrehead.mesh");
    Ogre::SceneNode *ogre_node3 =
        scn_manager->getRootSceneNode()->createChildSceneNode();
    ogre_node3->setPosition(0, 104, 0);
    ogre_node3->setScale(2, 1.2, 1);
    ogre_node3->attachObject(ogre_entity3);
  }

  {
    Ogre::Entity *ogre_entity4 = scn_manager->createEntity("ogrehead.mesh");
    Ogre::SceneNode *ogre_node4 =
        scn_manager->getRootSceneNode()->createChildSceneNode();
    ogre_node4->setPosition(-84, 48, 0);
    ogre_node4->roll(Ogre::Degree(-90));
    ogre_node4->attachObject(ogre_entity4);
  }
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
  Game(const GameConfig &cfg, std::filesystem::path project_path)
      : ApplicationContext(std::string(cfg.Name().data(), cfg.Name().size())),
        project_path_(project_path), cfg_(cfg) {}
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
  starting_scene_itr->second.Load(this);
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

  try {
    auto game = Game(game_config.value(), project_path);
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
