#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/rendering/rendering.hh>
#include <chrono>
#include <filesystem>
#include <vector>
#include <fstream>
#include <cstdlib>

namespace gazebo
{
  class TextureChanger : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      this->world = _world;

      // 画像ディレクトリのパス
      std::string imageDirectory = "/home/ciero/catkin_ws/src/nav_cloning/models/changing_model/my_image"; 
      this->imageFiles = this->GetImageFiles(imageDirectory);

      // 初期時間を設定
      this->lastUpdateTime = std::chrono::steady_clock::now();

      // 更新コネクションの設定
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&TextureChanger::OnUpdate, this));
    }

  private:
    void OnUpdate()
    {
      auto currentTime = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsedTime = currentTime - lastUpdateTime;

      if (elapsedTime.count() >= 0.5) // 0.5秒ごとに更新
      {
        std::string selectedImage = this->GetRandomImage();
        UpdateGazeboTexture(selectedImage);
        lastUpdateTime = currentTime; // 更新時間をリセット
      }
    }

    void UpdateGazeboTexture(const std::string &imagePath)
    {
      std::string textureFilePath = "/home/ciero/catkin_ws/src/nav_cloning/models/changing_model/models/colorfull_box/materials/scripts/my_textures/noise_image.png";
      std::filesystem::copy(imagePath, textureFilePath, std::filesystem::copy_options::overwrite_existing);

      // マテリアルファイルの更新
      std::string materialFilePath = "/home/ciero/catkin_ws/src/nav_cloning/models/changing_model/models/colorfull_box/materials/scripts/colorfull_box.material";
      std::ofstream materialFile(materialFilePath);
      materialFile << "material colorfull_boxTexture\n{\n  technique\n  {\n    pass\n    {\n      texture_unit\n      {\n        texture textures/my_texture.png\n        scale 1 1\n      }\n    }\n  }\n}\n";
      materialFile.close();
    }

    std::vector<std::string> GetImageFiles(const std::string &directory)
    {
      std::vector<std::string> images;
      for (const auto &entry : std::filesystem::directory_iterator(directory))
      {
        if (entry.path().extension() == ".png")
          images.push_back(entry.path().string());
      }
      return images;
    }

    std::string GetRandomImage()
    {
      if (this->imageFiles.empty()) return "";
      return this->imageFiles[rand() % this->imageFiles.size()];
    }

    physics::WorldPtr world;
    std::vector<std::string> imageFiles;
    std::chrono::steady_clock::time_point lastUpdateTime; // セミコロンを追加
    event::ConnectionPtr updateConnection; // 追加
  }; // ここでクラスの終了を明示

  GZ_REGISTER_WORLD_PLUGIN(TextureChanger)
}

