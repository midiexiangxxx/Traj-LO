#pragma once 


#include <tbb/concurrent_queue.h>
#include <trajlo/gui/shader.h>
#include <trajlo/utils/common_type.h>
#include <memory>

#include <implot.h>
#include <trajlo/gui/camera.h>

#include <trajlo/core/odometry.h>
#include <trajlo/io/data_loader.h>
#include <trajlo/utils/config.h>

namespace traj {


class OdomManager {

 private:

  bool load_dataset = false;

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  // opengl shader
  Shader::Ptr shader_trajectory;
  Shader::Ptr shader_pointcloud;
  Shader::Ptr shader_current;

  // opengl buffer
  //  static const int VB_COUNT = 3;
  //  static const int VA_COUNT = 2;
  static const int TEXTURE_COUNT =
      4;  // 0 for range image , 1 for render texture  . when record 0 for image
          // 2 for scan

  static const int VB_COUNT = 5;
  static const int VA_COUNT = 5;

  // trajectory
  std::vector<float> trajectory;
  std::vector<float> pointcloud;
  std::vector<float> current_pc;

  size_t buffer_size = 0;
  const size_t buffer_chunk = 100000000;
  //  const size_t buffer_chunk=1200000000;
  size_t last_points_size = 0;

  //  float3 lidarPos;
  glm::mat4 lidarPos = glm::mat4(.0f);
  glm::vec3 eye = glm::vec3(0.0f);
  glm::vec3 target = glm::vec3(0.0f);

  // data
  TrajLOdometry::Ptr odometry;
  DataLoader::Ptr dataset;
  TrajConfig config;

  bool odom_process_down = false;
  
public:
  using Ptr = std::shared_ptr<OdomManager>;
  OdomManager(TrajLOdometry::Ptr odometry, TrajConfig& config,
             DataLoader::Ptr dataset, int width = 1280, int height = 720)
      : odometry(odometry),
        config(config),
        dataset(dataset) {
    odometry->vis_data_queue = &vis_data_queue;
    vis_data_queue.set_capacity(30);
  }
  void mainloop();
  tbb::concurrent_bounded_queue<traj::ScanVisData::Ptr> vis_data_queue;
};
}