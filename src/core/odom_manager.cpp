#include "trajlo/core/odom_manager.h"

namespace traj {

void OdomManager::mainloop() {
  while(!odom_process_down) {
    odometry->Start();
    if(dataset) {
      dataset->laser_queue = &odometry->laser_data_queue;
    }
    dataset->start();
    odom_process_down = odometry->isFinish;
  }
  std::cout << "main loop break!" << std::endl;
}

}