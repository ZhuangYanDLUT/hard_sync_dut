#ifndef FLEA3_SINGLE_NODE_H_
#define FLEA3_SINGLE_NODE_H_

#include "flea3/flea3_ros.h"
#include "flea3/Flea3DynConfig.h"
#include <camera_base/camera_node_base.h>

namespace flea3 {

using Config = ::flea3::Flea3DynConfig;

class SingleNode : public camera_base::CameraNodeBase<Config> {
 public:
  explicit SingleNode(const ros::NodeHandle &pnh);

  void Acquire() override;
  void Setup(Config &config) override;

 private:
  Flea3Ros flea3_ros_;
};

}  // namespace flea3

#endif  // FLEA3_SINGLE_NODE_H_
