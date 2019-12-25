#ifndef FLEA3_STEREO_NODE_H_
#define FLEA3_STEREO_NODE_H_

#include "flea3/flea3_ros.h"
#include "flea3/Flea3DynConfig.h"
#include "camera_base/camera_node_base.h"

namespace flea3 {

class StereoNode : public camera_base::CameraNodeBase<Flea3DynConfig> {
 public:
  explicit StereoNode(const ros::NodeHandle &pnh);

  virtual void Acquire() override;
  virtual void Setup(Flea3DynConfig &config) override;

 private:
  Flea3Ros left_ros_;
  Flea3Ros right_ros_;
};

}  // namespace flea3

#endif  // FLEA3_STEREO_NODE_H_
