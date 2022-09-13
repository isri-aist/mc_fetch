#include "fetch.h"
#include "config.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/constants.h>
#include <RBDyn/parsers/urdf.h>
#include <boost/algorithm/string.hpp>

namespace mc_robots
{

  // TODO: fetch_description is apt package and does not contain convex and rsdf files.
  //       Therefore, currently we have separete package (mc_fetch_description) for rsdf and convex files.
  FetchRobotModule::FetchRobotModule() : 
    RobotModule(MC_FETCH_DESCRIPTION_PATH, "fetch",
                std::string(FETCH_DESCRIPTION_PATH) + "/robots/" + std::string(FETCH_NAME) + ".urdf") // RobotModule can receive urdf_path separately
  {
    bool fixed = false;
    rsdf_dir = path + "/rsdf";
    _real_urdf = urdf_path;
    init(rbd::parsers::from_urdf_file(urdf_path, fixed));
    setupCommon();
    findConvexes(path + "/convex/"+ FETCH_NAME);
  }

  void FetchRobotModule::setupCommon()
  {
    using namespace mc_rtc::constants;
    _stance["torso_lift_joint"]       = {toRad(3.0)       };
    _stance["shoulder_pan_joint"]     = {toRad(75.6304)   };
    _stance["shoulder_lift_joint"]    = {toRad(80.2141)   };
    _stance["upperarm_roll_joint"]    = {toRad(-18.0)     };
    _stance["elbow_flex_joint"]       = {toRad(98.5487)   };
    _stance["forearm_roll_joint"]     = {toRad(0.0)       };
    _stance["wrist_flex_joint"]       = {toRad(95.111)    };
    _stance["wrist_roll_joint"]       = {toRad(0.0)       };
    _stance["head_pan_joint"]         = {toRad(0.0)       };
    _stance["head_tilt_joint"]        = {toRad(0.0)       };
    _stance["l_gripper_finger_joint"] = {0.04             }; // finger links are from 0 to 0.05
    _stance["r_gripper_finger_joint"] = {0.04             };

    _ref_joint_order = {
      "torso_lift_joint",
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "upperarm_roll_joint",
      "elbow_flex_joint",
      "forearm_roll_joint",
      "wrist_flex_joint",
      "wrist_roll_joint",
      "head_pan_joint",
      "head_tilt_joint",
      "l_gripper_finger_joint",
      "r_gripper_finger_joint"
    };

    _minimalSelfCollisions = {
      mc_rbdyn::Collision("base_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("base_link", "elbow_flex_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("base_link", "forearm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("base_link", "wrist_flex_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("base_link", "wrist_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("base_link", "gripper_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("base_link", "r_gripper_finger_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("base_link", "l_gripper_finger_link", 0.01, 0.001, 0.),

      mc_rbdyn::Collision("torso_lift_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("torso_lift_link", "elbow_flex_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("torso_lift_link", "forearm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("torso_lift_link", "wrist_flex_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("torso_lift_link", "wrist_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("torso_lift_link", "gripper_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("torso_lift_link", "r_gripper_finger_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("torso_lift_link", "l_gripper_finger_link", 0.01, 0.001, 0.),

      mc_rbdyn::Collision("head_pan_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_pan_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_pan_link", "elbow_flex_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_pan_link", "forearm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_pan_link", "wrist_flex_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_pan_link", "wrist_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_pan_link", "gripper_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_pan_link", "r_gripper_finger_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_pan_link", "l_gripper_finger_link", 0.01, 0.001, 0.),

      mc_rbdyn::Collision("head_tilt_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_tilt_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_tilt_link", "elbow_flex_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_tilt_link", "forearm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_tilt_link", "wrist_flex_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_tilt_link", "wrist_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_tilt_link", "gripper_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_tilt_link", "r_gripper_finger_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("head_tilt_link", "l_gripper_finger_link", 0.01, 0.001, 0.),

      mc_rbdyn::Collision("l_gripper_finger_link", "shoulder_pan_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("l_gripper_finger_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("l_gripper_finger_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("l_gripper_finger_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("l_gripper_finger_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("l_gripper_finger_link", "elbow_flex_link", 0.01, 0.001, 0.),

      mc_rbdyn::Collision("r_gripper_finger_link", "shoulder_pan_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("r_gripper_finger_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("r_gripper_finger_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("r_gripper_finger_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("r_gripper_finger_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("r_gripper_finger_link", "elbow_flex_link", 0.01, 0.001, 0.),

      mc_rbdyn::Collision("gripper_link", "shoulder_pan_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("gripper_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("gripper_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("gripper_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("gripper_link", "upperarm_roll_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("gripper_link", "elbow_flex_link", 0.01, 0.001, 0.),

      mc_rbdyn::Collision("wrist_roll_link", "shoulder_pan_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("wrist_roll_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("wrist_roll_link", "upperarm_roll_link", 0.01, 0.001, 0.),

      mc_rbdyn::Collision("wrist_flex_link", "shoulder_pan_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("wrist_flex_link", "shoulder_lift_link", 0.01, 0.001, 0.),
      mc_rbdyn::Collision("wrist_flex_link", "upperarm_roll_link", 0.01, 0.001, 0.)
    };

    // define grippers
    _gripperSafety = {0.15, 0.01, 0.002};
    _grippers = {{"gripper", {"l_gripper_finger_joint", "r_gripper_finger_joint"}, false}};

    _bodySensors.emplace_back("MobileBase", "base_link", sva::PTransformd::Identity()); // in order to update base pose from odometry
  }

  void FetchRobotModule::findConvexes(const std::string &convexPath)
  {
    // Build _convexHull
    for(const auto & b : mb.bodies())
    {
      auto ch = bfs::path{convexPath} / (b.name() + "-ch.txt");
      std::cout << ch.string() << std::endl;
      if(bfs::exists(ch))
      {
        _convexHull[b.name()] = {b.name(), ch.string()};
      }
    }
  }

} // namespace mc_robots
