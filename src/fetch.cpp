#include "fetch.h"
#include "config.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/constants.h>
#include <RBDyn/parsers/urdf.h>
#include <boost/algorithm/string.hpp>

namespace mc_robots
{

FetchRobotModule::FetchRobotModule() : RobotModule(FETCH_DESCRIPTION_PATH, "fetch")
{
  bool fixed = false;
  // rsdf_dir = path + "/rsdf"; // TODO: no rsdf files
  urdf_path = path + "/robots/" + FETCH_NAME + ".urdf";
  _real_urdf = urdf_path;
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));
  setupCommon();
  // findConvexes(path + "/convex/"+ FETCH_NAME); // TODO: no convex files
}

void FetchRobotModule::setupCommon()
{
  using namespace mc_rtc::constants;
  _stance["torso_lift_joint"]     = {toRad(3.0)       }; // TODO: different from fetcheus
  _stance["shoulder_pan_joint"]   = {toRad(75.6304)   };
  _stance["shoulder_lift_joint"]  = {toRad(80.2141)   };
  _stance["upperarm_roll_joint"]  = {toRad(-11.4592)  };
  _stance["elbow_flex_joint"]     = {toRad(98.5487)   };
  _stance["forearm_roll_joint"]   = {toRad(0.0)       };
  _stance["wrist_flex_joint"]     = {toRad(95.111)    };
  _stance["wrist_roll_joint"]     = {toRad(0.0)       };
  _stance["hed_pan_joint"]        = {toRad(0.0)       };
  _stance["head_tilt_joint"]      = {toRad(0.0)       };

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
  };

  // TODO: define force sensors
}

void FetchRobotModule::findConvexes(const std::string &convexPath)
{
  // Build _convexHull
  for(const auto & b : mb.bodies())
  {
    auto ch = bfs::path{convexPath} / (b.name() + "_mesh-ch.txt");
    std::cout << ch.string() << std::endl;
    if(bfs::exists(ch))
    {
      _convexHull[b.name()] = {b.name(), ch.string()};
    }
  }
}

} // namespace mc_robots
