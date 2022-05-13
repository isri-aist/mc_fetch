#pragma once

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI FetchRobotModule : public mc_rbdyn::RobotModule
{
public:
  FetchRobotModule();

protected:
  void setupCommon();
  void findConvexes(const std::string &convexPath);

public:
  std::vector<std::string> virtualLinks;
  std::vector<std::string> gripperLinks;
};

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"Fetch"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("Fetch")
    if(n == "Fetch")
    {
      return new mc_robots::FetchRobotModule();
    }
    else
    {
      mc_rtc::log::error("Fetch module Cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
