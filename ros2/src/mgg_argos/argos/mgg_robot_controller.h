/*
 * Base interface for robots driven by the MGG planner in ARGoS.
 *
 * Implemented by CMGGFootbot, CMGGBunkerMini, and any platform that accepts
 * waypoint paths from mgg_pci and reports positioning, lidar, odometry, and IMU
 * to the mgg_bridge loop functions.
 */

#ifndef MGG_ROBOT_CONTROLLER_H
#define MGG_ROBOT_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector3.h>

#include <string>
#include <vector>

namespace argos {
   class CCI_PositioningSensor;
   class CCI_PhotorealisticLidarSensor;
   class CCI_OdometrySensor;
   class CCI_IMUSensor;
}

using namespace argos;

class CMGGRobotController : public CCI_Controller {

public:

   virtual ~CMGGRobotController() {}

   virtual void SetPath(const std::vector<CVector3>& vec_waypoints) = 0;
   virtual void StopPath() = 0;
   virtual const std::vector<CVector3>& GetPath() const = 0;

   virtual size_t GetCurrentWaypointIndex() const = 0;
   virtual bool HasActivePath() const = 0;

   virtual const std::string& GetRobotId() const = 0;

   virtual CCI_PositioningSensor* GetPositioning() = 0;
   virtual CCI_PhotorealisticLidarSensor* GetLidar() = 0;
   virtual CCI_OdometrySensor* GetOdometry() = 0;
   virtual CCI_IMUSensor* GetIMU() = 0;

};

#endif
