/*
 * An AgileX Bunker Mini tracked robot controller driven by the MGG planner.
 *
 * Implements pure-pursuit waypoint tracking for the Bunker Mini tracked chassis,
 * receiving paths over the MGG IPC bridge and streaming odometry, IMU, and
 * photorealistic lidar readings back to ROS 2.
 *
 * Params (in <params>):
 *   robot_id           default "r0"; the ROS namespace and bridge key
 *   waypoint_tolerance default 0.25 m (2D); Bunker Mini scale
 *   cruise_speed       default 40 (cm/s, both tracks when aligned; max ~150 cm/s)
 *   turn_speed         default 20 (cm/s, track speed for in-place turns)
 *   turn_gain          default 45 (cm/s of differential correction per
 *                      radian of heading error while cruising)
 *   align_threshold    default 35 (degrees: heading error above which the
 *                      robot turns in place instead of arcing)
 *   goal_tolerance     default 0.3 m; distance at which the final waypoint
 *                      counts as reached and the path is considered done
 */

#ifndef MGG_BUNKER_MINI_H
#define MGG_BUNKER_MINI_H

#include "mgg_robot_controller.h"

namespace argos {
   class CCI_DifferentialSteeringActuator;
   class CCI_PositioningSensor;
   class CCI_PhotorealisticLidarSensor;
   class CCI_IMUSensor;
   class CCI_OdometrySensor;
}

using namespace argos;

class CMGGBunkerMini : public CMGGRobotController {

public:

   virtual void Init(TConfigurationNode& t_tree) override;
   virtual void ControlStep() override;
   virtual void Reset() override;
   virtual void Destroy() override {}

   virtual void SetPath(const std::vector<CVector3>& vec_waypoints) override;
   virtual void StopPath() override;


   virtual const std::vector<CVector3>& GetPath() const override { return m_vecPath; }
   virtual size_t GetCurrentWaypointIndex() const override { return m_unCurrentWaypoint; }
   virtual bool HasActivePath() const override {
      return !m_vecPath.empty() && m_unCurrentWaypoint < m_vecPath.size();
   }

   virtual const std::string& GetRobotId() const override { return m_strRobotId; }

   virtual CCI_PositioningSensor* GetPositioning() override { return m_pcPositioning; }
   virtual CCI_PhotorealisticLidarSensor* GetLidar() override { return m_pcLidar; }
   virtual CCI_OdometrySensor* GetOdometry() override { return m_pcOdometry; }
   virtual CCI_IMUSensor* GetIMU() override { return m_pcIMU; }


private:

   void FollowWaypoints();

   CCI_DifferentialSteeringActuator* m_pcWheels = nullptr;
   CCI_PositioningSensor*            m_pcPositioning = nullptr;
   CCI_PhotorealisticLidarSensor*    m_pcLidar = nullptr;
   CCI_OdometrySensor*               m_pcOdometry = nullptr;
   CCI_IMUSensor*                    m_pcIMU = nullptr;

   std::string m_strRobotId = "r0";
   std::vector<CVector3> m_vecPath;
   size_t m_unCurrentWaypoint = 0;

   Real m_fWaypointTolerance = 0.25f;
   Real m_fGoalTolerance = 0.30f;
   Real m_fCruiseSpeed = 40.0f;
   Real m_fTurnSpeed = 20.0f;
   Real m_fTurnGain = 45.0f;
   Real m_fAlignThresholdDeg = 35.0f;

};

#endif
