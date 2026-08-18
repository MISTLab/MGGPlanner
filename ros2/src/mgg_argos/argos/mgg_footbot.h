/*
 * A foot-bot the MGG planner drives.
 *
 * Unlike swarm_slam_footbot in argos3-examples, which follows a route fixed
 * at configuration time, this controller starts with no route at all and
 * follows whatever path the planner sends over the bridge. That is the whole
 * point of the exercise: the planner has to be the thing deciding where the
 * robot goes.
 *
 * Path following, rather than velocity streaming, because that is what a real
 * robot does. mgg_pci publishes a nav_msgs/Path of a handful of poses and the
 * robot's own controller drives it; streaming wheel speeds at tick rate would
 * put the planner inside the servo loop and make the run depend on the
 * bridge's latency. It also keeps the ARGoS side useful when the planner is
 * mid-computation: the robot keeps driving its last path instead of stalling.
 *
 * Sensor pointers are public so the bridge loop function can read the same
 * readings this controller sees without a second copy. Loop functions run
 * after every ControlStep, so what they read belongs to the tick that just
 * finished.
 *
 * Params (in <params>):
 *   robot_id           default "r0"; the ROS namespace and bridge key
 *   waypoint_tolerance default 0.15 m (2D); foot-bot scale, not SMB scale
 *   cruise_speed       default 20 (cm/s, both wheels when aligned)
 *   turn_speed         default 10 (cm/s, wheel speed for in-place turns)
 *   turn_gain          default 30 (cm/s of differential correction per
 *                      radian of heading error while cruising)
 *   align_threshold    default 35 (degrees: heading error above which the
 *                      robot turns in place instead of arcing)
 *   goal_tolerance     default 0.2 m; distance at which the final waypoint
 *                      counts as reached and the path is considered done
 */

#ifndef MGG_FOOTBOT_H
#define MGG_FOOTBOT_H

#include "mgg_robot_controller.h"

namespace argos {
   class CCI_DifferentialSteeringActuator;
   class CCI_PositioningSensor;
   class CCI_PhotorealisticLidarSensor;
   class CCI_IMUSensor;
   class CCI_OdometrySensor;
}

using namespace argos;

class CMGGFootbot : public CMGGRobotController {

public:

   virtual void Init(TConfigurationNode& t_tree) override;
   virtual void ControlStep() override;
   virtual void Reset() override;
   virtual void Destroy() override {}

   virtual void SetPath(const std::vector<CVector3>& vec_waypoints) override;

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

   /** Drops the current path and stops the wheels. */
   void StopPath();

   /** True while there are waypoints left to reach. */
   bool HasPath() const { return m_unCurrentWaypoint < m_vecPath.size(); }

   /** How many waypoints of the current path are still ahead. */
   size_t GetRemainingWaypoints() const {
      return HasPath() ? m_vecPath.size() - m_unCurrentWaypoint : 0;
   }

private:

   CCI_DifferentialSteeringActuator* m_pcWheels = nullptr;
   CCI_PositioningSensor*            m_pcPositioning = nullptr;
   CCI_PhotorealisticLidarSensor*    m_pcLidar = nullptr;
   CCI_OdometrySensor*               m_pcOdometry = nullptr;
   CCI_IMUSensor*                    m_pcIMU = nullptr;

   std::string m_strRobotId = "r0";
   std::vector<CVector3> m_vecPath;
   size_t m_unCurrentWaypoint = 0;

   Real m_fWaypointTolerance = 0.15;
   Real m_fGoalTolerance = 0.2;
   Real m_fCruiseSpeed = 20.0;
   Real m_fTurnSpeed = 10.0;
   Real m_fTurnGain = 30.0;
   Real m_fAlignThresholdDeg = 35.0;

};

#endif

