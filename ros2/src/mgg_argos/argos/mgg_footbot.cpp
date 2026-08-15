#include "mgg_footbot.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_imu_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_odometry_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <cmath>

/****************************************/
/****************************************/

void CMGGFootbot::Init(TConfigurationNode& t_tree) {
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcLidar = GetSensor<CCI_PhotorealisticLidarSensor>("photorealistic_lidar");
   /* The drift-injected odometry and the IMU are what the planner is
    * allowed to see. Both optional, so a bare experiment still runs. */
   try {
      m_pcOdometry = GetSensor<CCI_OdometrySensor>("odometry");
   }
   catch(CARGoSException&) {
      m_pcOdometry = nullptr;
   }
   try {
      m_pcIMU = GetSensor<CCI_IMUSensor>("imu");
   }
   catch(CARGoSException&) {
      m_pcIMU = nullptr;
   }
   GetNodeAttributeOrDefault(t_tree, "robot_id", m_strRobotId, m_strRobotId);
   GetNodeAttributeOrDefault(t_tree, "waypoint_tolerance",
                             m_fWaypointTolerance, m_fWaypointTolerance);
   GetNodeAttributeOrDefault(t_tree, "goal_tolerance",
                             m_fGoalTolerance, m_fGoalTolerance);
   GetNodeAttributeOrDefault(t_tree, "cruise_speed",
                             m_fCruiseSpeed, m_fCruiseSpeed);
   GetNodeAttributeOrDefault(t_tree, "turn_speed", m_fTurnSpeed, m_fTurnSpeed);
   GetNodeAttributeOrDefault(t_tree, "turn_gain", m_fTurnGain, m_fTurnGain);
   GetNodeAttributeOrDefault(t_tree, "align_threshold",
                             m_fAlignThresholdDeg, m_fAlignThresholdDeg);
}

/****************************************/
/****************************************/

void CMGGFootbot::Reset() {
   m_vecPath.clear();
   m_unCurrentWaypoint = 0;
   m_pcWheels->SetLinearVelocity(0.0, 0.0);
}

/****************************************/
/****************************************/

void CMGGFootbot::SetPath(const std::vector<CVector3>& vec_waypoints) {
   m_vecPath = vec_waypoints;
   m_unCurrentWaypoint = 0;
   /* The planner's first waypoint is normally the robot's current pose,
    * which would be reached immediately; leaving it in costs nothing
    * because the tolerance check below skips it on the first step. */
}

/****************************************/
/****************************************/

void CMGGFootbot::StopPath() {
   m_vecPath.clear();
   m_unCurrentWaypoint = 0;
   m_pcWheels->SetLinearVelocity(0.0, 0.0);
}

/****************************************/
/****************************************/

void CMGGFootbot::ControlStep() {
   if(!HasPath()) {
      m_pcWheels->SetLinearVelocity(0.0, 0.0);
      return;
   }
   /* Ground truth is used to drive, as in swarm_slam_footbot: this is a
    * planner experiment, not a state-estimation one. The planner itself
    * only ever sees the drift-injected odometry. */
   const CCI_PositioningSensor::SReading& sPose = m_pcPositioning->GetReading();
   CRadians cYaw, cPitch, cRoll;
   sPose.Orientation.ToEulerAngles(cYaw, cPitch, cRoll);

   /* Retire every waypoint already within tolerance, not just the next
    * one: a path that doubles back can put several behind the robot in
    * one step, and advancing one per tick would make it crawl through
    * them. The last waypoint gets its own, looser tolerance. */
   while(m_unCurrentWaypoint < m_vecPath.size()) {
      const bool bIsLast = (m_unCurrentWaypoint + 1 == m_vecPath.size());
      const CVector3& cTarget = m_vecPath[m_unCurrentWaypoint];
      const Real fDistance =
         std::sqrt(std::pow(cTarget.GetX() - sPose.Position.GetX(), 2) +
                   std::pow(cTarget.GetY() - sPose.Position.GetY(), 2));
      if(fDistance > (bIsLast ? m_fGoalTolerance : m_fWaypointTolerance)) {
         break;
      }
      ++m_unCurrentWaypoint;
   }
   if(!HasPath()) {
      m_pcWheels->SetLinearVelocity(0.0, 0.0);
      return;
   }

   const CVector3& cTarget = m_vecPath[m_unCurrentWaypoint];
   const Real fDx = cTarget.GetX() - sPose.Position.GetX();
   const Real fDy = cTarget.GetY() - sPose.Position.GetY();
   CRadians cHeadingError = CRadians(std::atan2(fDy, fDx)) - cYaw;
   cHeadingError.SignedNormalize();
   const Real fHeadingErrorDeg = ToDegrees(cHeadingError).GetValue();

   if(std::abs(fHeadingErrorDeg) > m_fAlignThresholdDeg) {
      /* Too far off to arc onto: turn in place. A differential robot
       * that tries to arc through a large heading error sweeps a wide
       * curve, which in a corridor means into the wall. */
      const Real fSign = fHeadingErrorDeg > 0.0 ? 1.0 : -1.0;
      m_pcWheels->SetLinearVelocity(-fSign * m_fTurnSpeed,
                                     fSign * m_fTurnSpeed);
      return;
   }
   /* Aligned enough: drive, correcting proportionally */
   const Real fCorrection = m_fTurnGain * cHeadingError.GetValue();
   m_pcWheels->SetLinearVelocity(m_fCruiseSpeed - fCorrection,
                                 m_fCruiseSpeed + fCorrection);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CMGGFootbot, "mgg_footbot");
