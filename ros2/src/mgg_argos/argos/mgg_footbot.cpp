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
   if(m_vecPath.empty() || m_pcPositioning == nullptr) return;

   const CCI_PositioningSensor::SReading& sPose = m_pcPositioning->GetReading();
   const Real fRobotX = sPose.Position.GetX();
   const Real fRobotY = sPose.Position.GetY();

   /* If the robot has already moved ahead of the start of this path
    * (e.g. while the planner was computing), advance to the first waypoint
    * that is actually ahead of the robot so it never doubles back. */
   while(m_unCurrentWaypoint + 1 < m_vecPath.size()) {
      const CVector3& cA = m_vecPath[m_unCurrentWaypoint];
      const CVector3& cB = m_vecPath[m_unCurrentWaypoint + 1];
      const Real fDistA = std::sqrt(std::pow(cA.GetX() - fRobotX, 2) +
                                    std::pow(cA.GetY() - fRobotY, 2));
      if(fDistA <= m_fWaypointTolerance) {
         ++m_unCurrentWaypoint;
         continue;
      }
      const Real fAbX = cB.GetX() - cA.GetX();
      const Real fAbY = cB.GetY() - cA.GetY();
      const Real fAbLenSq = fAbX * fAbX + fAbY * fAbY;
      if(fAbLenSq > 1e-6) {
         const Real fApX = fRobotX - cA.GetX();
         const Real fApY = fRobotY - cA.GetY();
         const Real fProj = (fApX * fAbX + fApY * fAbY) / fAbLenSq;
         if(fProj >= 0.5) {
            const Real fPerpDistSq = (fApX * fApX + fApY * fApY) - fProj * fProj * fAbLenSq;
            if(fPerpDistSq <= std::pow(m_fWaypointTolerance * 2.0, 2)) {
               ++m_unCurrentWaypoint;
               continue;
            }
         }
      }
      break;
   }
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

   /* Retire every waypoint already within tolerance or behind the robot.
    * The last waypoint gets its own, looser tolerance. */
   while(m_unCurrentWaypoint < m_vecPath.size()) {
      const bool bIsLast = (m_unCurrentWaypoint + 1 == m_vecPath.size());
      const CVector3& cTarget = m_vecPath[m_unCurrentWaypoint];
      const Real fDistance =
         std::sqrt(std::pow(cTarget.GetX() - sPose.Position.GetX(), 2) +
                   std::pow(cTarget.GetY() - sPose.Position.GetY(), 2));
      if(fDistance <= (bIsLast ? m_fGoalTolerance : m_fWaypointTolerance)) {
         ++m_unCurrentWaypoint;
         continue;
      }
      if(!bIsLast) {
         const CVector3& cNext = m_vecPath[m_unCurrentWaypoint + 1];
         const Real fAbX = cNext.GetX() - cTarget.GetX();
         const Real fAbY = cNext.GetY() - cTarget.GetY();
         const Real fAbLenSq = fAbX * fAbX + fAbY * fAbY;
         if(fAbLenSq > 1e-6) {
            const Real fApX = sPose.Position.GetX() - cTarget.GetX();
            const Real fApY = sPose.Position.GetY() - cTarget.GetY();
            const Real fProj = (fApX * fAbX + fApY * fAbY) / fAbLenSq;
            if(fProj >= 0.5) {
               const Real fPerpDistSq = (fApX * fApX + fApY * fApY) - fProj * fProj * fAbLenSq;
               if(fPerpDistSq <= std::pow(m_fWaypointTolerance * 1.5, 2)) {
                  ++m_unCurrentWaypoint;
                  continue;
               }
            }
         }
      }
      break;
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
