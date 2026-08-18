#include "mgg_bunker_mini.h"

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

void CMGGBunkerMini::Init(TConfigurationNode& t_tree) {
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcLidar = GetSensor<CCI_PhotorealisticLidarSensor>("photorealistic_lidar");

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

void CMGGBunkerMini::Reset() {
   StopPath();
}

/****************************************/
/****************************************/

void CMGGBunkerMini::StopPath() {
   m_vecPath.clear();
   m_unCurrentWaypoint = 0;
   if(m_pcWheels != nullptr) {
      m_pcWheels->SetLinearVelocity(0.0, 0.0);
   }
}


/****************************************/
/****************************************/

void CMGGBunkerMini::SetPath(const std::vector<CVector3>& vec_waypoints) {
   m_vecPath = vec_waypoints;
   m_unCurrentWaypoint = 0;
   if(m_vecPath.empty() || m_pcPositioning == nullptr) return;

   const CCI_PositioningSensor::SReading& sPose = m_pcPositioning->GetReading();
   const Real fRobotX = sPose.Position.GetX();
   const Real fRobotY = sPose.Position.GetY();

   /* Skip waypoints already traversed */
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
      const Real fSegLenSq = fAbX * fAbX + fAbY * fAbY;
      if(fSegLenSq < 1e-4) {
         ++m_unCurrentWaypoint;
         continue;
      }
      const Real fProj = ((fRobotX - cA.GetX()) * fAbX +
                          (fRobotY - cA.GetY()) * fAbY) / fSegLenSq;
      if(fProj > 0.9) {
         ++m_unCurrentWaypoint;
      } else {
         break;
      }
   }
}

/****************************************/
/****************************************/

void CMGGBunkerMini::ControlStep() {
   if(!HasActivePath() || m_pcPositioning == nullptr) {
      m_pcWheels->SetLinearVelocity(0.0, 0.0);
      return;
   }
   FollowWaypoints();
}

/****************************************/
/****************************************/

void CMGGBunkerMini::FollowWaypoints() {
   const CCI_PositioningSensor::SReading& sPose = m_pcPositioning->GetReading();
   const CVector3& cRobotPos = sPose.Position;
   CRadians cZAngle, cYAngle, cXAngle;
   sPose.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

   const bool bIsFinal = (m_unCurrentWaypoint + 1 >= m_vecPath.size());
   const Real fTolerance = bIsFinal ? m_fGoalTolerance : m_fWaypointTolerance;

   const CVector3& cTarget = m_vecPath[m_unCurrentWaypoint];
   const Real fDx = cTarget.GetX() - cRobotPos.GetX();
   const Real fDy = cTarget.GetY() - cRobotPos.GetY();
   const Real fDist2D = std::sqrt(fDx * fDx + fDy * fDy);

   if(fDist2D <= fTolerance) {
      ++m_unCurrentWaypoint;
      if(!HasActivePath()) {
         m_pcWheels->SetLinearVelocity(0.0, 0.0);
         return;
      }
      FollowWaypoints();
      return;
   }

   const Real fBearing = std::atan2(fDy, fDx);
   Real fHeadingError = fBearing - cZAngle.GetValue();
   while(fHeadingError >  ARGOS_PI) fHeadingError -= 2.0 * ARGOS_PI;
   while(fHeadingError < -ARGOS_PI) fHeadingError += 2.0 * ARGOS_PI;

   const Real fAlignThresh = m_fAlignThresholdDeg * ARGOS_PI / 180.0;
   if(std::abs(fHeadingError) > fAlignThresh) {
      /* Turn in place */
      if(fHeadingError > 0.0) {
         m_pcWheels->SetLinearVelocity(-m_fTurnSpeed, m_fTurnSpeed);
      } else {
         m_pcWheels->SetLinearVelocity(m_fTurnSpeed, -m_fTurnSpeed);
      }
      return;
   }

   /* Pure-pursuit drive with heading correction */
   Real fLeft = m_fCruiseSpeed - m_fTurnGain * fHeadingError;
   Real fRight = m_fCruiseSpeed + m_fTurnGain * fHeadingError;

   const Real fMax = m_fCruiseSpeed * 1.5;
   fLeft = std::max(-fMax, std::min(fMax, fLeft));
   fRight = std::max(-fMax, std::min(fMax, fRight));

   m_pcWheels->SetLinearVelocity(fLeft, fRight);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CMGGBunkerMini, "mgg_bunker_mini_controller");
