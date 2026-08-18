#include "mgg_bridge.h"
#include "mgg_robot_controller.h"
#include "../include/mgg_argos/protocol.h"


#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/generic/control_interface/ci_imu_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_odometry_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/simulator/photorealism/pr_overlay.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>

using namespace mgg_argos::protocol;

/****************************************/
/****************************************/

void CMGGBridge::Init(TConfigurationNode& t_tree) {
   GetNodeAttribute(t_tree, "socket", m_strSocketPath);
   GetNodeAttributeOrDefault(t_tree, "timeout", m_fTimeout, m_fTimeout);
   GetNodeAttributeOrDefault(t_tree, "connect_timeout",
                             m_fConnectTimeout, m_fConnectTimeout);
   GetNodeAttributeOrDefault(t_tree, "send_ground_truth",
                             m_bSendGroundTruth, m_bSendGroundTruth);
   std::string strDrawMedium;
   GetNodeAttributeOrDefault(t_tree, "draw_medium", strDrawMedium, strDrawMedium);
   GetNodeAttributeOrDefault(t_tree, "path_width", m_fPathWidth, m_fPathWidth);
   if(!strDrawMedium.empty()) {
      m_pcOverlay = &GetPhotorealismOverlay(strDrawMedium);
   }
   std::string strRobots;
   GetNodeAttribute(t_tree, "robots", strRobots);
   std::istringstream cRobots(strRobots);
   std::string strRobot;
   while(std::getline(cRobots, strRobot, ',')) {
      if(!strRobot.empty()) {
         m_vecRobotIds.push_back(strRobot);
      }
   }
   if(m_vecRobotIds.empty()) {
      THROW_ARGOSEXCEPTION("The \"robots\" list is empty");
   }
   /* Resolve each robot's controller once; the per-tick path then only
    * reads sensors */
   for(const std::string& strId : m_vecRobotIds) {
      auto& cEntity = dynamic_cast<CComposableEntity&>(
         GetSpace().GetEntity(strId));
      auto& cControllable =
         cEntity.GetComponent<CControllableEntity>("controller");
      auto* pcController =
         dynamic_cast<CMGGRobotController*>(&cControllable.GetController());
      if(pcController == nullptr) {
         THROW_ARGOSEXCEPTION("Robot \"" << strId << "\" does not run an "
                              "MGG robot controller (e.g. mgg_footbot or mgg_bunker_mini)");
      }
      m_vecControllers.push_back(pcController);

   }
   m_unTicksPerSecond =
      UInt32(CPhysicsEngine::GetInverseSimulationClockTick());
   Connect();
}

/****************************************/
/****************************************/

void CMGGBridge::Connect() {
   m_nSocket = ::socket(AF_UNIX, SOCK_STREAM, 0);
   if(m_nSocket < 0) {
      THROW_ARGOSEXCEPTION("Cannot create the bridge socket: "
                           << ::strerror(errno));
   }
   struct sockaddr_un tAddress;
   std::memset(&tAddress, 0, sizeof(tAddress));
   tAddress.sun_family = AF_UNIX;
   if(m_strSocketPath.size() >= sizeof(tAddress.sun_path)) {
      THROW_ARGOSEXCEPTION("Socket path \"" << m_strSocketPath
                           << "\" is too long");
   }
   std::strncpy(tAddress.sun_path, m_strSocketPath.c_str(),
                sizeof(tAddress.sun_path) - 1);
   LOG << "[BRIDGE] Waiting for the MGG planner on " << m_strSocketPath
       << " ..." << std::endl;
   LOG.Flush();
   Real fWaited = 0.0;
   while(::connect(m_nSocket, reinterpret_cast<struct sockaddr*>(&tAddress),
                   sizeof(tAddress)) < 0) {
      if(fWaited >= m_fConnectTimeout) {
         THROW_ARGOSEXCEPTION("Could not connect to the MGG bridge at \""
                              << m_strSocketPath << "\" after "
                              << m_fConnectTimeout << " s: "
                              << ::strerror(errno)
                              << ". Is mgg_argos_bridge running?");
      }
      ::usleep(200000);
      fWaited += 0.2;
   }
   /* Bound the reply wait so a crashed far side stops the run instead of
    * hanging it forever */
   struct timeval tTimeout;
   tTimeout.tv_sec = time_t(m_fTimeout);
   tTimeout.tv_usec = 0;
   ::setsockopt(m_nSocket, SOL_SOCKET, SO_RCVTIMEO, &tTimeout, sizeof(tTimeout));
   LOG << "[BRIDGE] Connected; running in lockstep with the planner"
       << std::endl;
   LOG.Flush();
}

/****************************************/
/****************************************/

void CMGGBridge::Append(const void* pt_data, size_t un_size) {
   const auto* punData = static_cast<const UInt8*>(pt_data);
   m_vecBuffer.insert(m_vecBuffer.end(), punData, punData + un_size);
}

void CMGGBridge::AppendString(const std::string& str_value) {
   auto unLength = UInt8(str_value.size());
   Append(&unLength, 1);
   Append(str_value.data(), str_value.size());
}

size_t CMGGBridge::BeginBlock(UInt8 un_type) {
   Append(&un_type, 1);
   const size_t unOffset = m_vecBuffer.size();
   std::uint32_t unPlaceholder = 0;
   Append(&unPlaceholder, sizeof(unPlaceholder));
   return unOffset;
}

void CMGGBridge::EndBlock(size_t un_length_offset) {
   const auto unLength =
      std::uint32_t(m_vecBuffer.size() - un_length_offset - sizeof(std::uint32_t));
   std::memcpy(&m_vecBuffer[un_length_offset], &unLength, sizeof(unLength));
}

/****************************************/
/****************************************/

void CMGGBridge::SendAll(const void* pt_data, size_t un_size) {
   const auto* punData = static_cast<const UInt8*>(pt_data);
   size_t unSent = 0;
   while(unSent < un_size) {
      ssize_t nWritten = ::send(m_nSocket, punData + unSent,
                                un_size - unSent, MSG_NOSIGNAL);
      if(nWritten <= 0) {
         if(errno == EINTR) continue;
         THROW_ARGOSEXCEPTION("Lost the MGG bridge while sending: "
                              << ::strerror(errno));
      }
      unSent += size_t(nWritten);
   }
}

/****************************************/
/****************************************/

void CMGGBridge::RecvAll(void* pt_data, size_t un_size) {
   auto* punData = static_cast<UInt8*>(pt_data);
   size_t unRead = 0;
   while(unRead < un_size) {
      ssize_t nGot = ::recv(m_nSocket, punData + unRead, un_size - unRead, 0);
      if(nGot == 0) {
         THROW_ARGOSEXCEPTION("The MGG bridge closed the connection");
      }
      if(nGot < 0) {
         if(errno == EINTR) continue;
         if(errno == EAGAIN || errno == EWOULDBLOCK) {
            THROW_ARGOSEXCEPTION("The planner did not reply within "
                                 << m_fTimeout << " s; it is either stuck or "
                                 "has died");
         }
         THROW_ARGOSEXCEPTION("Lost the MGG bridge while waiting: "
                              << ::strerror(errno));
      }
      unRead += size_t(nGot);
   }
}

/****************************************/
/****************************************/

void CMGGBridge::RecvCommands(UInt32 un_tick) {
   char pchMagic[4];
   RecvAll(pchMagic, sizeof(pchMagic));
   if(std::memcmp(pchMagic, kCommandMagic, 4) != 0) {
      THROW_ARGOSEXCEPTION("Bad command magic from the bridge; the stream is "
                           "out of sync");
   }
   std::uint16_t unVersion = 0;
   RecvAll(&unVersion, sizeof(unVersion));
   if(unVersion != kVersion) {
      THROW_ARGOSEXCEPTION("The bridge speaks protocol version " << unVersion
                           << ", this build speaks " << kVersion);
   }
   std::uint32_t unTick = 0;
   RecvAll(&unTick, sizeof(unTick));
   if(unTick != un_tick) {
      /* Every observation gets exactly one reply, so a mismatch means a
       * message was lost or misparsed. Better to stop than to steer a
       * robot with a command meant for a different tick. */
      THROW_ARGOSEXCEPTION("The bridge replied for tick " << unTick
                           << " while tick " << un_tick << " is in flight");
   }
   std::uint32_t unRobots = 0;
   RecvAll(&unRobots, sizeof(unRobots));
   /* Overlays are rebuilt from scratch each tick rather than accumulated:
    * a graph that is not resent has been superseded, and leaving the old one
    * on screen would show a plan the robot is no longer following. */
   if(m_pcOverlay != nullptr) {
      m_pcOverlay->Clear();
   }
   for(std::uint32_t i = 0; i < unRobots; ++i) {
      UInt8 unIdLength = 0;
      RecvAll(&unIdLength, 1);
      std::string strId(unIdLength, '\0');
      if(unIdLength > 0) RecvAll(&strId[0], unIdLength);
      UInt8 unType = 0;
      RecvAll(&unType, 1);
      /* Resolve the target before reading the payload, so an unknown id
       * still consumes its bytes and leaves the stream aligned */
      CMGGRobotController* pcController = nullptr;
      for(CMGGRobotController* pcCandidate : m_vecControllers) {
         if(pcCandidate->GetRobotId() == strId) {
            pcController = pcCandidate;
            break;
         }
      }

      if(unType == kCommandPath) {
         std::uint32_t unWaypoints = 0;
         RecvAll(&unWaypoints, sizeof(unWaypoints));
         std::vector<CVector3> vecPath;
         vecPath.reserve(unWaypoints);
         for(std::uint32_t w = 0; w < unWaypoints; ++w) {
            double pfPose[4];
            RecvAll(pfPose, sizeof(pfPose));
            vecPath.emplace_back(pfPose[0], pfPose[1], pfPose[2]);
         }
         if(pcController != nullptr) {
            pcController->SetPath(vecPath);
         }
      }
      else if(unType == kCommandStop) {
         if(pcController != nullptr) {
            pcController->StopPath();
         }
      }
      else if(unType != kCommandNone) {
         THROW_ARGOSEXCEPTION("Unknown command type " << int(unType)
                              << " for robot \"" << strId << "\"");
      }
      if(pcController == nullptr) {
         LOGERR << "[BRIDGE] Command for unknown robot \"" << strId
                << "\", ignored" << std::endl;
      }
      /* Read the overlay blocks whether or not anything will be drawn with
       * them: they are in the stream either way, and skipping the read would
       * leave the parser mid-message. */
      RecvOverlays(i);
   }
}

/****************************************/
/****************************************/

void CMGGBridge::RecvPoints(std::vector<CVector3>& vec_out,
                            std::uint32_t un_count) {
   vec_out.clear();
   vec_out.reserve(un_count);
   constexpr Real fOverlayLift = 0.12;
   for(std::uint32_t i = 0; i < un_count; ++i) {
      float pfPoint[3];
      RecvAll(pfPoint, sizeof(pfPoint));
      vec_out.emplace_back(pfPoint[0], pfPoint[1], pfPoint[2] + fOverlayLift);
   }
}

/****************************************/
/****************************************/

void CMGGBridge::RecvOverlays(size_t un_robot_index) {
   UInt8 unBlocks = 0;
   RecvAll(&unBlocks, 1);
   /* One colour per robot, so four robots' graphs stay distinguishable when
    * they overlap. Deliberately saturated: these are drawn over a
    * photorealistic scene and have to read against it. */
   static const CColor pcPalette[] = {
      CColor(51, 153, 255),    /* blue    */
      CColor(255, 115, 26),    /* orange  */
      CColor(77, 230, 90),     /* green   */
      CColor(242, 64, 191),    /* magenta */
   };
   const CColor& cColor =
      pcPalette[un_robot_index % (sizeof(pcPalette) / sizeof(pcPalette[0]))];
   /* The graph is the same hue, dimmed: it is context for the path, and at a
    * few thousand edges it would otherwise drown the path out entirely. */
   const CColor cGraphColor(UInt8(cColor.GetRed() * 0.55),
                            UInt8(cColor.GetGreen() * 0.55),
                            UInt8(cColor.GetBlue() * 0.55));
   constexpr Real fOverlayLift = 0.12;

   for(UInt8 b = 0; b < unBlocks; ++b) {
      UInt8 unType = 0;
      std::uint32_t unLength = 0;
      RecvAll(&unType, 1);
      RecvAll(&unLength, sizeof(unLength));
      if(m_pcOverlay == nullptr) {
         /* Nothing to draw into: consume the payload and move on */
         std::vector<UInt8> vecSkip(unLength);
         if(unLength > 0) RecvAll(vecSkip.data(), unLength);
         continue;
      }
      CPROverlay& cDraw = *m_pcOverlay;
      switch(unType) {
         case kOverlayPath: {
            std::uint32_t unCount = 0;
            RecvAll(&unCount, sizeof(unCount));
            RecvPoints(m_vecPoints, unCount);
            cDraw.AddThickPolyline(m_vecPoints, m_fPathWidth, cColor);
            if(!m_vecPoints.empty()) {
               const CVector3& cGoal = m_vecPoints.back();
               const Real fR = 0.3;
               cDraw.AddThickLine(cGoal - CVector3(fR, 0.0, 0.0),
                                  cGoal + CVector3(fR, 0.0, 0.0),
                                  m_fPathWidth * 0.8, cColor);
               cDraw.AddThickLine(cGoal - CVector3(0.0, fR, 0.0),
                                  cGoal + CVector3(0.0, fR, 0.0),
                                  m_fPathWidth * 0.8, cColor);
            }
            break;
         }
         case kOverlayGraphEdges: {
            std::uint32_t unCount = 0;
            RecvAll(&unCount, sizeof(unCount));
            for(std::uint32_t e = 0; e < unCount; ++e) {
               float pfSegment[6];
               RecvAll(pfSegment, sizeof(pfSegment));
               cDraw.AddLine(CVector3(pfSegment[0], pfSegment[1], pfSegment[2] + fOverlayLift),
                             CVector3(pfSegment[3], pfSegment[4], pfSegment[5] + fOverlayLift),
                             cGraphColor);
            }
            break;
         }
         case kOverlayGlobalGraph: {
            std::uint32_t unCount = 0;
            RecvAll(&unCount, sizeof(unCount));
            const CColor cGlobalColor(255, 215, 0); /* Bright Gold */
            for(std::uint32_t e = 0; e < unCount; ++e) {
               float pfSegment[6];
               RecvAll(pfSegment, sizeof(pfSegment));
               cDraw.AddThickLine(CVector3(pfSegment[0], pfSegment[1], pfSegment[2] + fOverlayLift),
                                  CVector3(pfSegment[3], pfSegment[4], pfSegment[5] + fOverlayLift),
                                  m_fPathWidth * 0.75, cGlobalColor);
            }
            break;
         }
         case kOverlayMerge: {
            std::uint32_t unCount = 0;
            RecvAll(&unCount, sizeof(unCount));
            const CColor cMergeColor(0, 255, 255); /* Bright Cyan */
            for(std::uint32_t e = 0; e < unCount; ++e) {
               float pfSegment[6];
               RecvAll(pfSegment, sizeof(pfSegment));
               cDraw.AddThickLine(CVector3(pfSegment[0], pfSegment[1], pfSegment[2] + fOverlayLift),
                                  CVector3(pfSegment[3], pfSegment[4], pfSegment[5] + fOverlayLift),
                                  m_fPathWidth * 1.2, cMergeColor);
            }
            break;
         }

         case kOverlayPoints: {
            std::uint32_t unCount = 0;
            RecvAll(&unCount, sizeof(unCount));
            RecvPoints(m_vecPoints, unCount);
            const Real fR = 0.2;
            for(const CVector3& cPoint : m_vecPoints) {
               cDraw.AddLine(cPoint - CVector3(fR, 0.0, 0.0),
                             cPoint + CVector3(fR, 0.0, 0.0), cColor);
               cDraw.AddLine(cPoint - CVector3(0.0, fR, 0.0),
                             cPoint + CVector3(0.0, fR, 0.0), cColor);
            }
            break;
         }
         default: {
            /* Length-prefixed precisely so an unknown overlay can be skipped */
            std::vector<UInt8> vecSkip(unLength);
            if(unLength > 0) RecvAll(vecSkip.data(), unLength);
            break;
         }
      }

   }
}

/****************************************/
/****************************************/

void CMGGBridge::PostStep() {
   m_vecBuffer.clear();
   const auto unTick = UInt32(GetSpace().GetSimulationClock());
   const auto unRobots = std::uint32_t(m_vecControllers.size());
   Append(kObservationMagic, 4);
   std::uint16_t unVersion = kVersion;
   Append(&unVersion, sizeof(unVersion));
   Append(&unTick, sizeof(unTick));
   Append(&m_unTicksPerSecond, sizeof(m_unTicksPerSecond));
   Append(&unRobots, sizeof(unRobots));
   for(CMGGRobotController* pcController : m_vecControllers) {
      AppendString(pcController->GetRobotId());
      /* Count the blocks this robot will emit before writing any */
      UInt8 unBlocks = 0;
      const bool bHasOdometry = pcController->GetOdometry() != nullptr;
      const bool bHasImu = pcController->GetIMU() != nullptr;
      const bool bHasScan = pcController->GetLidar() != nullptr &&
                            pcController->GetLidar()->HasNewScan();
      if(bHasOdometry) ++unBlocks;
      if(bHasImu) ++unBlocks;
      if(bHasScan) ++unBlocks;
      if(m_bSendGroundTruth) ++unBlocks;
      Append(&unBlocks, 1);

      if(bHasOdometry) {
         const size_t unBlock = BeginBlock(kBlockOdometry);
         const CCI_OdometrySensor::SReading& sOdom =
            pcController->GetOdometry()->GetReading();
         const double pfOdom[7] = {
            double(sOdom.Position.GetX()), double(sOdom.Position.GetY()),
            double(sOdom.Position.GetZ()), double(sOdom.Orientation.GetW()),
            double(sOdom.Orientation.GetX()), double(sOdom.Orientation.GetY()),
            double(sOdom.Orientation.GetZ())};
         Append(pfOdom, sizeof(pfOdom));
         EndBlock(unBlock);
      }
      if(bHasScan) {
         /* Only when the sensor actually produced one, so the far side
          * sees exactly the scan rate the sensor delivers */
         const size_t unBlock = BeginBlock(kBlockLidar);
         const CCI_PhotorealisticLidarSensor::SScan& sScan =
            pcController->GetLidar()->GetScan();
         const auto unRings = std::uint32_t(sScan.NumRings);
         const auto unAzimuths = std::uint32_t(sScan.NumAzimuths);
         Append(&unRings, sizeof(unRings));
         Append(&unAzimuths, sizeof(unAzimuths));
         /* The ray pattern is a regular grid, so these three scalars
          * define every direction; sending the table would cost 300 kB
          * a tick to say the same thing. */
         float fElevationMin = 0.0f, fElevationMax = 0.0f;
         if(!sScan.Readings.empty()) {
            fElevationMin = float(sScan.Readings.front().Elevation.GetValue());
            fElevationMax = float(sScan.Readings[sScan.NumRings - 1]
                                     .Elevation.GetValue());
         }
         const auto fMaxRange = float(sScan.MaxRange);
         Append(&fElevationMin, sizeof(fElevationMin));
         Append(&fElevationMax, sizeof(fElevationMax));
         Append(&fMaxRange, sizeof(fMaxRange));
         /* Ranges as f32 and hits as a byte each: ARGoS keeps these as
          * double, but a lidar with millimetre truth does not need 15
          * significant digits, and this halves the per-tick traffic. */
         std::vector<float> vecRanges;
         std::vector<UInt8> vecHits;
         vecRanges.reserve(sScan.Readings.size());
         vecHits.reserve(sScan.Readings.size());
         for(const auto& sReading : sScan.Readings) {
            vecRanges.push_back(float(sReading.Range));
            vecHits.push_back(sReading.Hit ? 1 : 0);
         }
         Append(vecRanges.data(), vecRanges.size() * sizeof(float));
         Append(vecHits.data(), vecHits.size());
         EndBlock(unBlock);
      }
      if(bHasImu) {
         const size_t unBlock = BeginBlock(kBlockImu);
         const CCI_IMUSensor::SReading& sImu = pcController->GetIMU()->GetReading();
         const double pfImu[6] = {
            double(sImu.AngularVelocity.GetX()),
            double(sImu.AngularVelocity.GetY()),
            double(sImu.AngularVelocity.GetZ()),
            double(sImu.LinearAcceleration.GetX()),
            double(sImu.LinearAcceleration.GetY()),
            double(sImu.LinearAcceleration.GetZ())};
         Append(pfImu, sizeof(pfImu));
         EndBlock(unBlock);
      }
      if(m_bSendGroundTruth && pcController->GetPositioning() != nullptr) {
         const size_t unBlock = BeginBlock(kBlockGroundTruth);
         const CCI_PositioningSensor::SReading& sPose =
            pcController->GetPositioning()->GetReading();
         const double pfPose[7] = {
            double(sPose.Position.GetX()), double(sPose.Position.GetY()),
            double(sPose.Position.GetZ()), double(sPose.Orientation.GetW()),
            double(sPose.Orientation.GetX()), double(sPose.Orientation.GetY()),
            double(sPose.Orientation.GetZ())};
         Append(pfPose, sizeof(pfPose));
         EndBlock(unBlock);
      }
   }

   SendAll(m_vecBuffer.data(), m_vecBuffer.size());
   /* Blocks here: this is what makes the run lockstep rather than
    * best-effort, and what lets a slow planning cycle cost wall-clock
    * instead of simulated data */
   RecvCommands(unTick);
}

/****************************************/
/****************************************/

void CMGGBridge::Destroy() {
   if(m_nSocket >= 0) {
      /* Tell the far side the run is over so it can flush its logs */
      ::shutdown(m_nSocket, SHUT_WR);
      ::close(m_nSocket);
      m_nSocket = -1;
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMGGBridge, "mgg_bridge");
