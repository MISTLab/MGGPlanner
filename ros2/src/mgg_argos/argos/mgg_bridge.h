/*
 * Streams every robot's lidar and odometry to the MGG planner, and drives
 * the robots with the paths it sends back.
 *
 * The bidirectional half is what separates this from swarm_slam_bridge in
 * argos3-examples, which streams to a SLAM front end that only ever
 * consumes. A planner has to steer, so the reply carries commands instead
 * of a bare ack. See include/mgg_argos/protocol.h for the wire format.
 *
 * ARGoS does not link against ROS 2; the far side of the socket does the
 * publishing. The simulator keeps building and running with no ROS 2
 * present.
 *
 * The exchange is lockstep: PostStep writes the observation and blocks on
 * the reply, so the simulation cannot outrun the planner. A planning cycle
 * that takes two seconds costs two seconds of wall-clock and no simulated
 * data at all, which is the property that makes results reproducible.
 *
 * Attributes:
 *   socket           path of the Unix socket to connect to (required)
 *   robots           comma-separated entity ids, in robot order (required)
 *   timeout          seconds to wait for the reply (default 300; a planning
 *                    cycle is far slower than a SLAM keyframe)
 *   connect_timeout  seconds to wait for the far side at startup
 *                    (default 60), since the container needs a moment
 *   send_ground_truth  default false; adds a ground-truth pose block for
 *                    evaluation. The planner never reads it.
 *   path_width       width of the drawn path in metres (default 0.12)
 *   draw_medium      optional; the id of a <photorealism> medium. When given,
 *                    each robot's planned path and graph are drawn into that
 *                    medium's debug overlay, which only the interactive viewer
 *                    shows. Leave it out for headless runs.
 */

#ifndef MGG_BRIDGE_H
#define MGG_BRIDGE_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/vector3.h>

#include <cstdint>
#include <string>
#include <vector>

namespace argos {
   class CPROverlay;
}

class CMGGFootbot;

using namespace argos;

class CMGGBridge : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void Destroy();

private:

   void Connect();
   void SendAll(const void* pt_data, size_t un_size);
   void RecvAll(void* pt_data, size_t un_size);
   void Append(const void* pt_data, size_t un_size);
   void AppendString(const std::string& str_value);
   /** Opens a block, returns where its length has to be back-patched */
   size_t BeginBlock(UInt8 un_type);
   void EndBlock(size_t un_length_offset);
   /** Reads one command message and applies it to the controllers */
   void RecvCommands(UInt32 un_tick);
   /** Reads one robot's overlay blocks and hands them to the debug draw */
   void RecvOverlays(size_t un_robot_index);
   /** Reads a length-prefixed run of f32 triplets */
   void RecvPoints(std::vector<CVector3>& vec_out, std::uint32_t un_count);

   int m_nSocket = -1;
   std::string m_strSocketPath;
   std::vector<std::string> m_vecRobotIds;
   std::vector<CMGGFootbot*> m_vecControllers;
   Real m_fTimeout = 300.0;
   Real m_fConnectTimeout = 60.0;
   UInt32 m_unTicksPerSecond = 10;
   bool m_bSendGroundTruth = false;
   /** Set when a <photorealism> medium is named: the planner's paths and
    *  graphs are then drawn into the viewer. Optional, because a headless
    *  run has nothing to draw into and should not pay for the geometry. */
   CPROverlay* m_pcOverlay = nullptr;
   /** Width of the drawn path, metres. The graph stays as hairlines: at a few
    *  thousand edges, giving it real geometry costs twelve vertices an edge
    *  and buries the path it is supposed to be context for. */
   Real m_fPathWidth = 0.12;
   /** Scratch, reused per tick so a long run does not churn the allocator */
   std::vector<CVector3> m_vecPoints;
   /* Reused every tick so a long run does not churn the allocator */
   std::vector<UInt8> m_vecBuffer;
   std::vector<UInt8> m_vecReply;

};

#endif
