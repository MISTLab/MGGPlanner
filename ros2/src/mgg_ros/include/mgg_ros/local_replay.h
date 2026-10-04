#ifndef MGG_ROS_LOCAL_REPLAY_H_
#define MGG_ROS_LOCAL_REPLAY_H_

#include <filesystem>

namespace mgg {
// C5 production replay. The explicit ROS parameter YAML is required until
// the manifest supports hash-listed parameter inputs. Failure returns nonzero
// without publishing a partial output; diagnostics name the missing input.
int runLocalReplay(const std::filesystem::path& manifest,
                   const std::filesystem::path& output);
int runLocalReplay(const std::filesystem::path& manifest,
                   const std::filesystem::path& output,
                   const std::filesystem::path& local_params);
}  // namespace mgg
#endif
