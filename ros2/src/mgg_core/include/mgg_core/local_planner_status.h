#ifndef MGG_CORE_LOCAL_PLANNER_STATUS_H_
#define MGG_CORE_LOCAL_PLANNER_STATUS_H_
namespace mgg {
enum class LocalStatus { kMoving, kWaitingForMap, kNoLocalTarget, kBlocked };
}
#endif
