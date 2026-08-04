/*** 
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-10-12 11:33:14
 * @LastEditTime: 2025-03-12 17:02:18
 * @Description: 
 * @
 * @Copyright (c) 2025 by ning-zelin, All Rights Reserved. 
 */


#include <epic_planner/fast_exploration_fsm.h>
#include <epic_planner/fast_exploration_manager.h>
#include <frontier_manager/frontier_manager.h>
#include <plan_manage/backward.hpp>
#include <plan_manage/planner_manager.h>
#include <ros/ros.h>
#include <memory>

namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

namespace {

// The planner is a mutually-referencing object graph with several worker
// threads and callback connections. A same-name registration makes roscpp tear
// those facilities down asynchronously; running the graph's C++ destructors at
// that point has produced both message_filters mutex failures and a
// FastPlannerManager/BubbleAstar double-free. The process is exiting anyway, so
// the shutdown path deliberately abandons this one owner and lets the OS reclaim
// it atomically. During normal operation it remains owned by main().
struct ExplorationRuntime {
  LIOInterface::Ptr lio_interface = std::make_shared<LIOInterface>();
  ParallelBubbleAstar::Ptr parallel_path_finder =
      std::make_shared<ParallelBubbleAstar>();
  FrontierManager::Ptr frontier_manager = std::make_shared<FrontierManager>();
  TopoGraph::Ptr graph = std::make_shared<TopoGraph>();
  FastPlannerManager::Ptr planner_manager =
      std::make_shared<FastPlannerManager>();
  FastExplorationManager::Ptr explore_manager =
      std::make_shared<FastExplorationManager>();
  FastExplorationFSM expl_fsm;
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  std::unique_ptr<ExplorationRuntime> runtime(new ExplorationRuntime());
  auto exitWithoutRuntimeTeardown = [&runtime]() {
    (void)runtime.release();
    return 0;
  };

  // [race: name-collision-shutdown] If a second node registers the same name
  // (/exploration_node) on the shared roscore while this one is still in
  // main(), the master sends this node a shutdown that roscpp processes
  // asynchronously on its PollManager thread -- concurrently with the *->init()
  // sequence below. lio_interface->init() alone can block up to 20s
  // (waitForMessage 10s + 10s), so the exposure window is wide. Left
  // unguarded this raced into two crashes: publishing on a Publisher that
  // PollManager had already invalidated (ROS_ASSERT in FSM::init ->
  // publishEarlyFinishStatus), and a message_filters::Synchronizer destructor
  // locking a mutex PollManager had already destroyed (boost::lock_error).
  // Checking ros::ok() between steps and catching what a shutdown-induced
  // throw would otherwise turn into std::terminate lets this be a clean exit
  // instead.
  try {
    runtime->lio_interface->init(nh);
    if (!ros::ok()) return exitWithoutRuntimeTeardown();
    runtime->graph->init(nh, runtime->lio_interface,
                         runtime->parallel_path_finder);
    if (!ros::ok()) return exitWithoutRuntimeTeardown();
    runtime->parallel_path_finder->init(nh, runtime->lio_interface);
    if (!ros::ok()) return exitWithoutRuntimeTeardown();
    runtime->planner_manager->initPlanModules(
        nh, runtime->parallel_path_finder, runtime->graph);
    if (!ros::ok()) return exitWithoutRuntimeTeardown();
    runtime->frontier_manager->init(nh, runtime->lio_interface,
                                    runtime->graph);
    if (!ros::ok()) return exitWithoutRuntimeTeardown();
    runtime->explore_manager->initialize(
        nh, runtime->frontier_manager, runtime->planner_manager);
    if (!ros::ok()) return exitWithoutRuntimeTeardown();
    runtime->expl_fsm.init(nh, runtime->explore_manager);
    if (!ros::ok()) return exitWithoutRuntimeTeardown();

    ros::Duration(1.0).sleep();
    ros::spin();
    // ros::spin() only returns after shutdown. Do not destruct the planner graph
    // after roscpp has already invalidated its callbacks and worker resources.
    exitWithoutRuntimeTeardown();
    ros::shutdown();
  } catch (const std::exception &e) {
    // A shutdown mid-init (see above) can surface here as an exception (e.g.
    // roscpp tearing down a Publisher/Synchronizer out from under us). Normal
    // roslaunch teardown, not a startup failure -> return 0, not an error code.
    ROS_ERROR("[exploration_node] aborting startup: %s", e.what());
    return exitWithoutRuntimeTeardown();
  }
  return 0;
}
