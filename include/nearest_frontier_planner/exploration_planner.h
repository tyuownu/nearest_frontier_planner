#ifndef EXPLORATION_PLANNER_H_
#define EXPLORATION_PLANNER_H_

#define EXPL_TARGET_SET 1
#define EXPL_FINISHED   2
#define EXPL_WAITING    3
#define EXPL_FAILED     4

#include <string>

#include <nearest_frontier_planner/grid_map.h>


// The base class for all exploration planners
class ExplorationPlanner {
 public:
  virtual ~ExplorationPlanner() {}
  virtual int findExplorationTarget(GridMap* map,
      unsigned int start, unsigned int &goal) = 0;
};

#endif  // end EXPLORATION_PLANNER_H_
