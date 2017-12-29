#ifndef NEAREST_FRONTIER_PLANNER_H_
#define NEAREST_FRONTIER_PLANNER_H_

#define EXPL_TARGET_SET 1
#define EXPL_FINISHED   2
#define EXPL_WAITING    3
#define EXPL_FAILED     4

#include <nearest_frontier_planner/grid_map.h>


class NearestFrontierPlanner {
 public:
    NearestFrontierPlanner() {}
    ~NearestFrontierPlanner() {}

    int findExplorationTarget(GridMap* map, unsigned int start,
        unsigned int &goal);
};

#endif  // NEAREST_FRONTIER_PLANNER_H_
