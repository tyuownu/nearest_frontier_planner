#include "nearest_frontier_planner/nearest_frontier_planner.h"

typedef std::multimap<double, unsigned int> Queue;
typedef std::pair<double, unsigned int> Entry;

int NearestFrontierPlanner::findExplorationTarget(GridMap* map,
    unsigned int start, unsigned int &goal) {
  // Create some workspace for the wavefront algorithm
  unsigned int map_size = map->getSize();
  double* plan = new double[map_size];
  for ( unsigned int i = 0; i < map_size; i++ ) {
    plan[i] = -1;
  }

  // Initialize the queue with the robot position
  Queue queue;
  Entry start_point(0.0, start);
  queue.insert(start_point);
  plan[start] = 0;

  Queue::iterator next;
  double distance;
  double linear = map->getResolution();
  bool found_frontier = false;
  int cell_count = 0;

  // Do full search with weightless Dijkstra-Algorithm
  while ( !queue.empty() ) {
    cell_count++;
    // Get the nearest cell from the queue
    next = queue.begin();
    distance = next->first;
    unsigned int index = next->second;
    queue.erase(next);

    // Add all adjacent cells
    if ( map->isFrontier(index) ) {
      // We reached the border of the map, which is unexplored terrain as well:
      found_frontier = true;
      goal = index;
      break;
    } else {
      unsigned int ind[4];

      ind[0] = index - 1;                // left
      ind[1] = index + 1;                // right
      ind[2] = index - map->getWidth();  // up
      ind[3] = index + map->getWidth();  // down

      for ( unsigned int it = 0; it < 4; it++ ) {
        unsigned int i = ind[it];
        if ( map->isFree(i) && plan[i] == -1 ) {
          queue.insert(Entry(distance+linear, i));
          plan[i] = distance+linear;
        }
      }
    }
  }

  ROS_DEBUG("Checked %d cells.", cell_count);
  delete[] plan;

  if ( found_frontier ) {
    return EXPL_TARGET_SET;
  } else {
    if ( cell_count > 50 )
      return EXPL_FINISHED;
    else
      return EXPL_FAILED;
  }
}
