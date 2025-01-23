
#include <rrt_planner/collision_detector.h>

namespace rrt_planner {

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap) {

        costmap_ = costmap->getCostmap();

        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();
    }

    bool CollisionDetector::inFreeSpace(const double* world_pos)
    {
  	/*------------------------*/
	unsigned int OBSTACLE_THRESHOLD = 140;
	
  	unsigned int map_x, map_y;
  	
  	costmap_->worldToMap(world_pos[0], world_pos[1], map_x, map_y);
  	
  	
  	// TO LOG THE COST AT SPECIFIC POINTS
  	/*unsigned int map_x2, map_y2;
  	costmap_->worldToMap(-1.0, -0.5, map_x2, map_y2);
  	std::cout << map_x2 << " --- " << map_y2 << "\n";
  	// std::cout << costmap_->getCost(map_x2, map_y2) << "\n";
  	ROS_INFO("%d", costmap_->getCost(map_x2, map_y2));*/
  	
  	
  	auto cost = costmap_->getCost(map_x, map_y);
  	
  	
  	
  	// ROS_INFO("World_Pos = (%f, %f)", world_pos[0], world_pos[1]);
  	// ROS_INFO("Map_Pos = (%d, %d)", map_x, map_y);
  	// ROS_INFO("x_Cells = %d", costmap_->getSizeInCellsX());
  	
  	// ROS_INFO("%d,  %d,  %d", costmap_2d::NO_INFORMATION, costmap_2d::FREE_SPACE, costmap_2d::LETHAL_OBSTACLE);
  	
  	//if(cost == costmap_2d::FREE_SPACE)
  	if(cost <= OBSTACLE_THRESHOLD)
  	{
  		return true;
  	}
  	
  	return false;
	/*------------------------*/
    }

    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {

        double dist = computeDistance(point_a, point_b);

        if (dist < resolution_) {
            return ( !inFreeSpace(point_b) ) ? true : false;

        } else {
            
            int num_steps = static_cast<int>(floor(dist/resolution_));

            double point_i[2];
            for (int n = 1; n <= num_steps; n++) {

                point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
                point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;

                if ( !inFreeSpace(point_i) ) return true;
            }
            
            return false;
        }

    }

};
