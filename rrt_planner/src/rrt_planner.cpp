
#include <rrt_planner/rrt_planner.h>
#include <iostream>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++)
        {
			p_rand = sampleRandomPoint();
			nearest_node = nodes_[getNearestNodeId(p_rand)];
			p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate


            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new))
            {
                createNewNode(p_new, nearest_node.node_id);
            } else
            {
                continue;
            }

            if(k > params_.min_num_nodes)
            {
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance)
                {
                    return true;
                }
            }
        }

        return false;
    }
    
    void RRTPlanner::smoothPath(std::vector<geometry_msgs::PoseStamped>& plan)
    {
    	std::cout << "[SMOOTH PATH] ---------------------------------\n";
    	
    	double *start_pt = (double*)malloc(2 * sizeof(double));
    	double *inter_pt = (double*)malloc(2 * sizeof(double));
    	double *end_pt = (double*)malloc(2 * sizeof(double));
    	
    	tf2::Quaternion quat_tf;
    	geometry_msgs::Quaternion quat_msg;
    	
		for(int i = 0; i < plan.size(); ++i)
        {
        	start_pt[0] = plan[i].pose.position.x;
        	start_pt[1] = plan[i].pose.position.y;
        	
        	
        	if(i + 2 < plan.size())
        	{
        		end_pt[0] = plan[i + 2].pose.position.x;
        		end_pt[1] = plan[i + 2].pose.position.y;
        		
        		// If can skip a point without an obstacle in between
        		if(!collision_dect_.obstacleBetween(start_pt, end_pt))
        		{
        			double dx = end_pt[0] - start_pt[0];
        			double dy = end_pt[1] - start_pt[1];
        			
        			double yaw = std::atan2(dy, dx);  // Get yaw from atan2 using current point and previous point
          			quat_tf.setRPY(0., 0., yaw); // Convert RPY to quat
          			quat_msg = tf2::toMsg(quat_tf); // Convert Quat TF to msg
        			
        			// Coords of the point in between start_pt and end_pt
        			inter_pt[0] = start_pt[0] + dx * 0.5;
        			inter_pt[1] = start_pt[1] + dy * 0.5;
        			
        			if(collision_dect_.inFreeSpace(inter_pt))
        			{
        				// Set the middle position between the two points if it is in free space
        				plan[i + 1].pose.position.x = inter_pt[0];
        				plan[i + 1].pose.position.y = inter_pt[1];
        				
        				// Set the same orientation as the starting point
        				plan[i + 1].pose.orientation = quat_msg;
        			}
        		}
        	}	
        }
    }

    void RRTPlanner::optimizePath(std::vector<geometry_msgs::PoseStamped>& plan)
    {
    	std::cout << "[OPTIMIZE PATH] ---------------------------------\n";
    	
    	double *start_pt = (double*)malloc(2 * sizeof(double));
    	double *inter_pt = (double*)malloc(2 * sizeof(double));
    	double *end_pt = (double*)malloc(2 * sizeof(double));
    	
    	tf2::Quaternion quat_tf;
    	geometry_msgs::Quaternion quat_msg;
    	
    	int next_i = 0;
    	bool found_obstacle;
		for(int i = 0; i < plan.size(); ++i)
        {
        	found_obstacle = false;
        	start_pt[0] = plan[i].pose.position.x;
        	start_pt[1] = plan[i].pose.position.y;
        	
			for(int j = i + 2; j < plan.size(); ++j)
			{
			    end_pt[0] = plan[j].pose.position.x;
        		end_pt[1] = plan[j].pose.position.y;
        		
			    // If you can't connect the 2 points without an obstacle in between
        		if(collision_dect_.obstacleBetween(start_pt, end_pt) || j == plan.size() - 1)
        		{
					end_pt[0] = plan[j - 1].pose.position.x;
        			end_pt[1] = plan[j - 1].pose.position.y;
					
					found_obstacle = true;
					
					next_i = j - 1;
					break;
        		}	
			}
			
			// Set the middle points between i and next_i in a straight line
			int N = next_i - i + 1;
			
			double dx = (end_pt[0] - start_pt[0])/(N - 2.);
			double dy = (end_pt[1] - start_pt[1])/(N - 2.);
			
			double yaw = std::atan2(dy, dx);  // Get yaw from atan2 using current point and previous point
  			quat_tf.setRPY(0., 0., yaw); // Convert RPY to quat
  			quat_msg = tf2::toMsg(quat_tf); // Convert Quat TF to msg
			
			for(int m = 1; m <= N - 2; ++m)
			{
		        // Set the middle position between the two points if it is in free space
				plan[i + m].pose.position.x = start_pt[0] + dx * m;
				plan[i + m].pose.position.y = start_pt[1] + dy * m;
				
				// Set the same orientation as the starting point
				plan[i + m].pose.orientation = quat_msg;
			}
			
			if(found_obstacle)
				i = next_i;
        }
    }


    int RRTPlanner::getNearestNodeId(const double *point)
    {
		/*------------------------*/
		int node_id = 0;
		double min_dist = std::numeric_limits<double>::infinity();

		for(Node node : nodes_)
		{
			double dist = computeDistance(point, node.pos);
			if(dist < min_dist)
			{
				min_dist = dist;
				node_id = node.node_id;
			}
		}
		return node_id;
		
		/*------------------------*/
    }

    void RRTPlanner::createNewNode(const double* pos, int parent_node_id)
    {
        Node new_node;

        /*------------------------*/
		new_node.pos[0] = pos[0];
		new_node.pos[1] = pos[1];
		
		new_node.parent_id = parent_node_id;
		new_node.node_id = nodes_.size(); 	// So that there is a unique id for each one
	
        /*------------------------*/
        nodes_.emplace_back(new_node);
    }

    double* RRTPlanner::sampleRandomPoint()
    {
		/*------------------------*/
		double *rand_point_ = (double*)malloc(2 * sizeof(double));;
		
		    rand_point_[0] = random_double_x.generate();
		    rand_point_[1] = random_double_y.generate();
		/*------------------------*/

        return rand_point_;
    }

    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand)
    {
		/*------------------------*/
		double x_offset = point_rand[0] - point_nearest[0];
		double y_offset = point_rand[1] - point_nearest[1];
		
		double inv_magnitude = 1./sqrt(x_offset*x_offset + y_offset*y_offset);

		    candidate_point_[0] = point_nearest[0] + x_offset * params_.step * inv_magnitude;
		    candidate_point_[1] = point_nearest[1] + y_offset * params_.step * inv_magnitude;
		/*------------------------*/
        return candidate_point_;
    }

    const std::vector<Node>& RRTPlanner::getTree() {

        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {

        start_[0] = start[0];
        start_[1] = start[1];
    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

};
