/*********************************************************************
 * Author: lyy@<1031016671@qq.com>
 *********************************************************************/

#ifndef EDGE_VELOCITY_GOAL_H
#define EDGE_VELOCITY_GOAL_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>


#include <iostream>

namespace teb_local_planner
{

  
/**
 * @class EdgeVelocityGoal
 * @brief Edge defining the cost function for limiting rotation at final goal.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [v,omega]^T ) \cdot weight \f$. \n
 * \e v is calculated using the difference quotient and the position parts of both poses. \n
 * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational velocity and
 * the second one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */  
class EdgeVelocityGoal : public BaseTebMultiEdge<1, double>
{
public:
  
  /**
   * @brief Construct edge.
   */	      
  EdgeVelocityGoal()
  {
    this->resize(1); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }
  
  /**
   * @brief Actual cost function
   */  
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocityGoal()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    
    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    const double omega = angle_diff / deltaT->estimate();
  
    // _error[0] = penaltyBoundToInterval(omega, -0.1, 0.1, 0.0);
    _error[0] = fabs(omega);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocityGoal::computeError() _error[0]=%f\n",_error[0]);
    // printf("+++++++++++++ EdgeVelocityGoal::computeError() _error[0]=%f\n",_error[0]);
  }
 
  
public:
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
} // end namespace

#endif
