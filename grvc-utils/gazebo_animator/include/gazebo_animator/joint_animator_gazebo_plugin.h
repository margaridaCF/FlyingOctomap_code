//----------------------------------------------------------------------------------------------------------------------
// GRVC Utils
// Based on John Hsu's GazeboRosJointTrajectory plugin
//----------------------------------------------------------------------------------------------------------------------
#ifndef GAZEBO_ANIMATOR_GAZEBO_PLUGIN_H
#define GAZEBO_ANIMATOR_GAZEBO_PLUGIN_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <boost/thread.hpp>

namespace gazebo {

class JointAnimatorGazeboPlugin : public ModelPlugin {
public:

    JointAnimatorGazeboPlugin();

    virtual ~JointAnimatorGazeboPlugin();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:

    void SetJointState(const sensor_msgs::JointState::ConstPtr& _joint_state);

    physics::WorldPtr world_;
    physics::ModelPtr model_;

    common::Time last_time_;

    ros::NodeHandle* rosnode_;
    ros::Subscriber sub_;

    void QueueThread();
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;

    std::map<std::string, double> joint_positions_;
}; 

}  // namespace gazebo
#endif  // GAZEBO_ANIMATOR_GAZEBO_PLUGIN_H
