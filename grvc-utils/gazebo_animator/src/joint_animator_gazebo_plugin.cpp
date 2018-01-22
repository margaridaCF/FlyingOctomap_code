//----------------------------------------------------------------------------------------------------------------------
// GRVC Utils
// Based on John Hsu's GazeboRosJointTrajectory plugin
//----------------------------------------------------------------------------------------------------------------------
#include "gazebo_animator/joint_animator_gazebo_plugin.h"

namespace gazebo {

JointAnimatorGazeboPlugin::JointAnimatorGazeboPlugin() {}

JointAnimatorGazeboPlugin::~JointAnimatorGazeboPlugin() {
    this->rosnode_->shutdown();
    this->queue_.clear();
    this->queue_.disable();
    this->callback_queue_thread_.join();
    delete this->rosnode_;
}

void JointAnimatorGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model_ = _model;
    this->world_ = _model->GetWorld();

    std::string robot_namespace = "";
    if (_sdf->HasElement("robotNamespace")) {
        robot_namespace = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString() + "/";
    }

    std::string topic_name = "set_joint_state";
    if (_sdf->HasElement("topicName")) {
        topic_name = _sdf->GetElement("topicName")->GetValue()->GetAsString();
    }

    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = NULL;
        ros::init( argc, argv, "gazebo", ros::init_options::NoSigintHandler);
        gzwarn << "should start ros::init in simulation by using the system plugin\n";
    }

    this->rosnode_ = new ros::NodeHandle(robot_namespace);
    ros::SubscribeOptions joint_state_so = ros::SubscribeOptions::create<sensor_msgs::JointState>(
        topic_name, 10, boost::bind(&JointAnimatorGazeboPlugin::SetJointState, this, _1),
        ros::VoidPtr(), &this->queue_);
    this->sub_ = this->rosnode_->subscribe(joint_state_so);

    this->last_time_ = this->world_->GetSimTime();

    // start custom queue for joint state plugin ros topics
    this->callback_queue_thread_ = boost::thread(boost::bind(&JointAnimatorGazeboPlugin::QueueThread, this));
}

void JointAnimatorGazeboPlugin::SetJointState(const sensor_msgs::JointState::ConstPtr& _joint_state) {
    // copy joint configuration into a map
    for (unsigned int i = 0; i < _joint_state->name.size(); ++i) {
        joint_positions_[_joint_state->name[i]] = _joint_state->position[i];
    }
    model_->SetJointPositions(joint_positions_);
}

void JointAnimatorGazeboPlugin::QueueThread() {
    static const double timeout = 0.01;
    while (this->rosnode_->ok()) {
        this->queue_.callAvailable(ros::WallDuration(timeout));
    }
}

GZ_REGISTER_MODEL_PLUGIN(JointAnimatorGazeboPlugin);

}  // namespace gazebo
