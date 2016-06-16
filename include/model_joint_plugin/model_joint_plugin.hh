#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "joint_state.pb.h"

namespace gazebo
{
    typedef const boost::shared_ptr<
  const joint_state_msgs::msgs::JointState>
    ConstJointStatePtr;

class ModelJointPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
public:
    void onCommand(ConstJointStatePtr& cmd_in);
public:
    void WorldUpdateBegin();
    void WorldUpdateEnd();

private:
    physics::ModelPtr model;
    joint_state_msgs::msgs::JointState cmd,state;
    gazebo::msgs::Time time;
    event::ConnectionPtr world_up_begin,world_up_end;
    transport::NodePtr node;
    transport::PublisherPtr statePub;
    transport::SubscriberPtr cmdSub;
    physics::WorldPtr world;
    std::mutex mtx;
    std::condition_variable cmd_cond,status_cond;
    const int nb_status_needed_to_wait_for_cmd = 10;
    int n_status_sent;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelJointPlugin)
}
