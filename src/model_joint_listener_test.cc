#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "joint_state.pb.h" // generated protobuf header

typedef const boost::shared_ptr<
    const joint_state_msgs::msgs::JointState>
    ConstJointStatePtr;

gazebo::transport::PublisherPtr pub;
/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstJointStatePtr&_msg)
{
  // Dump the message contents to stdout.
  std::cout << _msg->time().sec()<<" " <<_msg->time().nsec()<<std::endl;
  pub->Publish(*_msg,true);
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::printVersion();
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/kuka_kr10r1100sixx/joint_states", cb/*,true*/);
  pub = node->Advertise<joint_state_msgs::msgs::JointState>("/gazebo/default/kuka_kr10r1100sixx/joint_states_command");

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
