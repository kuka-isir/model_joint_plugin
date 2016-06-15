#include "manipulator_sim/manipulator_sim.hh"

using namespace RTT;
using namespace std;
using namespace RTT::os;
using namespace Eigen;

ManipulatorSim::ManipulatorSim(const std::string& name):
    RTT::TaskContext(name)
{
    this->addPort("JointPosition",port_joint_position_out).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_out).doc("Current joint velocities");
    this->addPort("JointTorque",port_joint_torque_out).doc("Current joint torques");

    this->addPort("JointPositionCommand",port_joint_position_cmd_in).doc("Command joint positions");
    this->addPort("JointVelocityCommand",port_joint_velocity_cmd_in).doc("Command joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_cmd_in).doc("Command joint torques");

    this->addProperty("argv",argv).doc("You can pass arguments to gazebo setupclient");
    this->addAttribute("pos",jnt_pos_out);

}
void ManipulatorSim::gazeboStateCallback(ManipulatorSim::ConstJointStatePtr& _msg)
{
    static bool rec_one = false;
    joint_state_msgs::msgs::JointState msg_out;
    // Dump the message contents to stdout.
//     std::cout << _msg->time().sec()<<" " <<_msg->time().nsec()<<std::endl;

    size_t pos_size = _msg->position().size();
    size_t vel_size = _msg->velocity().size();
    size_t eff_size = _msg->effort().size();

    if(!rec_one)
    {


        jnt_pos_cmd_in.setZero(pos_size);
        jnt_vel_cmd_in.setZero(vel_size);
        jnt_trq_cmd_in.setZero(eff_size);

        jnt_pos_out.setZero(pos_size);
        jnt_vel_out.setZero(vel_size);
        jnt_trq_out.setZero(eff_size);

        port_joint_position_out.setDataSample(jnt_pos_out);
        port_joint_velocity_out.setDataSample(jnt_vel_out);
        port_joint_torque_out.setDataSample(jnt_trq_out);
                
        rec_one = true;
        this->start();
    }

    jnt_pos_out = Map<const VectorXd>(_msg->position().data(),pos_size);
    jnt_vel_out = Map<const VectorXd>(_msg->velocity().data(),vel_size);
    jnt_trq_out = Map<const VectorXd>(_msg->effort().data(),eff_size);
    
    if(port_joint_position_cmd_in.read(jnt_pos_cmd_in)  == RTT::NewData)
        for(int i=0;i<jnt_pos_cmd_in.size();++i)
            msg_out.add_position(jnt_pos_cmd_in[i]);
        
    if(port_joint_velocity_cmd_in.read(jnt_vel_cmd_in)  == RTT::NewData)
        for(int i=0;i<jnt_vel_cmd_in.size();++i)
            msg_out.add_velocity(jnt_vel_cmd_in[i]);
        
    if(port_joint_torque_cmd_in.read(jnt_trq_cmd_in)    == RTT::NewData)
        for(int i=0;i<jnt_trq_cmd_in.size();++i)
            msg_out.add_effort(jnt_trq_cmd_in[i]);
    
    port_joint_position_out.write(jnt_pos_out);
    port_joint_velocity_out.write(jnt_vel_out);
    port_joint_torque_out.write(jnt_trq_out);
    
    gz_state_pub->Publish(msg_out,true);
}

bool ManipulatorSim::configureHook()
{

    gazebo::client::printVersion();
    gazebo::client::setup(argv);

    gz_node.reset(new gazebo::transport::Node());
    gz_node->Init();
    // Listen to Gazebo world_stats topic
    gz_state_sub = gz_node->Subscribe("~/" + this->getName() + "/joint_states", &ManipulatorSim::gazeboStateCallback,this/*,true*/);
    gz_state_pub = gz_node->Advertise<joint_state_msgs::msgs::JointState>("~/" + this->getName() + "/joint_states_command");

    return true;
}
bool ManipulatorSim::startHook()
{
    return true;
}

void ManipulatorSim::updateHook()
{

}
void ManipulatorSim::stopHook()
{
    Logger::In(this->getName());
    log(Info) << "stopHook()" << endlog();

}

void ManipulatorSim::cleanupHook()
{
    Logger::In(this->getName());
    log(Info) << "cleanupHook() --> Shutting down gazebo...";

    gazebo::client::shutdown();

    log() << "done."<<endlog();
}

// Let orocos know how to create the component
ORO_CREATE_COMPONENT(ManipulatorSim)
