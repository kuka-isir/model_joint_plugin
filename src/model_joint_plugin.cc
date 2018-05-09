#include "model_joint_plugin/model_joint_plugin.hh"
#include "joint_state.pb.h" // generated protobuf header

namespace gazebo
{

ModelJointPlugin::ModelJointPlugin()
: cmd(new joint_state_msgs::msgs::JointState)
, state(new joint_state_msgs::msgs::JointState)
{}
    
void ModelJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    n_status_sent = 0;
    this->model = _parent;
    this->world = model->GetWorld();

    int i=0;
    for(auto j:model->GetJoints())
    {
        state->add_position(0.0);
        state->add_velocity(0.0);
        state->add_effort(0.0);
        cmd->add_position(0.0);
        cmd->add_velocity(0.0);
        cmd->add_effort(0.0);
        state->add_name(j->GetName());
        cmd->add_name(j->GetName());
        joint_idx_map[j->GetName()] = i;
        i++;
    }

    //state->set_allocated_time(&time);

    node = transport::NodePtr(new transport::Node());
    node->Init();
    gazebo::printVersion();

    std::cout <<"\x1B[32m[[--- ["<<model->GetName()<<"] Loading Model Joint Plugin---]]\033[0m"<<std::endl;

    statePub = node->Advertise<joint_state_msgs::msgs::JointState>("~/" + model->GetName() + "/joint_states");
    cmdSub = node->Subscribe("~/" + model->GetName() + "/joint_states_command", &ModelJointPlugin::onCommand,this/*,true*/);

    this->world_up_end = event::Events::ConnectWorldUpdateEnd(std::bind(&ModelJointPlugin::WorldUpdateEnd, this));
}

void ModelJointPlugin::onCommand(ConstJointStatePtr& cmd_in)
{
    {
//         std::cout << "wait...";
        std::lock_guard<std::mutex> lk(mtx);
        *cmd = *cmd_in;
        flag = true;
    }
//     std::cout << "New Command : "<<world->GetRealTime()<<std::endl;
    cmd_cond.notify_one();
}

// void ModelJointPlugin::WorldUpdateBegin() {}

void ModelJointPlugin::WorldUpdateEnd()
{
    std::unique_lock<std::mutex> lk(mtx);
    // Here we get all the cmd from users, if we have sent at least N status
    if(statePub->HasConnections() && n_status_sent >= nb_status_needed_to_wait_for_cmd)
    {
        cmd_cond.wait(lk);
        flag = false;
        //std::cout << "Waiting on command..."<<std::endl;
        // BUG: Waiting with chrono is slow ! (gz goes down to 0.01)
        // if(cmd_cond.wait_for(lk,std::chrono::microseconds(2000000)) == std::cv_status::timeout)
        // {
        //     gzerr << " Timeout, connection lost or update rate too slow !"<<std::endl;
        // }
    }
    
    int i=0;
    for(auto j:model->GetJoints())
    {
        state->set_position(i, j->GetAngle(0).Radian());
        state->set_velocity(i, j->GetVelocity(0));
        state->set_effort(i, j->GetForce(0u));
        i++;
    }
    
    auto joints = model->GetJoints();
    for(int i=0; i<cmd->position_size(); ++i)
    {
#ifdef GAZEBO_GREATER_6
        joints[joint_idx_map[cmd->name(i)]]->SetPosition(0,cmd->position(i));
#else
        joints[joint_idx_map[cmd->name(i)]]->SetAngle(0,cmd->position(i));
#endif
    }
    for(int i=0; i<cmd->velocity_size(); ++i)
    {
        joints[joint_idx_map[cmd->name(i)]]->SetVelocity(0,cmd->velocity(i));
    }
    for(int i=0; i<cmd->effort_size(); ++i)
    {
//             std::cout <<cmd->name(i)<<" ("<<joint_idx_map[cmd->name(i)]<<") --> "<<cmd->effort(i)<<"N.m"<<std::endl;
        joints[joint_idx_map[cmd->name(i)]]->SetForce(0,cmd->effort(i));
    }
    
    lk.unlock();

    

    if(statePub->HasConnections())
    {
        statePub->Publish(*state/*,true*/);
        n_status_sent++;
    }else{
        n_status_sent=0;
    }

    // std::cout << "sta <-- "<<n_status_sent << std::endl;
}

}
