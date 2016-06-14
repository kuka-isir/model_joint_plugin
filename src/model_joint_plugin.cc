#include "model_joint_plugin/model_joint_plugin.hh"
namespace gazebo{

void ModelJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    n_status_sent = 0;
    this->model = _parent;
    this->world = model->GetWorld();
    
    for(auto j:model->GetJoints())
    {
        state.add_position(0.0);
        state.add_velocity(0.0);
        state.add_effort(0.0);
        cmd.add_position(0.0);
        cmd.add_velocity(0.0);
        cmd.add_effort(0.0);
        state.add_name(j->GetName());
        cmd.add_name(j->GetName());
    }
    
    state.set_allocated_time(&time);
    
    node = transport::NodePtr(new transport::Node());
    node->Init();
    
    statePub = node->Advertise<joint_state_msgs::msgs::JointState>("~/joint_states");
    cmdSub = node->Subscribe("~/joint_states_command", &ModelJointPlugin::onCommand,this/*,true*/);

    this->world_up_begin = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelJointPlugin::WorldUpdateBegin, this));
    this->world_up_end = event::Events::ConnectWorldUpdateEnd(std::bind(&ModelJointPlugin::WorldUpdateEnd, this));
}

void ModelJointPlugin::onCommand(ConstJointStatePtr& cmd_in)
{
    {
        std::lock_guard<std::mutex> lk(mtx);
        cmd = *cmd_in;
    }
    std::cout << "New Command : "<<world->GetRealTime()<<std::endl;
    cmd_cond.notify_one();
}

void ModelJointPlugin::WorldUpdateBegin() {}

void ModelJointPlugin::WorldUpdateEnd()
{

    int i=0;
    common::Time t  = world->GetRealTime();
    time.set_sec(t.sec);
    time.set_nsec(t.nsec);
    
            
    for(auto j:model->GetJoints())
    {
        state.set_position(i, j->GetAngle(0).Radian());
        state.set_velocity(i, j->GetVelocity(0));
        state.set_effort(i, j->GetForce(0u));
        i++;
    }
    
    statePub->Publish(state,true);
    
    if(statePub->HasConnections())
        n_status_sent++;
    else
        n_status_sent=0;
    
    //std::cout << "New status sent"<<std::endl;
        {
        std::unique_lock<std::mutex> lk(mtx);
        // Here we get all the cmd from users, if we have sent at least N status
        if(statePub->HasConnections() && n_status_sent >= nb_status_needed_to_wait_for_cmd)
        {
            //std::cout << "Waiting on command..."<<std::endl;
//                 if(cmd_cond.wait_for(lk,std::chrono::microseconds(2000000)) == std::cv_status::timeout)
//                 {
//                     gzerr << " Timeout, connection lost or update rate too slow !"<<std::endl;
//                 }
            
            std::cout << "gz real     : "<<world->GetRealTime()<<std::endl;
        }
        
        cmd.position();
        cmd.velocity();
        cmd.effort();
    }
}
    
}