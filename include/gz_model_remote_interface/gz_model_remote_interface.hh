// gz_model_remote_interface - ISIR Mon 06 Jun 2016 03:35:07 PM CEST
// Copyright (c) Antoine Hoarau, All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3.0 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library.

#ifndef __MANIPULATOR_SIM_HPP__
#define __MANIPULATOR_SIM_HPP__

// Orocos
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
// Eigen
#include <Eigen/Dense>
// RTT-ROS Utilities
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "model_joint_plugin/joint_state.pb.h"


class GazeboModelRemoteInterface : public RTT::TaskContext{

    typedef const boost::shared_ptr<
        const joint_state_msgs::msgs::JointState>
    ConstJointStatePtr;

    public:
        GazeboModelRemoteInterface(const std::string& name);
        virtual ~GazeboModelRemoteInterface(){};
        virtual void updateHook();
        virtual bool configureHook();
        virtual void cleanupHook();
        virtual bool startHook();
        virtual void stopHook();
        void gazeboStateCallback(GazeboModelRemoteInterface::ConstJointStatePtr& _msg);
    protected:
        gazebo::transport::PublisherPtr gz_state_pub;
        std::vector<std::string> argv;
        gazebo::transport::NodePtr gz_node;
        gazebo::transport::SubscriberPtr gz_state_sub;
        // Input ports
        RTT::OutputPort<Eigen::VectorXd>  port_joint_position_out,
                                         port_joint_velocity_out,
                                         port_joint_torque_out;
        // Some input variables
        Eigen::VectorXd jnt_pos_cmd_in,
                        jnt_vel_cmd_in,
                        jnt_trq_cmd_in;
        // Output ports
        RTT::InputPort<Eigen::VectorXd> port_joint_position_cmd_in,
                                         port_joint_velocity_cmd_in,
                                         port_joint_torque_cmd_in;
        // Some output variables
        Eigen::VectorXd jnt_pos_out,
                        jnt_vel_out,
                        jnt_trq_out;
};

#endif
