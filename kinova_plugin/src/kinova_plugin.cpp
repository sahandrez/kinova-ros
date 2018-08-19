#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#define HOLD_FRAMES 100

namespace gazebo
{
        class ModelPush : public ModelPlugin
        {
		private: 
			ros::NodeHandle * n;
			ros::Publisher joint_state_pub;
			ros::Subscriber joint_effort_sub;
			bool applyPastCommand;
			int pastCommandCounter;
			std::vector<float> pastCommand;

		public: void JointEffortCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
		{
			this->applyPastCommand = true;
			this->pastCommandCounter = 0;
			// for (int i = 0; i < msg->data.size(); i++)
			// {
			// 	std::cout << msg->data[i] << std::endl;
			// }	
			// std::cout << "===============================" << std::endl;
			physics::Joint_V joints = this->model->GetJoints();
			this->pastCommand.clear();

			this->pastCommand.push_back(0);
			for (int i = 0; i < msg->data.size(); i++) this->pastCommand.push_back(msg->data[i]);
			
		}

                public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
                                {
                                        // Store the pointer to the model
                                        this->model = _parent;

                                        // Listen to the update event. This event is broadcast every
                                        // simulation iteration.
                                        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                                                        boost::bind(&ModelPush::OnUpdate, this, _1));
					this->n = new ros::NodeHandle("kinova_plugin");
					this->joint_state_pub = n->advertise<std_msgs::Float32MultiArray>("j2n6s300/joint_state", 1);
					this->joint_effort_sub = n->subscribe("j2n6s300/joint_effort", 1, &ModelPush::JointEffortCallback, this);
					this->pastCommandCounter = 0;
					this->applyPastCommand = false;

					ROS_INFO("Finished loading Kinova Jaco Plugin.");
                                }

                                // Called by the world update start event
                public: void OnUpdate(const common::UpdateInfo & _info)/*_info*/
                                {
					std::vector<float> ang_arr;
					std::vector<float> vel_arr;
					std::vector<float> state_arr;
                                        physics::Joint_V joints = this->model->GetJoints();

					if (this->applyPastCommand)
					{
						for (int i = 0; i < joints.size() && i < this->pastCommand.size(); i++)
						{
						     // Force is additive (multiple calls to SetForce to the same joint in the same time step 
						     // will accumulate forces on that Joint)?
						     joints[i]->SetForce(0, this->pastCommand[i]);
						}
						this->pastCommandCounter += 1;
						if (this->pastCommandCounter >= HOLD_FRAMES) 
						{ 
							this->applyPastCommand = false;
						}
					}

                                        for (int i = 1; i < joints.size(); i++)
                                        {
                                                //std::cout << joints[i]->GetAngle(0) << " blarg" << std::endl;
						ang_arr.push_back( joints[i]->GetAngle(0).Radian() );
						vel_arr.push_back( joints[i]->GetVelocity(0));
                                        }
                                        //concatenate two vectors
                                        state_arr.reserve( ang_arr.size() + vel_arr.size() + 1 );
					state_arr.insert(state_arr.end(), ang_arr.begin(), ang_arr.end() );
					state_arr.insert(state_arr.end(), vel_arr.begin(), vel_arr.end() );
                                        state_arr.insert(state_arr.end(), _info.simTime.Float() );
                                        
					//std::cout << "----"<< _info.simTime.Float() << std::endl;
					
					std_msgs::Float32MultiArray msg;
					msg.data = state_arr;
					
					this->joint_state_pub.publish(msg);
                                        
                                }

                                // Pointer to the model
                private: physics::ModelPtr model;

                                 // Pointer to the update event connection
                private: event::ConnectionPtr updateConnection;
        };

        // Register this plugin with the simulator
        GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

