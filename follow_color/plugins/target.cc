#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

namespace gazebo
{
    class Target : public ModelPlugin
    {

    private:
        ignition::math::Pose3d pose;

    private:
        bool flag;

    private:
        int state;

    private:
        double vel1;

    private:
        double vel2;

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr)
        {
            this->model = _parent;
            flag = true;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Target::OnUpdate, this, _1));
            std::cout << "Loading target to follow" << std::endl;
            this->state = 0;
            this->vel1 = 0.5 + double(rand()) / RAND_MAX * 1.0;
            this->vel2 = (0.5 + double(rand()) / RAND_MAX * 1.0) * -1;
        }

    public:
        void OnUpdate(const common::UpdateInfo &)
        {
             pose = this->model->WorldPose();

            if (flag)
            {
                this->model->SetLinearVel(ignition::math::Vector3d(1, 0, this->vel1));
            }
            if (pose.Pos().Z() >= 8)
            {
                pose.Pos().Z() = 8;
                this->model->SetWorldPose(pose);
                flag = false;
            }
            if (!flag)
            {
                this->model->SetLinearVel(ignition::math::Vector3d(-1, 0, this->vel2));
            }
            if (pose.Pos().Z() <= 0.5)
            {
                pose.Pos().Z() = 0.5;
                this->model->SetWorldPose(pose);
                flag = true;
            }
//            pose = this->model->WorldPose();
//            std::cout << "Status " << state << " at [" << pose.Pos().X() << ", " << pose.Pos().Y() << ", " << pose.Pos().Z() << "]" << std::endl;
//            switch(state){
//                case 0:
//                    this->model->SetLinearVel(ignition::math::Vector3d(this->vel1, 0, 0));
//                    if (pose.Pos().X() <= -4){
//                        pose.Pos().X() = -4;
//                        this->model->SetWorldPose(pose);
//                        state = 1;
//                        std::cout << "STATE TO 1" << std::endl;
//                    }
//                case 1:
//                    this->model->SetLinearVel(ignition::math::Vector3d(this->vel2, 0, 0));
//                    if (pose.Pos().X() >= 4){
//                        pose.Pos().X() = 4;
//                        this->model->SetWorldPose(pose);
//                        state = 2;
//                    }
//                case 2:
//                    this->model->SetLinearVel(ignition::math::Vector3d(this->vel1, 0, 0));
//                    if (pose.Pos().X() >= -0.1 && pose.Pos().X() <= 0.1){
//                        pose.Pos().X() = 0;
//                        this->model->SetWorldPose(pose);
//                        state = 3;
//                    }
//                case 3:
//                    this->model->SetLinearVel(ignition::math::Vector3d(0, this->vel1, 0));
//                    if (pose.Pos().Y() <= -4){
//                        pose.Pos().Y() = -4;
//                        this->model->SetWorldPose(pose);
//                        state = 4;
//                    }
//                case 4:
//                    this->model->SetLinearVel(ignition::math::Vector3d(0, this->vel2, 0));
//                    if (pose.Pos().Y() >= -4){
//                        pose.Pos().Y() = -4;
//                        this->model->SetWorldPose(pose);
//                        state = 0;
//                    }
////                case 5:
////                    this->model->SetLinearVel(ignition::math::Vector3d(0, this->vel1, 0));
////                    if (pose.Pos().Y() >= -0.1 && pose.Pos().Y() <= 0.1){
////                        pose.Pos().Y() = 0;
////                        this->model->SetWorldPose(pose);
////                        state = 0;
////                    }
//                default:
//                    break;
//            }
        }

    private:
        physics::ModelPtr model;

    private:
        event::ConnectionPtr updateConnection;
    };
    GZ_REGISTER_MODEL_PLUGIN(Target)
}