#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <math.h>       /* sin, cos, tan */

namespace gazebo
{
    class PolarPerson : public ModelPlugin
    {

    private:
        int cont;
        int state;
        double vel;
        double vx, vy;
        ignition::math::Pose3d pose;

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr)
        {
            this->model = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&PolarPerson::OnUpdate, this, _1));

            std::cout << "Loading person to follow at " 
                << this->model->WorldPose() << std::endl;

            // Init values
            this->cont = 0;
            this->vel = 0.5 + double(rand()) / RAND_MAX * 1.0;
        }

    public:
        void OnUpdate(const common::UpdateInfo &)
        {
            pose = this->model->WorldPose();
            pose.Rot().Euler(0, 0, pose.Rot().Yaw());
            this->model->SetWorldPose(pose);

            if (cont <= 250) {
                // Init vels
                this->model->SetAngularVel(ignition::math::Vector3d(0, 0, vel));
            } else if (cont > 250 && cont <= 1000) {
                this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
            } else {
                this->cont = 0;
                this->vel = 0.5 + double(rand()) / RAND_MAX * 1.0;
            }
            this->cont = this->cont + 1;
            vx = 0*cos(pose.Rot().Yaw()) - 0.5*sin(pose.Rot().Yaw());
            vy = 0*sin(pose.Rot().Yaw()) + 0.5*cos(pose.Rot().Yaw());
            this->model->SetLinearVel(ignition::math::Vector3d(-vx, -vy, 0));
            //std::cout << "Cont " << cont << " at [" << pose.Pos() << "]" << std::endl;
        }

    private:
        physics::ModelPtr model;

    private:
        event::ConnectionPtr updateConnection;
    };
    GZ_REGISTER_MODEL_PLUGIN(PolarPerson)
}
