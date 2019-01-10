#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>

#ifndef GAZEBO_TUTORIALS_SET_VELOCITY_PID_LINK_HH_
#define GAZEBO_TUTORIALS_SET_VELOCITY_PID_LINK_HH_

using namespace gazebo;

//////////////////////////////////////////////////
// \brief use a PID controller to apply forces to achieve a target velocity
class PIDLinkVelocityController
{
public:
    PIDLinkVelocityController()
    {
    }

public:
    ~PIDLinkVelocityController()
    {
        this->Stop();
    }

public:
    void Start(physics::LinkPtr _link, ignition::math::Vector3d _linearVel,
        ignition::math::Vector3d _angularVel, double _maxForce, double _maxTorque)
    {
        this->Stop();
        this->link = _link;
        this->targetLinearVel = _linearVel;
        this->targetAngularVel = _angularVel;

        // Hard coded gains. Tune these for your own application!
        double linear_p = 100.0;
        double linear_i = 0.0;
        double linear_d = 0.0;
        double linear_imax = 123456789.0;
        double angular_p = 100.0;
        double angular_i = 0.0;
        double angular_d = 0.0;
        double angular_imax = 123456789.0;

        // Add a PID controller for each DoF
        for (int i = 0; i < 3; i++) {
            common::PID controller_translation(linear_p, linear_i, linear_d,
                linear_imax, -linear_imax, _maxForce, -_maxForce);
            common::PID controller_rotation(angular_p, angular_i, angular_d,
                angular_imax, -angular_imax, _maxTorque, -_maxTorque);
            this->controllers.push_back(controller_translation);
            this->controllers.push_back(controller_rotation);
        }

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&PIDLinkVelocityController::Update, this,
                std::placeholders::_1));
    }

public:
    void Stop()
    {
        this->lastSimTime.Set(0.0);
        this->link.reset();
        this->controllers.clear();
        this->updateConnection.reset();
    }

public:
    void Update(const common::UpdateInfo& _info)
    {
        common::Time curTime = _info.simTime;
        if (this->lastSimTime.Double() < 0.000001) {
            // First update, need a second one to get change time
        }
        if (curTime < this->lastSimTime) {
            // Time moved backwards (World reset?)
            this->Stop();
        } else {
            // Get change in time between updates
            double dt = (curTime - lastSimTime).Double();

            // Calculate the error between actual and target velocity
            ignition::math::Vector3d curLinearVel = this->link->WorldLinearVel();
            ignition::math::Vector3d curAngularVel = this->link->WorldAngularVel();
            ignition::math::Vector3d linearError = curLinearVel - this->targetLinearVel;
            ignition::math::Vector3d angularError = curAngularVel - this->targetAngularVel;

            // Get forces to apply from controllers
            ignition::math::Vector3d worldForce;
            ignition::math::Vector3d worldTorque;
            //worldForce.X = this->controllers[0].Update(linearError.X, dt);
            //worldTorque.X = this->controllers[1].Update(angularError.X, dt);
            //worldForce.Y = this->controllers[2].Update(linearError.Y, dt);
            //worldTorque.Y = this->controllers[3].Update(angularError.Y, dt);
            //worldForce.Z = this->controllers[4].Update(linearError.Z, dt);
            //worldTorque.Z = this->controllers[5].Update(angularError.Z, dt);

            worldForce.X(this->controllers[0].Update(linearError.X(), dt));
            worldTorque.X(this->controllers[1].Update(angularError.X(), dt));
            worldForce.Y(this->controllers[2].Update(linearError.Y(), dt));
            worldTorque.Y(this->controllers[3].Update(angularError.Y(), dt));
            worldForce.Z(this->controllers[4].Update(linearError.Z(), dt));
            worldTorque.Z(this->controllers[5].Update(angularError.Z(), dt));

            // Add those forces to the body
            this->link->AddForce(worldForce);
            this->link->AddTorque(worldTorque);
        }
        lastSimTime = curTime;
    }

private:
    physics::LinkPtr link;

private:
    std::vector<common::PID> controllers;

private:
    event::ConnectionPtr updateConnection;

private:
    ignition::math::Vector3d targetLinearVel;

private:
    ignition::math::Vector3d targetAngularVel;

private:
    common::Time lastSimTime;
};

#endif
