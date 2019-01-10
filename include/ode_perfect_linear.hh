#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math/Vector3.hh>

#ifndef GAZEBO_TUTORIALS_SET_VELOCITY_ODE_PERFECT_LINEAR_HH_
#define GAZEBO_TUTORIALS_SET_VELOCITY_ODE_PERFECT_LINEAR_HH_

namespace gazebo
{
//////////////////////////////////////////////////
/// \brief uses joint motors to perfectly control velocity (ODE only)
class OdePerfectLinearVelController
{
public:
    OdePerfectLinearVelController()
    {
    }

public:
    ~OdePerfectLinearVelController()
    {
        this->Stop();
    }

    /// \brief start controlling the linear velocity of an object
    /// \param[in] _link the link to move at the given velocity
    /// \param[in] _vel world velocity to move the link at
    /// \param[in] _maxForce Maximum force to apply each timestep
public:
    void Start(
        physics::LinkPtr _link, ignition::math::Vector3d _vel, double _maxForce)
    {
        this->Stop();
        // create prismatic joint with the world as a parent
        physics::ModelPtr model = _link->GetModel();
        physics::WorldPtr world = physics::get_world("default");
        //physics::PhysicsEnginePtr engine = world->SetPhysicsEnabled(true);
        physics::PhysicsEnginePtr engine = world->Physics();
        world->SetPhysicsEnabled(true);
        this->joint = engine->CreateJoint("prismatic");
        this->joint->SetName(model->GetName() + "__perfect_lin_joint__");
        physics::LinkPtr worldLink = boost::dynamic_pointer_cast<physics::Link>(
            world->BaseByName("world"));
        ignition::math::Pose3d jointOrigin;
        this->joint->Load(worldLink, _link, jointOrigin);
        this->joint->Init();
        double magnitude = _vel.Length();
        this->joint->SetAxis(0, _vel.Normalize());
        this->joint->SetParam("fmax", 0, _maxForce);
        this->joint->SetParam("vel", 0, magnitude);
    }

public:
    void Stop()
    {
        //remove the joint from the world
        if (joint) {
            this->joint->SetParam("fmax", 0, 0.0);
            this->joint->Detach();
            this->joint->Fini();
            this->joint.reset();
        }
    }

private:
    physics::JointPtr joint;
};
}  // namespace gazebo
#endif
