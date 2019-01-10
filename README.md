
## What is this

- sample code of [Setting Velocity on Joints and Links (Gazebo ver7.0)](http://gazebosim.org/tutorials?tut=set_velocity) changing for Gazebo version 9.0

## Bebug from Gazebo version 7.0

* math::Vector3 -> ignition::math::Vector3d
* math::Pose -> ignition::math::Pose3d

* physics::PhysicsEnginePtr engine = world->SetPhysicsEnabled(true);
    * reference
        * <http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1World.html>
        * <http://docs.ros.org/melodic/api/gazebo_plugins/html/gazebo__ros__joint__pose__trajectory_8cpp_source.html>

```
//physics::PhysicsEnginePtr engine = world->SetPhysicsEnabled(true);
physics::PhysicsEnginePtr engine = world->Physics();
world->SetPhysicsEnabled(true);
```

* worldForce.X(this->controllers[0].Update(linearError.X(), dt));
    * reference
        * <https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Vector3.html>

```
//worldForce.X = this->controllers[0].Update(linearError.X, dt);
worldForce.X(this->controllers[0].Update(linearError.X(), dt));
```




