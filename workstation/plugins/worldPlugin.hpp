#ifndef _WORLD_TIME_PUBLISHER_HH
#define _WORLD_TIME_PUBLISHER_HH

#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/msgs/msgs.hh"

#define REFTIME 0.001 // ~ 1000Hz

namespace gazebo {

/// A Gazebo plugin for publishing simulation time at rate according to REFTIME.
class worldPlugin: public WorldPlugin
{
public:
    ///  Standard Load.
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    ///  Standard Init.
    void Init();

    ///  Update function for publishing messages on a topic.
    void OnUpdate();

private: 
    ///  Remembering previus simulation time.
    double PreviusRefTime;

    ///  Copy of current simulation time.
    double tmpTime;

    ///  Pointer to the world.
    physics::WorldPtr world;

    ///  Standard connection pointer.
    event::ConnectionPtr updateConnection;

    ///  Transport node pointer.
    gazebo::transport::NodePtr node;

    ///  Transport publisher pointer.
    gazebo::transport::PublisherPtr pub;

    ///  Standard double vector message.
    gazebo::msgs::Time msg;
};

} // end namespace gazebo

#endif