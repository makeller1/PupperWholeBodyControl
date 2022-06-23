#include "worldPlugin.hpp"

using namespace gazebo;

//////////////////////////////////////////////////
/*
This is skeleton code for a Gazebo world plugin if needed in the future. 
It is not currently tied to the world Pupper is launched in. If this plugin
is used, we will have to launch Gazebo and the Pupper in a different way than
the current load_pupper.launch implementation. 
*/
void worldPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{	std::cout << "WorldPublisher: Entering Load()" << std::endl;
    this->world = _parent;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&worldPlugin::OnUpdate, this));
    PreviusRefTime = 0;
}

/////////////////////////////////////////////////
void worldPlugin::Init()
 {	std::cout << "WorldPublisher: Entering Init()" << std::endl;
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("EmptyWorld");
    this->pub = node->Advertise<gazebo::msgs::Time>("~/WorldTime_topic");
 }

//////////////////////////////////////////////////
void worldPlugin::OnUpdate()
{
    //std::cout << "WorldPublisher: Entering OnUpdate()" << std::endl;
    tmpTime = this->world->SimTime().Double() ;
    if ( (tmpTime- PreviusRefTime) <= REFTIME )
        return;
    PreviusRefTime = tmpTime;
    //msg.set_sec(this->world->GetSimTime().sec);
    //msg.set_nsec(this->world->GetSimTime().nsec);
    gazebo::msgs::Set(&msg, this->world->SimTime());
    this->pub->Publish(msg);
    // std::cout << "time message published" << std::endl;
}

GZ_REGISTER_WORLD_PLUGIN(worldPlugin)