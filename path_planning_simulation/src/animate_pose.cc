#include "common/CommonTypes.hh"
#include "common/Animation.hh"
#include "common/KeyFrame.hh"
#include "physics/Model.hh"
#include "gazebo.hh"
#include <ros/ros.h>

double getNumber(XmlRpc::XmlRpcValue& value)
{
    if( value.getType() == XmlRpc::XmlRpcValue::TypeInt ){
        return static_cast<int>(value);
    }else if( value.getType() == XmlRpc::XmlRpcValue::TypeDouble ){
        return static_cast<double>(value);
    }else{
        ROS_ERROR("BAD TYPE");
        return 0.0;
    }
}

namespace gazebo
{
  class AnimatePose : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      int argc = 1;
      char* argv = new char[15];
      strcpy(argv, "animate_pose");

      ros::init(argc, &argv, "animate_PP", ros::init_options::AnonymousName);
      ros::NodeHandle nh("/nav_experiments/scenario/objects");

      if(!nh.hasParam(_parent->GetName() + "/movement")){
          return;
      }
      XmlRpc::XmlRpcValue keyframes;
      nh.getParam(_parent->GetName() + "/movement", keyframes);
      ROS_ASSERT(keyframes.getType() == XmlRpc::XmlRpcValue::TypeArray);

      gazebo::common::PoseAnimationPtr anim(
          new gazebo::common::PoseAnimation("animation", 1000.0, true));

      gazebo::common::PoseKeyFrame *key;

      for (int32_t i=0; i<keyframes.size(); i++){
          XmlRpc::XmlRpcValue frame = keyframes[i];
          double t = getNumber(frame["t"]);
          anim->SetLength(t);
          key = anim->CreateKeyFrame( t );
          
          XmlRpc::XmlRpcValue triple = frame["xyz"];
          key->SetTranslation(math::Vector3( getNumber(triple[0]), getNumber(triple[1]), getNumber(triple[2])));
          triple = frame["rpy"];
          key->SetRotation(math::Quaternion(getNumber(triple[0]), getNumber(triple[1]), getNumber(triple[2])));
      }

      _parent->SetAnimation(anim);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatePose)
}
