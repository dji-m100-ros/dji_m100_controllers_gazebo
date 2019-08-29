#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <gazebo_msgs/ContactsState.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

#include <cmath>
#include <string>


#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif


namespace gazebo
{
    class DJI_ROS_ControlPlugin : public ModelPlugin{
        public:
            DJI_ROS_ControlPlugin(){}
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
                std::cout<<"\033[1;32m DJI ROS Control Plugin is successfully plugged in to "<<_model->GetName()<<"\033[0m\n";
                this->model = _model;
                this->world = _model->GetWorld();
                this->base_link = model->GetLink();
                std::string ns = _model->GetName();
                this->gimbal_yaw_link = _model->GetLink("gimbal_yaw_link");
                

                std::cout<<"\033[1;32m Gimbal Yaw link exists : "<<this->gimbal_yaw_link->GetName()<<"\033[0m\n";
                
                ros::SubscribeOptions attitude_ops = ros::SubscribeOptions::create<geometry_msgs::QuaternionStamped>(
                    this->model->GetName()+"/dji_sdk/attitude", 1000,
                    boost::bind(&DJI_ROS_ControlPlugin::attitudeCallback, this, _1),
                    ros::VoidPtr(), &this->callback_queue);

                this->attitude_subscriber = nh.subscribe(attitude_ops);

                
                ros::SubscribeOptions gps_position_ops = ros::SubscribeOptions::create<sensor_msgs::NavSatFix>(
                    this->model->GetName()+"/dji_sdk/gps_position",1000,
                    boost::bind(&DJI_ROS_ControlPlugin::gpsCallback,this,_1),
                    ros::VoidPtr(),&this->callback_queue);
                this->gps_position_subscriber = nh.subscribe(gps_position_ops);

                ros::SubscribeOptions gimbal_ops = ros::SubscribeOptions::create<geometry_msgs::Vector3Stamped>(
                    this->model->GetName()+"/dji_sdk/gimbal_angle", 1000,
                    boost::bind(&DJI_ROS_ControlPlugin::gimbalOrientationCallback, this, _1),
                    ros::VoidPtr(), &this->callback_queue);
                this->gimbal_orientation_subscriber = nh.subscribe(gimbal_ops);

                if (_sdf->HasElement("allow_contact_sensing"))
                    this->contact_allowed = _sdf->GetElement("allow_contact_sensing")->Get<bool>();
                if (_sdf->HasElement("initial_height"))
                    this->initial_height = _sdf->GetElement("initial_height")->Get<double>();

                if(this->contact_allowed)
                {
                    ros::SubscribeOptions contact_ops = ros::SubscribeOptions::create<gazebo_msgs::ContactsState>(
                        this->model->GetName()+"/contact",1000,
                        boost::bind(&DJI_ROS_ControlPlugin::contactCallback,this,_1),
                        ros::VoidPtr(),&this->callback_queue);
                    this->contact_subscriber = nh.subscribe(contact_ops);
                }    
                
                this->spherical_coordinates_handle = this->world->SphericalCoords();
                
                
                this->reset();
                this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DJI_ROS_ControlPlugin::OnUpdate,this));
            }
            void attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& quat_msg)
            {
                #if GAZEBO_MAJOR_VERSION >= 8 
                    this->base_orientation.W() = quat_msg->quaternion.w;
                    this->base_orientation.X() = -quat_msg->quaternion.x;
                    this->base_orientation.Y() = -quat_msg->quaternion.y;
                    this->base_orientation.Z() = quat_msg->quaternion.z;
                #else
                    this->base_orientation.w = quat_msg->quaternion.w;
                    this->base_orientation.x = -quat_msg->quaternion.x;
                    this->base_orientation.y = -quat_msg->quaternion.y;
                    this->base_orientation.z = quat_msg->quaternion.z;
                #endif
            }
            void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
            {
                double lat = gps_msg->latitude;
                double lon = gps_msg->longitude;
                double alt = gps_msg->altitude;
                math::Vector3d local_position = this->spherical_coordinates_handle->LocalFromSpherical(math::Vector3d(lat,lon,alt));
                
                #if GAZEBO_MAJOR_VERSION >= 8
                   this->base_position.Set(-local_position.X(),-local_position.Y(),local_position.Z());
                #else
                    this->base_position.Set(-local_position.x,-local_position.y,local_position.z);
                #endif
            }
            void reset()
            {
                this->base_link->SetForce(math::Vector3d(0,0,0));
                this->base_link->SetTorque(math::Vector3d(0,0,0));
                this->base_orientation.Set(1,0,0,0);
                double x = this->base_link->WorldPose().Pos().X();
                double y = this->base_link->WorldPose().Pos().Y();
                this->base_position.Set(x,y,this->initial_height);
                this->gimbal_orientation.Set();
            }
            void gimbalOrientationCallback(const geometry_msgs::Vector3Stamped::ConstPtr& gimbal_orientation_msg)
            {
                #if GAZEBO_MAJOR_VERSION >= 8
                    this->gimbal_orientation.X() = gimbal_orientation_msg->vector.x;
                    this->gimbal_orientation.Y() = gimbal_orientation_msg->vector.y;
                    this->gimbal_orientation.Z() = gimbal_orientation_msg->vector.z;
                #else
                    this->gimbal_orientation.x) = gimbal_orientation_msg->vector.x;
                    this->gimbal_orientation.y = gimbal_orientation_msg->vector.y;
                    this->gimbal_orientation.z = gimbal_orientation_msg->vector.z;
                #endif
            }
            void contactCallback(const gazebo_msgs::ContactsState::ConstPtr& contact_msg)
            {
                int num_of_contacts = contact_msg->states.size();
                if(num_of_contacts)//Pause the simulation
                {
                    
                    this->contact = true;
                    std::cout<<"\033[1;31m Contact information :"<<contact_msg->states[0].info<<"\033[0m\n";
                }
            }
            void OnUpdate(){
                //Early check for contact
                if(this->contact)
                {
                    this->world->SetPaused(true);
                    return;
                }
                this->callback_queue.callAvailable();
                // Get simulator time
                #if GAZEBO_MAJOR_VERSION >= 8
                    common::Time sim_time = this->world->SimTime();
                #else
                    common::Time sim_time = this->world->GetSimTime();
                #endif
                double dt = (sim_time - this->last_time).Double();
                if (dt == 0.0) return;
                
                
                // save last time stamp
                this->last_time = sim_time;  
                this->base_orientation.Normalize();
                math::Pose3d target_pose;
                target_pose.Set(this->base_position,this->base_orientation);
                this->world->SetPaused(true);
                this->model->SetWorldPose(target_pose);
                this->world->SetPaused(false);
                #if GAZEBO_MAJOR_VERSION >= 8
                    math::Pose3d gimbal_pose = this->gimbal_yaw_link->RelativePose();
                    gimbal_pose.Set(gimbal_pose.Pos(),this->gimbal_orientation);
                #else
                    math::Pose3d gimbal_pose = this->gimbal_yaw_link->GetRelativePose();
                    gimbal_pose.Set(gimbal_pose.pos,this->gimbal_orientation);
                #endif
                this->model->SetLinearVel(math::Vector3d(0,0,0));
                this->model->SetAngularVel(math::Vector3d(0,0,0));
                this->gimbal_yaw_link->SetRelativePose(gimbal_pose);
            }
        private:
            ros::NodeHandle nh;
            physics::WorldPtr world;
            physics::ModelPtr model;
            physics::LinkPtr base_link,gimbal_yaw_link;
            common::SphericalCoordinatesPtr spherical_coordinates_handle;
            event::ConnectionPtr update_connection;

            math::Quaterniond base_orientation;
            math::Vector3d base_position;
            math::Vector3d gimbal_orientation;
            
            ros::Subscriber attitude_subscriber;
            ros::Subscriber gps_position_subscriber;
            ros::Subscriber gimbal_orientation_subscriber;
            
            ros::Subscriber contact_subscriber;
            bool contact = false;
            bool contact_allowed = false;
            double initial_height = 0;
            ros::CallbackQueue callback_queue;
            common::Time last_time;

    };
    GZ_REGISTER_MODEL_PLUGIN(DJI_ROS_ControlPlugin)
}