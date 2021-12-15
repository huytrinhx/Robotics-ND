#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "add_markers/AddMarkers.h"
#include "geometry_msgs/Pose.h"

class AddMarkers {
    private:
        ros::NodeHandle _n;
        ros::Publisher _pub;
        ros::ServiceServer _serv;
        geometry_msgs::Pose _pickup;
        geometry_msgs::Pose _dropoff;
        geometry_msgs::Pose _current_pose;
        visualization_msgs::Marker _marker;
        int _robot_state;
        int _duration;

    public:
        //Define initialization values
        AddMarkers(geometry_msgs::Pose pickup_pose, geometry_msgs::Pose dropoff_pose):
        _pickup(pickup_pose), _dropoff(dropoff_pose),_robot_state(0) {
            this->_serv = this->_n.advertiseService("add_markers", &AddMarkers::addMarkersCallback, this);
            this->_pub = this->_n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        };
        //Define marker common properties
        //Set color_alpha to 1.0 to make it visible and 0.0 to make it transparent (invisible)
        void setMarkerProperties(float color_alpha) {
             // Set our initial shape type to be a cube
            uint32_t shape = visualization_msgs::Marker::CUBE;
            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            this->_marker.header.frame_id = "map";
            this->_marker.header.stamp = ros::Time::now();
            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            this->_marker.ns = "basic_shapes";
            this->_marker.id = 0;
            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
            this->_marker.type = shape;
            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            this->_marker.action = visualization_msgs::Marker::ADD;
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            this->_marker.scale.x = 0.2;
            this->_marker.scale.y = 0.2;
            this->_marker.scale.z = 0.2;
            // Set the color -- be sure to set alpha to something non-zero!
            this->_marker.color.r = 0.0f;
            this->_marker.color.g = 1.0f;
            this->_marker.color.b = 0.0f;
            this->_marker.color.a = color_alpha;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            ROS_INFO("Current marker pose: X: %.2f, Y: %.2f",this->_current_pose.position.x,this->_current_pose.position.y);
            this->_marker.pose.position.x = this->_current_pose.position.x;
            this->_marker.pose.position.y = this->_current_pose.position.y;
            this->_marker.pose.position.z = this->_current_pose.position.z;
            this->_marker.pose.orientation.x = this->_current_pose.orientation.x;
            this->_marker.pose.orientation.y = this->_current_pose.orientation.y;
            this->_marker.pose.orientation.z = this->_current_pose.orientation.z;
            this->_marker.pose.orientation.w = this->_current_pose.orientation.w;

            this->_marker.lifetime = ros::Duration(_duration);
        }

        //Publish the marker
        void publishMarker() {
            // Publish the marker
            while (this->_pub.getNumSubscribers() < 1)
                {
                if (!ros::ok())
                    {
                        return;
                    }
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1);
                }
            ROS_INFO("Publishing marker...");
            this->_pub.publish(this->_marker);
            return;
        }

        //Define the main logic based on the request from pick_objects
        bool addMarkersCallback(add_markers::AddMarkers::Request &req,
                                add_markers::AddMarkers::Response &res){
                
           

            if(req.str_request.compare("pickup") == 0) {
                //Once robot arrives pickup zone, make the pickup marker invisible
                _current_pose = _pickup;
                _duration = 5;
                this->setMarkerProperties(0.0);
                this->publishMarker();
                ROS_INFO("Hiding the pickup marker...");
                res.str_response = "done";
                return true;

            } else if (req.str_request.compare("dropoff") == 0) {
                //Once robot done picking up, make the dropoff marker visible
                _current_pose = _dropoff;
                _duration = 30;
                this->setMarkerProperties(1.0);
                this->publishMarker();
                ROS_INFO("Showing the dropoff marker...");
                res.str_response = "done";
                return true;
            } else if (req.str_request.compare("default") == 0) {
                //Once robot done picking up, make the dropoff marker visible
                _current_pose = _pickup;
                _duration = 30;
                this->setMarkerProperties(1.0);
                this->publishMarker();
                ROS_INFO("Showing the pickup marker...");
                res.str_response = "done";
                return true;
            } else {
                //Return invalid response
                res.str_response = "invalid request";
                return false;
            }
 
            return true;

        }

        //Test module to test add module sequence
        void testAddMarkers() {
            ROS_INFO("Showing the pickup marker...");
            _current_pose = _pickup;
            _duration = 5;
            this->setMarkerProperties(1.0);
            this->publishMarker();
            
            ROS_INFO("Hiding the pickup marker...");
            _current_pose = _pickup;
            _duration = 5;
            this->setMarkerProperties(0.0);
            this->publishMarker();
            

            ROS_INFO("Showing the dropoff marker...");
            _current_pose = _dropoff;
            _duration = 5;
            this->setMarkerProperties(1.0);
            this->publishMarker();
            return;
        }

};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  geometry_msgs::Pose pickup_pose, dropoff_pose;
  pickup_pose.position.x = -10.0;
  pickup_pose.position.y = 1.0;
  pickup_pose.orientation.w = 1.0;

  dropoff_pose.position.x = -10.0;
  dropoff_pose.position.y = -4.0;
  dropoff_pose.orientation.w = 1.0;

  AddMarkers node(pickup_pose, dropoff_pose);

  //Only enable when running add_markers.sh to test
  //node.testAddMarkers();

  ros::spin();
  return 0;
}