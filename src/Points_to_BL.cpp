#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
"others include..."

tf::TransformListener listener;



class MarkerArray_Transform {
private:

  ros::NodeHandle nh_;
  ros::Subscriber MarkerArray_Camera;
  ros::Publisher MarkerArray_Base;
  std::string MarkerArray_topic;
  std::string out_MarkerArray_topic;
  tf::TransformListener listener;
  visualization_msgs::MarkerArray marker_array;

  
  void init_params(){    
    nh_.getParam("MarkerArray_topic", MarkerArray_topic);
    nh_.getParam("out_MarkerArray_topic", out_MarkerArray_topic);
  }  

  void init_subs(){
    MarkerArray_Camera = nh_.subscribe(MarkerArray_topic, 1, &MarkerArray_Transform::Marker_Transform, this); 
  }

  void init_pubs(){
    MarkerArray_Base = nh_.advertise<visualization_msgs::MarkerArray> (out_object_markers_topic, 1);
  }

  void Marker_Transform (const visualization_msgs::MarkerArrayConstPtr&  marker_array){
    visualization_msgs::MarkerArray  marker_array_in = *marker_array; 
    visualization_msgs::MarkerArray  marker_array_transformed = marker_array_in; 
    

   listener.transformPoint(

    
  }
  
public:

  MarkerArray_Transform(ros::NodeHandle* nodehandle):nh_(*nodehandle) {
    init_params();
    init_subs();
    init_pubs();
  }
  
};


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "segment_tabletop_node");
    ros::NodeHandle node_handle("~");
    MarkerArray_Transform MA_TF(&node_handle);
    ros::spin();
    return 0;
} 

void findCoords(const sensor_msgs::LaserScanPtr dataFromLaser){
   //in this function I have to transform,so i do the stuff and then..
   listener.transformPoint("base_link",point,newPoint);
   //where point,newPoint are StampedPoint
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& fromLaser){
   //do stuff...
   findCoords(data);
}



int main(int argc,char** argv){
   ros::init(argc,argv,"name");
   ros::NodeHandle nh;
   ros::Subscriber laser=nh.subscribe("base_scan",10,laserCallback);
   pubLaser=nh.advertise<sensor_msgs::LaserScan>("from_Laser",100);
   ros::spin();
   return 0;
}