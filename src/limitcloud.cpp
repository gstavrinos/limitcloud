#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::Publisher pub;
float robot_width, tolerance;

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& msg){

    sensor_msgs::PointCloud2 output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = msg->header.frame_id;
    output.height = msg->height;
    output.width = msg->width;
    output.fields = msg->fields;
    output.is_bigendian = msg->is_bigendian;
    output.point_step = msg->point_step;
    output.row_step = output.width * output.point_step;
    output.is_dense = msg->is_dense;
    output.data = msg->data;
    for (sensor_msgs::PointCloud2Iterator<float>
        iter_x(output, "x"), iter_y(output, "y"), iter_z(output, "z");
        iter_x != iter_x.end();
        ++iter_x, ++iter_y, ++iter_z){
        if(*iter_y > robot_width/2.0f+tolerance or *iter_y < -robot_width/2.0f-tolerance){
            *iter_x = NAN;
            *iter_y = NAN;
            *iter_z = NAN;
        }
    }

    pub.publish (output);
}


int main (int argc, char** argv){
    ros::init (argc, argv, "limitcloud");
    ros::NodeHandle nh;

    std::string in_topic;
    nh.param("limitcloud/input_topic", in_topic, std::string("zed/point_cloud/cloud_registered"));
    nh.param("limitcloud/robot_width", robot_width, 0.55f);
    nh.param("limitcloud/tolerance", tolerance, 0.05f);
    ros::Subscriber sub = nh.subscribe (in_topic, 1, cloud_callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> (in_topic + "/limitcloud", 1);

    ros::spin ();
}