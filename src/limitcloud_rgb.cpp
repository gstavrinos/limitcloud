#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::Publisher pub;
int red, green, blue, r_tolerance, g_tolerance, b_tolerance;

bool approx(int x1, int x2, int tolerance){
    if (x1 >= x2 - tolerance and x1 <= x2 + tolerance){
        return true;
    }
    return false;
}

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
    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x"), iter_y(output, "y"), iter_z(output, "z");
    for (sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(output, "rgb");
        iter_rgb != iter_rgb.end(); ++iter_rgb, ++iter_x, ++iter_y, ++iter_z){
        //ROS_WARN("%f %f %f, [%d %d %d]", *iter_x, *iter_y, *iter_z, iter_rgb[0], iter_rgb[1], iter_rgb[2]);
        if(!(approx(iter_rgb[2], red, r_tolerance) and approx(iter_rgb[1], green, g_tolerance) and approx(iter_rgb[0], blue, b_tolerance))){
            *iter_x = NAN;
            *iter_y = NAN;
            *iter_z = NAN;
            // iter_rgb[0] = 0;
            // iter_rgb[1] = 150;
            // iter_rgb[2] = 0;
        }
    }

    pub.publish (output);
}


int main (int argc, char** argv){
    ros::init (argc, argv, "limitcloud_rgb");
    ros::NodeHandle nh;

    std::string in_topic;
    nh.param("limitcloud_rgb/input_topic", in_topic, std::string("zed/point_cloud/cloud_registered"));
    nh.param("limitcloud_rgb/r_tolerance", r_tolerance, 5);
    nh.param("limitcloud_rgb/g_tolerance", g_tolerance, 5);
    nh.param("limitcloud_rgb/b_tolerance", b_tolerance, 5);

    nh.param("limitcloud_rgb/r", red, 253);
    nh.param("limitcloud_rgb/g", green, 67);
    nh.param("limitcloud_rgb/b", blue, 11);
    ros::Subscriber sub = nh.subscribe (in_topic, 1, cloud_callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> (in_topic + "/limitcloud", 1);

    ros::spin ();
}