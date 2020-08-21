#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>


using namespace std;
using namespace cv;


ros::Publisher pub;
bool showPreview;
std::string camera_topic_name;
std::string default_camera_topic = "/camera_topic_2";


    pub.publish(faces_msg);
    if (showPreview) {
        imshow("Live Feed_2", cvPtr->image);
        waitKey(3);
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "id_no_send");
    ros::NodeHandle nh;
    nh.param("/id_no_send/show_preview", showPreview, false);
    nh.param("/id_no_send/camera_topic_2", camera_topic_name, default_camera_topic);

   
    ros::spin();
}

