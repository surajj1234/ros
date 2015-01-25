#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image Window";

class ImageConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub;
    image_transport::Publisher imagePub;
    cv::Mat lastImage;
    bool display;

public:
    ImageConverter() : it(nh)
    {
        // Enable or disable image display
        display = false;

        // Subscribe to input video feed and publish output video feed
        imageSub = it.subscribe("/camera/depth/image", 1, &ImageConverter::imageCb, this);
        imagePub = it.advertise("/baby_gate/kinect_depth_image", 1);

        if(display == true)
            cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        if(display == true)
            cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        // Scale floating point images so that they display nicely
        if(msg->encoding.find("F") != std::string::npos)
        {
            cv::Mat floatImageBridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
            cv::Mat_<float> floatImage = floatImageBridge;
            double maxVal;
            cv::minMaxIdx(floatImage, 0, &maxVal);

            if(maxVal > 0)
            {
                floatImage /= maxVal;
            }
            lastImage = floatImage;
        }

        // Update GUI window
        if(display == true)
        {
            cv::imshow(OPENCV_WINDOW, lastImage);
            cv::waitKey(3);
        }

        // Create ROS image
        sensor_msgs::Image rosImage;
        
        rosImage.header = msg->header;
        rosImage.encoding = msg->encoding;
        
        rosImage.height = lastImage.rows;
        rosImage.width = lastImage.cols;
          
        rosImage.is_bigendian = false;
        rosImage.step = lastImage.step;
        size_t size = lastImage.step * lastImage.rows;
        rosImage.data.resize(size);
        memcpy((char*)(&rosImage.data[0]), lastImage.data, size);
        

        // Output modified video stream
        imagePub.publish(rosImage);
    }

};

int main(int argc, char **argv)
{
    //Do this before doing any further ROS stuff
    ros::init(argc, argv, "kinect_imaq_depth");

    ImageConverter ic;
    ros::spin();

    return 0;
}
