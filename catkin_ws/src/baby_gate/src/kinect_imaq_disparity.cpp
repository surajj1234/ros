#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <baby_gate/Object_Params.h>

#include "colormap.h"

static const std::string OPENCV_WINDOW = "Kinect Disparity Image";

class ImageConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    ros::Subscriber imageSub;
    image_transport::Publisher imagePub;
    cv::Mat_<cv::Vec3b> disparityImage;
    bool display;

public:
    ImageConverter() : it(nh)
    {
        // Enable or disable image display
        display = false;

        // Subscribe to input video feed and publish output video feed
        imageSub = nh.subscribe<stereo_msgs::DisparityImage>("/camera/depth/disparity", 1, &ImageConverter::imageCb, this);
        imagePub = it.advertise("/baby_gate/kinect_disparity_image", 1);

    }

    ~ImageConverter()
    {
        if(display == true)
            cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const stereo_msgs::DisparityImageConstPtr& msg)
    {
        // Check for errors in input
        if(msg->min_disparity == 0.0 && msg->max_disparity == 0.0)
        {
            ROS_DEBUG("Disparity image fields min_disparity and max_disparity are not set");
            return; 
        }
        if(msg->image.encoding != sensor_msgs::image_encodings::TYPE_32FC1)
        {
            ROS_DEBUG("Disparity image must be 32-bit floating point (encoding '32FC1'), but has encoding %s", msg->image.encoding.c_str());
            return;
        }

        // Colormap and display the disparity image
        float minDisparity = msg->min_disparity;
        float maxDisparity = msg->max_disparity;
        float multiplier = 255.0f / (maxDisparity - minDisparity);

        const cv::Mat_<float> dmat(msg->image.height, msg->image.width, (float*)&msg->image.data[0], msg->image.step);
        disparityImage.create(msg->image.height, msg->image.width);

        for(int row = 0; row < disparityImage.rows; row++)
        {
            const float* disparity = dmat[row];
            cv::Vec3b *disparityColor = disparityImage[row];
            cv::Vec3b *disparityColorEnd = disparityColor + disparityImage.cols;

            for(; disparityColor < disparityColorEnd; disparityColor++, disparity++)
            {
                // Convert disparity(a function of depth) to a value in the range 0 - 255
                int index = (*disparity - minDisparity) * multiplier + 0.5;
                index = std::min(255, std::max(0, index));

                // A high disparity implies that index will be close to 255; since objects far away have a high disparity.
                (*disparityColor)[2] = colormap[3*index + 0];
                (*disparityColor)[1] = colormap[3*index + 1];
                (*disparityColor)[0] = colormap[3*index + 2];
            }
        }

        

        // Update GUI window
        if(display == true)
        {
            cv::imshow(OPENCV_WINDOW, disparityImage);
            cv::waitKey(1);
        }

        // Create ROS image
        sensor_msgs::Image rosImage;
        
        rosImage.header = msg->header;
        rosImage.encoding = sensor_msgs::image_encodings::BGR8;
        
        rosImage.height = disparityImage.rows;
        rosImage.width = disparityImage.cols;
          
        rosImage.is_bigendian = false;
        rosImage.step = 1 * rosImage.width * 3;             // step = pixel-depth(1 for a char) * columns(or width) * no. of channels(3 for BGR)
        size_t size = rosImage.step * rosImage.height;
        rosImage.data.resize(size);
        memcpy((char*)(&rosImage.data[0]), disparityImage.data, size);
        
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
