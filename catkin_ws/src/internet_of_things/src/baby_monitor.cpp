#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


static const std::string OPENCV_WINDOW = "Baby Monitor Live Feed";


class Baby_Monitor
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher imagePub;
    cv::Mat outputImage;
    cv::Mat inputImage;
    bool display;
    cv::VideoCapture cap;

    public:
    Baby_Monitor() : it(nh)
    {
        // Enable or disable image display
        display = false;

        // Subscribe to input video feed and publish output video feed
        imagePub = it.advertise("baby_monitor/baby_monitor_image", 1);

        // Open webcam for image acquisition
        cap = cv::VideoCapture(CV_CAP_ANY);
        //cap = cv::VideoCapture(1);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

        if(display == true)
            cv::namedWindow(OPENCV_WINDOW);
    }

    ~Baby_Monitor()
    {
        if(display == true)
            cv::destroyWindow(OPENCV_WINDOW);
    }

    void get_new_frame(void)
    {
        // Grab image from camera
        cap >> inputImage;

        if(inputImage.data)
        {

            publish_image(inputImage);
            
            // Update GUI window
            if(display == true)
            {
                cv::imshow(OPENCV_WINDOW, inputImage);
                cv::waitKey(1);
            }
        }
    }

    void publish_image(const cv::Mat &sendImage)
    {
        // Create ROS image
        sensor_msgs::Image rosImage;
        
        rosImage.header = std_msgs::Header();
        rosImage.encoding = "bgr8";
        
        rosImage.height = sendImage.rows;
        rosImage.width = sendImage.cols;
          
        rosImage.is_bigendian = false;
        rosImage.step = sendImage.step;
        size_t size = sendImage.step * sendImage.rows;
        rosImage.data.resize(size);
        memcpy((char*)(&rosImage.data[0]), sendImage.data, size);
 
        // Output modified video stream
        imagePub.publish(rosImage);
    }
};


int main(int argc, char **argv)
{
    //Do this before doing any further ROS stuff
    ros::init(argc, argv, "baby_monitor");

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    Baby_Monitor bm;
    
    ros::Rate loop_rate(30);       // 30 Hz

    while(ros::ok())
    {
        bm.get_new_frame();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
