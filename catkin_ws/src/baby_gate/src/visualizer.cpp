#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <baby_gate/Filtered_Params.h>
#include <math.h>


class Visualizer 
{
    ros::NodeHandle nh;
    ros::Subscriber objParamsSub;
    ros::Time prevTime;
    std::vector<cv::Point2i> poseList;           // Stores list of most recent poses
    int zoneStartY;
    int zoneEndY;
    int zoneStartX;
    int zoneEndX;
    int zoneStartDepth;
    int zoneEndDepth;
 
public:
    Visualizer()
    {
        // Subscribe to input video feed and publish output video feed
        objParamsSub = nh.subscribe("/baby_gate/obj_pose_filtered", 1, &Visualizer::poseCb, this);
        prevTime = ros::Time::now();

        cv::namedWindow("Visualizer", CV_WINDOW_AUTOSIZE);
    
        zoneStartY = 200;
        zoneEndY = 400;
        zoneStartX = 100;
        zoneEndX = 540;
        zoneStartDepth = 35;
        zoneEndDepth = 130;
    }

    ~Visualizer()
    {
        cv::destroyWindow("Visualizer");
    }

    void poseCb(const baby_gate::Filtered_Params::ConstPtr &poseMsg)
    {
        // Start timer to measure computation time
        double startTick = (double)cv::getTickCount();

        // Get pose and timestamp from message
        int xPosition = poseMsg->xPosition;
        int yPosition = poseMsg->yPosition;
        int zPosition = poseMsg->zPosition;
        int xPredicted = poseMsg->xPredicted;
        float velocityZ = poseMsg->velocityZ;

        ros::Time currentTime = poseMsg->timeStamp;
        float deltaTime = (currentTime - prevTime).toSec();

        // Convert depth pose to pixel co-ordinates
        int zMapped = 0;
        int pixelToDepthRatio = (zoneEndY - zoneStartY)/(zoneEndDepth - zoneStartDepth);
        
        if((zPosition >= zoneStartDepth) && (zPosition <= zoneEndDepth))
            zMapped = zoneStartY + (pixelToDepthRatio * (zPosition - zoneStartDepth)) ;
        else
            ROS_DEBUG("Bad z position data!");

        ROS_DEBUG("%d %d", zMapped, zPosition);
        
        prevTime = currentTime;

        // Drawing image used to visualize motion
        int rows = 480, cols = 640;
        cv::Mat drawImage = cv::Mat::zeros(rows, cols, CV_8UC3);

        // Create baby gate rectangle

        int gateStartX = 200;
        int gateEndX = 500;
        int gateStartY = 20;
        int gateEndY = 50;

        cv::rectangle(drawImage, cv::Point(gateStartX, gateStartY), cv::Point(gateEndX, gateEndY), cv::Scalar(0, 255, 255), 2, 8);
        cv::line(drawImage, cv::Point(0, 50), cv::Point(cols, 50), cv::Scalar(0, 255, 0), 2);

        // Color blue zone in which objects are being detected        
        color_blue_zone(drawImage);    

        // Clear list of poses if no activity for a while (e.g, when a new object enters the zone)
        if(deltaTime > 1)
        {
            poseList.clear();
        }

        // Update list of most recent poses, insert newest Pose at the end of the list
        int maxPoses = 10;
        poseList.push_back(cv::Point(xPosition, zMapped));

        if(poseList.size() > maxPoses)
            poseList.erase(poseList.begin());

        // Draw most recent poses
        for(int i = 0; i < poseList.size(); i++)
        {
            cv::putText(drawImage, "o", poseList[i], CV_FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 255));    
        }

        // Draw predicted pat if object is walking towards the gate
        
        if(velocityZ < 0)
        {
            int numPoses = poseList.size();

            cv::Point pathStart = poseList[numPoses - 1];               // Current pose
            cv::Point pathEnd;

            pathEnd.x = xPredicted;
            pathEnd.y = gateEndY;
            
            cv::line(drawImage, pathStart, pathEnd, cv::Scalar(255, 255, 0), 2);
        }

        // Calculate computation time
        double endTick = (double)cv::getTickCount();
        double procTime = (endTick - startTick)/cv::getTickFrequency();

        //ROS_DEBUG("Processing time is %f", procTime);
    
        // Display images
        cv::imshow("Visualizer", drawImage);
        cv::waitKey(1); 
 
    }

    void color_blue_zone(cv::Mat &thresholdImage)
    {
        CV_Assert(thresholdImage.depth() == CV_8U);

        if(thresholdImage.channels() == 3)
        {
            int i, j;
            unsigned char *pixel;

            int channels = thresholdImage.channels();
            int nRows = thresholdImage.rows;
            int nCols = thresholdImage.cols * channels;

           
            // Draw position indicators of the zonei
            std::stringstream textOutput;
            
            textOutput << "Depth: " << zoneStartDepth << "\"";
            putText(thresholdImage, textOutput.str(), cv::Point(zoneStartX, zoneStartY - 10), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255));    

            textOutput.str("");
            textOutput << "Depth: " << zoneEndDepth << "\"";
            putText(thresholdImage, textOutput.str(), cv::Point(zoneStartX, zoneEndY + 17), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255));    


            for(i = 0; i < nRows; i++)
            {
                pixel = thresholdImage.ptr<uchar>(i); 

                for(j = 0; j < nCols; j = j + channels)
                {
                    if((i >= zoneStartY) && (i <= zoneEndY) && (j/3 >= zoneStartX) && (j/3 <= zoneEndX))           // Blue channel is pixel[j], green is pixel[j+1], red is pixel[j+2]
                    {
                        pixel[j] = 255;
                        pixel[j + 1] = 0;
                        pixel[j + 2] = 0;    
                    }
                }
            }
        }
    }   

};

int main(int argc, char **argv)
{
    //Do this before doing any further ROS stuff
    ros::init(argc, argv, "visualizer");

    Visualizer tr;
    
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::spin();

    return 0;
}

