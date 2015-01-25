#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <baby_gate/Object_Params.h>
#include "colormap.h"

#define RAW_DISPLAY 0 
//#define RAW_DISPLAY 1

static const std::string OPENCV_WINDOW_INPUT = "Input";
static const std::string OPENCV_WINDOW_OUTPUT = "Output";

static const std::string OPENCV_WINDOW_MORPHED = "Morphed";
static const std::string OPENCV_WINDOW_THRESHOLD = "Threshold";

class ImageConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub;
    ros::Publisher objParamsPub;
    
    int segmentDepthMin, segmentDepthMax, detectDepthMin, detectDepthMax;

public:
    ImageConverter() : it(nh)
    {
        // Subscribe to input video feed and publish output video feed
        imageSub = it.subscribe("/baby_gate/kinect_disparity_image", 1, &ImageConverter::imageCb, this);

        // Setup publisher to publish pose of objects detected
        objParamsPub = nh.advertise<baby_gate::Object_Params>("/baby_gate/obj_pose_raw", 1);

        // Pixels outside this range are segmented out
        segmentDepthMin = 25;
        segmentDepthMax = 140;

        // Only objects within this range are accepted as valid objects
        detectDepthMin = 35;
        detectDepthMax = 130;

        if(RAW_DISPLAY)
        {
            cv::namedWindow(OPENCV_WINDOW_MORPHED);
            cv::namedWindow(OPENCV_WINDOW_THRESHOLD);
        }

        cv::namedWindow(OPENCV_WINDOW_INPUT);
        cv::namedWindow(OPENCV_WINDOW_OUTPUT);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW_INPUT);
        cv::destroyWindow(OPENCV_WINDOW_OUTPUT);
    
        if(RAW_DISPLAY)
        {
            cv::destroyWindow(OPENCV_WINDOW_MORPHED);
            cv::destroyWindow(OPENCV_WINDOW_THRESHOLD);
        }
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        // Start timer to measure computation time
        double startTick = (double)cv::getTickCount();

        // Get disparity image from publisher node
        cv::Mat inputImage = cv_bridge::toCvCopy(msg, msg->encoding)->image;
      
        // Extract objects in desired range from disparity image
        cv::Mat thresholdImage = cv::Mat::zeros(inputImage.size(), CV_8UC1);
        segment_zone(inputImage, thresholdImage);

        // Morphological operations
        cv::Mat morphedImage;
        morph_out_noise(thresholdImage, morphedImage);    

        // Find connected blobs
        cv::Mat binaryImage;
        std::vector <std::vector<cv::Point2i> > blobs;

        cv::threshold(morphedImage, binaryImage, 0.0, 1.0, CV_THRESH_BINARY);        
        find_blobs(binaryImage, blobs);
        
        // Color the blobs and display a rectangle around them
        cv::Mat blobImage = cv::Mat::zeros(morphedImage.size(), CV_8UC3);
        
        for(int i = 0; i < blobs.size(); i++)
        {
            if(blobs[i].size() > 10000)
            {
                int sumX = 0;
                int sumY = 0;

                for(int j = 0; j < blobs[i].size(); j++)
                {
                    int x = blobs[i][j].x;
                    int y = blobs[i][j].y;

                    // Color the blob
                    blobImage.at<cv::Vec3b>(y, x)[0] = 255;
                    blobImage.at<cv::Vec3b>(y, x)[1] = 0;
                    blobImage.at<cv::Vec3b>(y, x)[2] = 0;
                    
                    // Centroid calculations
                    sumX += x;
                    sumY += y;
                }
               
                // Display bounding box around object 
                cv::Rect blobRect = cv::boundingRect(blobs[i]);
                cv::rectangle(blobImage, blobRect, cv::Scalar(0, 255, 0), 2, 8, 0);

                // Calculate centroid of blob
                int centroidX = sumX/(blobs[i].size());
                int centroidY = sumY/(blobs[i].size());

                // Calculate distance of centroid in real world from kinect
                float centroidZ = calculate_depth(inputImage, centroidX, centroidY); 

                // Further reduce detection zone to improve object detections
                if((centroidZ >= detectDepthMin) && (centroidZ <= detectDepthMax))
                {
                    // Display centroid of object
                    std::stringstream textOutput;
                    textOutput << "x: " << centroidX << " y: " << centroidY << " z: " << centroidZ;

                    cv::putText(blobImage, textOutput.str(), cv::Point(centroidX, centroidY), CV_FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255));

                    // Publish pose 
                    baby_gate::Object_Params poseMsg;

                    poseMsg.timeStamp = ros::Time::now();
                    poseMsg.xPosition = centroidX;
                    poseMsg.yPosition = centroidY;
                    poseMsg.zPosition = centroidZ;
                
                    objParamsPub.publish(poseMsg);
                }
            }
        }

        // Calculate computation time
        double endTick = (double)cv::getTickCount();
        double procTime = (endTick - startTick)/cv::getTickFrequency();

        //ROS_DEBUG("Processing time is %f", procTime);
 
        // Update GUI windows
        

        if(RAW_DISPLAY)
        {
            cv::imshow(OPENCV_WINDOW_MORPHED, morphedImage);
            cv::imshow(OPENCV_WINDOW_THRESHOLD, thresholdImage);
        }
        
        cv::imshow(OPENCV_WINDOW_INPUT, inputImage);
        cv::imshow(OPENCV_WINDOW_OUTPUT, blobImage);
 
        cv::waitKey(3); 
    }

    void segment_zone(const cv::Mat &inputImage, cv::Mat &thresholdImage)
    {
        CV_Assert(thresholdImage.depth() == CV_8U);

        if(thresholdImage.channels() == 1)
        {
            int i, j;
            unsigned char *pixel;

            int channels = thresholdImage.channels();
            int nRows = thresholdImage.rows;
            int nCols = thresholdImage.cols * channels;

            for(i = 0; i < nRows; i++)
            {
                pixel = thresholdImage.ptr<uchar>(i); 

                for(j = 0; j < nCols; j = j + channels)
                {
                    // Get depth at each pixel location
                    float depth = calculate_depth(inputImage, j, i);

                    if((depth >= segmentDepthMin) && (depth <= segmentDepthMax))   
                    {
                        // Color the pixels white (Blue channel is pixel[j], green is pixel[j+1], red is pixel[j+2])
                        pixel[j] = 255;
                    }
                    else
                    {
                        // Color the pixels black
                        pixel[j] = 0;
                    }
                }
            }
        }
    }

    void morph_out_noise(const cv::Mat &thresholdImage, cv::Mat &morphedImage)
    {
        int morphOperation = 3;         // 2:Opening, 3:Closing, 4:Gradient, 5:Top Hat, 6:Black Hat 
        int morphElement = 0;           // 0:Rect, 1:Cross, 2:Ellipse
        int morphSize = 9;              // Kernel size = 2 * morphsize + 1

        cv::Mat morphKernel = cv::getStructuringElement(morphElement, cv::Size((2 * morphSize) + 1, (2 * morphSize) + 1), cv::Point(morphSize, morphSize));

        morphologyEx(thresholdImage, morphedImage, morphOperation, morphKernel);
    }

    void find_blobs(const cv::Mat &binaryImage, std::vector <std::vector<cv::Point2i> > &blobs)
    {
        blobs.clear();

        // Fill the label image with blobs
        // 0 - background
        // 1 - unlabeled background
        // 2+ - labelled background
        
        cv::Mat labelImage;
        binaryImage.convertTo(labelImage, CV_32SC1);

        int labelCount = 2;         // Starts at 2 since 0,1 are in use already
        for(int i = 0; i < labelImage.rows; i++)
        {
            int *row = (int*)labelImage.ptr(i);

            for(int j = 0; j < labelImage.cols; j++)
            {
                if(row[j] != 1)
                {
                    continue;       // For an black pixel, don't do anything     
                }

                cv::Rect rect;
                cv::floodFill(labelImage, cv::Point(j, i), labelCount, &rect, 0, 0, 4);     // Label all neighboring pixels which have a value 1 with the current label no.
                
                std::vector <cv::Point2i> blob;

                for(int k = rect.y; k < (rect.y + rect.height); k++)
                {
                    int *row2 = (int*)labelImage.ptr(k);
                    
                    for(int l = rect.x; l < (rect.x + rect.width); l++)
                    {
                        if(row2[l] != labelCount)
                        {
                            continue;       // Discard pixels which aren't part of the flood fill
                        }
                        blob.push_back(cv::Point2i(l, k));
                    }
                }

                blobs.push_back(blob);
                labelCount++;
            } 
        }
    }

    float calculate_depth(const cv::Mat &inputImage, int x, int y)
    {
        float depth = 0;

        float maxDepth = 13.0;      // In feet, max range of kinect
        float minDepth = 1.5;       // In feet, min range of kinect

        CV_Assert(inputImage.depth() == CV_8U);

        cv::Vec3b pixelColors = inputImage.at<cv::Vec3b>(y, x);
        
        // Find location in colormap which has same color value as the current pixel
        for(int i = 0; i < COLORMAP_SIZE * 3; i = i + 3)
        {
            // Colormap is stored in RGB, whereas current pixel is in BGR
            if((colormap[i] == pixelColors[2]) && (colormap[i + 1] == pixelColors[1]) && (colormap[i + 2] == pixelColors[0]))
            {
                int index = i/3;

                if(index == 0)
                    depth = 0;                          // Depth is either too small or too large
                else
                    depth = 6056/(index + 38);          // Based on calibration and subsequent curve fitting analysis
            
                break;
            }   
        }    
        
        return depth;
    }
};

int main(int argc, char **argv)
{
    //Do this before doing any further ROS stuff
    ros::init(argc, argv, "object_detector");

    ImageConverter ic;

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::spin();

    return 0;
}

        // Get depth image from publisher node
        // cv::Mat  inputImage = cv_bridge::toCvCopy(msg, msg->encoding)->image;
        
        // Convert to 8-bit char image from 32-bit float image
        //cv::Mat inputImage;
        //double minVal, maxVal;
        //minMaxLoc(depthImage, &minVal, &maxVal);
        //Image.convertTo(inputImage, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));         
 
        // Filter image 
        //cv::Mat filteredImage;
        //int kernelLength = 3;
        //cv::blur(thresholdImage, filteredImage, cv::Size(kernelLength, kernelLength));
 
        /*
        // Canny Edge Detection
        cv::Mat cannyImage;
        int cannyLowThreshold = 30;
        int cannyRatio = 3;
        int cannyKernelSize = 3;
        cv::Canny(preCannyImage, cannyImage, cannyLowThreshold, cannyLowThreshold * cannyRatio, cannyKernelSize);

        // Find contours
        cv::Mat contourImage = cannyImage.clone();
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        findContours(contourImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));       
        ROS_DEBUG("%d", contours.size()); 

        // Draw contours
        cv::Mat drawImage = cv::Mat::zeros(contourImage.size(), CV_8UC3);
        cv::Rect boundRect;
        double maxArea = 0;
        double maxIndex = 0;

        for(int i = 0; i < contours.size(); i++)
        {
             // Calculate contour area
            double area = contourArea(contours[i], false);

            if(area > maxArea)
            {
                maxArea = area;
                maxIndex = i;
                boundRect = cv::boundingRect(contours[i]);
            }
 
            //cv::Scalar color = cv::Scalar(255, 255, 255);
            //drawContours(drawImage, contours, i, color, CV_FILLED, 8, hierarchy, 0, cv::Point());
            //cv::rectangle(drawImage, boundRect, color, 2, 8, 0);
        
        }

        // Draw only for the largest contour
        if(maxArea > 0)
        {
            drawContours(drawImage, contours, maxIndex, cv::Scalar(0, 255, 0), 2, 8, hierarchy, 0, cv::Point());
            cv::rectangle(drawImage, boundRect, cv::Scalar(255, 0, 0), 2, 8, 0);
        } 
        */    
  
