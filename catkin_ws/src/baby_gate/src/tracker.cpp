#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <baby_gate/Object_Params.h>
#include <baby_gate/Filtered_Params.h>
#include <math.h>
#include "SerialComms.h"

#define ROLLING_SAMPLES 8
#define ERROR_TIME 0.5 
#define GATE_LOW_LIMIT 200          // In pixels, x co-ordinate of start of gate
#define GATE_HIGH_LIMIT 500         // in pixels, x co-ordinate of end of gate
#define BABY_HEIGHT_THRESHOLD 5 

#define OPENING 1
#define OPEN 2
#define CLOSING 3
#define CLOSED 4

#define GATE_OPEN_DEPTH 65      // In inches

#define GATE_OPENING_TIME 7     // In seconds
#define GATE_OPEN_TIME 5        // In seconds
#define GATE_CLOSING_TIME 6     // In seconds

class Tracker
{
    public:
    ros::NodeHandle nh;
    ros::Subscriber objParamsSub;
    ros::Publisher filteredParamsPub;
    ros::Time prevTime;

    SerialComms arduino;
    
    float deltaXArr[ROLLING_SAMPLES];
    float deltaYArr[ROLLING_SAMPLES];
    float deltaZArr[ROLLING_SAMPLES];
    float deltaTimeArr[ROLLING_SAMPLES];
    float xArr[ROLLING_SAMPLES];
    float yArr[ROLLING_SAMPLES];
    float zArr[ROLLING_SAMPLES];

    float avgX, avgY, avgZ, avgDeltaX, avgDeltaY, avgDeltaZ, avgTime;        // Filtered readings
    bool objectsFound;
    float x_predicted;
    
    int gateState;

    ros::Time openingStartTime;
    ros::Time openStartTime;
    ros::Time closingStartTime;

public:
    Tracker()
    {
        // Subscribe to raw pose estimates from object detector node
        objParamsSub = nh.subscribe("/baby_gate/obj_pose_raw", 1, &Tracker::poseCb, this);
        
        // Setup publisher to publish pose of objects after filtering
        filteredParamsPub = nh.advertise<baby_gate::Filtered_Params>("/baby_gate/obj_pose_filtered", 1);
        
	    // Make sure Arduino serial port is connected
        if(arduino.openSerialPort() == -1)
	    {
	        ros::shutdown();
        }
        sleep(1);
        
        prevTime = ros::Time::now();
        objectsFound = false;
        avgX = 0, avgY = 0, avgZ = 0, avgDeltaX = 0, avgDeltaY = 0, avgDeltaZ = 0, avgTime = 0;
    
        // Initialize gate state machine
        gateState = OPEN;
        openStartTime = ros::Time::now();

        // Tell Arduino to stop gate
        arduino.sendData("k", 1);
    }

    ~Tracker()
    {
        // Tell Arduino to open the gate when the program terminates
        arduino.sendData("o", 1);
    }
    void poseCb(const baby_gate::Object_Params::ConstPtr &poseMsg)
    {
        // Run a moving average filter on the raw pose readings to remove noise
        filter_data(poseMsg);

        if(objectsFound == true)
        {
            // Calculate velocities
            float velX = avgDeltaX/avgTime;
            float velZ = avgDeltaZ/avgTime;

            // Path prediction stuff

            float xPredicted = 0;
            bool intersectGate = false;

            float predictedTime = abs((avgZ/velZ));
            xPredicted = avgX + (velX * predictedTime);

	    	if(velZ < 0)        // Object is coming towards the gate
            {
                if((xPredicted >= GATE_LOW_LIMIT) && (xPredicted <= GATE_HIGH_LIMIT))
                    intersectGate = true;
            }

            // Human vs Baby Classifier
            float heightRatio = avgY/avgZ;
            char detectedObject = 'B';

            if(heightRatio < BABY_HEIGHT_THRESHOLD)
                detectedObject = 'H';
           
            std::string outputString;

            if(detectedObject == 'H')
                outputString = "HUMAN";
            else
               outputString = "BABY";

            ROS_DEBUG("%d %.1f %s", intersectGate, heightRatio, outputString.c_str());

            // Open baby gate
            if((intersectGate == true) && (detectedObject == 'H') && (avgZ <= GATE_OPEN_DEPTH))
            {
                if((gateState == CLOSING) || (gateState == CLOSED))
                {
                    gateState = OPENING;

                    // Tell Arduino to open gate
                    arduino.sendData("o", 1);

                    openingStartTime = ros::Time::now();
                }
                else if(gateState == OPEN)
                {
                    // Reset open timer;  a new is about to cross the gate
                    openStartTime = ros::Time::now();
                }
                else
                {
                    // Do nothing if gate is already opening
                }

            }

            // Publish filtered pose 
            baby_gate::Filtered_Params filteredPoseMsg;

            filteredPoseMsg.timeStamp = ros::Time::now();
            filteredPoseMsg.xPosition = (int)avgX;
            filteredPoseMsg.yPosition = (int)avgY;
            filteredPoseMsg.zPosition = (int)avgZ;
            filteredPoseMsg.xPredicted = (int)xPredicted;   
            filteredPoseMsg.velocityZ = velZ;        

            filteredParamsPub.publish(filteredPoseMsg);
        }
    }

    void filter_data(const baby_gate::Object_Params::ConstPtr &poseMsg)
    {
        static float prevX = 0;
        static float prevY = 0;
        static float prevZ = 0;

        static int sampleCount = 0;
        static int bufStart = 0;        

        float deltaX = 0;
        float deltaY = 0;
        float deltaZ = 0;  
        float deltaTime = 0;

        // Get pose and timestamp from message
        float xPosition = poseMsg->xPosition;
        float yPosition = poseMsg->yPosition;
        float zPosition = poseMsg->zPosition;
        
        ros::Time currentTime = poseMsg->timeStamp;

        // Calculate change in readings
        deltaTime = (currentTime - prevTime).toSec();
        deltaX = xPosition - prevX;
        deltaY = yPosition - prevY;
        deltaZ = zPosition - prevZ;

        // If a reading comes after a long interval, it's probably a new object entering the zone
        if(deltaTime >= ERROR_TIME)
            sampleCount = 0;

        if(sampleCount < ROLLING_SAMPLES)
        {
            objectsFound = false;

            // Fill up averaging window with the first few samples
            if(deltaTime < ERROR_TIME)
            {
                xArr[sampleCount] = xPosition;
                yArr[sampleCount] = yPosition;
                zArr[sampleCount] = zPosition;

                deltaXArr[sampleCount] = deltaX;
                deltaYArr[sampleCount] = deltaY;
                deltaZArr[sampleCount] = deltaZ;
                
                deltaTimeArr[sampleCount] = deltaTime;

                sampleCount++;
            }
        }
        else
        {
            objectsFound = true;

            // Compute rolling average

            float sumX = 0;
            float sumY = 0;
            float sumZ = 0;
            float sumDeltaX = 0;
            float sumDeltaY = 0;
            float sumDeltaZ = 0;
            float sumTime = 0;

            for(int i = 0; i < ROLLING_SAMPLES; i++)
            {
                sumX = sumX + xArr[i];
                sumY = sumY + yArr[i];
                sumZ = sumZ + zArr[i];

                sumDeltaX = sumDeltaX + deltaXArr[i];
                sumDeltaY = sumDeltaY + deltaYArr[i];
                sumDeltaZ = sumDeltaZ + deltaZArr[i];
           
                sumTime = sumTime + deltaTimeArr[i];
            }

            // Update sample buffers
            xArr[bufStart] = xPosition;
            yArr[bufStart] = yPosition;
            zArr[bufStart] = zPosition;

            deltaXArr[bufStart] = deltaX;
            deltaYArr[bufStart] = deltaY;
            deltaZArr[bufStart] = deltaZ;
          
            deltaTimeArr[bufStart] = deltaTime;

            // Update buffer start pointer
            bufStart++;

            if(bufStart >= ROLLING_SAMPLES)
                bufStart = 0;

            // Calculate the moving average
            avgX = sumX/ROLLING_SAMPLES;
            avgY = sumY/ROLLING_SAMPLES;
            avgZ = sumZ/ROLLING_SAMPLES;

            avgDeltaX = sumDeltaX/ROLLING_SAMPLES;
            avgDeltaY = sumDeltaY/ROLLING_SAMPLES;
            avgDeltaZ = sumDeltaZ/ROLLING_SAMPLES; 

            avgTime = sumTime/ROLLING_SAMPLES;
        }

        // Store current values for next filtering round
        prevX = xPosition;
        prevY = yPosition;
        prevZ = zPosition;
        prevTime = currentTime;
     
    }

    void runGateStateMachine()
    {
        ros::Time currentTime = ros::Time::now();
        
        switch(gateState)
        {
            case OPENING:

            // Gate takes some time to open
            if((currentTime - openingStartTime).toSec() >= GATE_OPENING_TIME)
            {
                gateState = OPEN;
                openStartTime = ros::Time::now();

               // Tell Arduino to stop gate
                arduino.sendData("k", 1);
            }
            break;

            case OPEN:

            // Gate stays open for some time, then starts to close
            if((currentTime - openStartTime).toSec() >= GATE_OPEN_TIME)
            {
                gateState = CLOSING;
                closingStartTime = ros::Time::now();

                // Tell Arduino to close gate
                arduino.sendData("c", 1);
            }
            break;

            case CLOSING:
            
            // Gate takes some time to close
            if((currentTime - closingStartTime).toSec() >= GATE_CLOSING_TIME)
            {
                gateState = CLOSED;

                // Tell Arduino to stop gate
                arduino.sendData("k", 1);
            }

            break;

            case CLOSED:
            // Do nothing
            break;
        }
    }
};

int main(int argc, char **argv)
{
    //Do this before doing any further ROS stuff
    ros::init(argc, argv, "tracker");

    Tracker tr;

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::Rate loop_rate(100);       // 100 Hz

    while(ros::ok())
    {
        tr.runGateStateMachine();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
