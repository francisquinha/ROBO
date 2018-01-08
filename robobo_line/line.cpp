// This program gets camera rgb images and makes turtlebot follow a black line on white background
#include <ros/ros.h>
#include <com_mytechia_robobo_ros_msgs/Command.h>
#include <com_mytechia_robobo_ros_msgs/KeyValue.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <float.h>
#include <vector>
#include <sstream>

#define SSTR(x) static_cast< std::ostringstream & > ((std::ostringstream() << std::dec << x )).str()
//#define DEBUG 0

using namespace std;

class LineFollower {

public:
    LineFollower(int color, int kp, int ki, int kd, int speed, float distance) {

        switch(color){
            case 1:
                this->color = red;
                ROS_INFO_STREAM("Color: red");
                break;
            case 2:
                this->color = green;
                ROS_INFO_STREAM("Color: green");
                break;
            case 3:
                this->color = blue;
                ROS_INFO_STREAM("Color: blue");
                break;
            default:
                this->color = black;
                ROS_INFO_STREAM("Color: black");
                break;            
        }

        this->state = random;

        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        ROS_INFO_STREAM("PID Constants: " << this->kp << ", " << this->ki << ", " << this->kd);

        this->speed = speed;
        ROS_INFO_STREAM("Speed: " << this->speed);

        this->distance = distance;
        ROS_INFO_STREAM("ROI distance: " << this->distance);

        this->error = 0;
        this->previous_error = 0;
        this->integral = 0;

        // Initiate the service.
        this->client = nh.serviceClient<com_mytechia_robobo_ros_msgs::Command>("command");
        this->command_id = 1;

        setBackCamera();
        //moveTilt(90);
        //movePan(30);

        // Initiate the subscriber.
        // We subscribe to the robots camera to detect the line to follow
        image_transport::ImageTransport it(nh);
        ros::Duration(1).sleep();
        this->sub = it.subscribe("camera/image", 1, &LineFollower::imageHandler, this);

    }

private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    int command_id;
    image_transport::Subscriber sub;
    enum Color {black, red, green, blue};
    Color color;
    enum State {random, seeLine, noSeeLine};
    State state;
    int kp;
    int ki;
    int kd;
    int speed;
    float error;
    float previous_error;
    float integral;
    float distance;

    float getError(cv::Mat image, float distance) {
        #ifdef DEBUG
            cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);        
            cv::imshow("Image", image);
        #endif
        cv::Mat roi, hsv, mask, mono, blur, thresh;
        roi = image(cv::Rect(0, distance, image.cols, image.rows/5));
        #ifdef DEBUG
            cv::Mat drawing;
            cv::namedWindow("ROI", cv::WINDOW_AUTOSIZE);        
            cv::imshow("ROI", roi);
        #endif
        switch(this->color){
            case red:
                cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), mask);
                cv::bitwise_not(mask, mask);
                roi.setTo(cv::Scalar(255, 255, 255), mask);
                break;
            case green:
                cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, cv::Scalar(70, 100, 40), cv::Scalar(90, 255, 255), mask);
                cv::bitwise_not(mask, mask);
                roi.setTo(cv::Scalar(255, 255, 255), mask);
                break;
            case blue:
                cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, cv::Scalar(110, 100, 40), cv::Scalar(130, 255, 255), mask);
                cv::bitwise_not(mask, mask);
                roi.setTo(cv::Scalar(255, 255, 255), mask);
                break;
            default:
                cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(roi, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 50), mask);
                cv::bitwise_not(mask, mask);
                roi.setTo(cv::Scalar(255, 255, 255), mask);
                break;            
        }
        #ifdef DEBUG
            cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE);        
            cv::imshow("Mask", roi);
        #endif
        cv::cvtColor(roi, mono, cv::COLOR_BGR2GRAY);
        #ifdef DEBUG
            cv::namedWindow("Mono", cv::WINDOW_AUTOSIZE);  
            cv::imshow("Mono", mono);
        #endif
        cv::GaussianBlur(mono, blur, cv::Size(9, 9), 2, 2);
        #ifdef DEBUG
            cv::namedWindow("Blur", cv::WINDOW_AUTOSIZE);        
            cv::imshow("Blur", blur);
        #endif
        cv::threshold(blur, thresh, 100, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        #ifdef DEBUG
            cv::namedWindow("Thresh", cv::WINDOW_AUTOSIZE);        
            cv::imshow("Thresh", thresh);
        #endif
        std::vector< std::vector< cv::Point > > contours;
        std::vector< cv::Vec4i > hierarchy;
        cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        cv::Point position(roi.cols / 2, roi.rows);
        double error = DBL_MAX;
        cv::Point destination;
        for(size_t i = 0; i < contours.size(); i++) {
            cv::Moments moments = cv::moments(contours[i]);
            if (moments.m00 > 10) {
                cv::Point center(moments.m10/moments.m00, moments.m01/moments.m00);
                #ifdef DEBUG
                    cv::cvtColor(thresh, drawing, cv::COLOR_GRAY2BGR);
                    cv::drawContours(drawing, contours, i, cv::Scalar(0, 255, 0), 2, 8, hierarchy, 0, cv::Point());
                    cv::namedWindow("Contours", cv::WINDOW_AUTOSIZE);        
                    cv::imshow("Contours", drawing);
                    cv::circle(drawing, center, 4, cv::Scalar(0, 0, 255), -1, 8, 0 );
                    cv::namedWindow("Final", cv::WINDOW_AUTOSIZE);        
                    cv::imshow("Final", drawing);
                    cv::waitKey(0);
                #endif
                double e = cv::norm(center - position);
                if (e < error) {
                    error = e;
                    destination = center;
                }
            }
        }
        if (error == DBL_MAX) return FLT_MAX;
        return (float)1.0 - 2.0 * destination.x / roi.cols;
    }

    void imageHandler(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv::Mat camera = cv_bridge::toCvShare(msg, "bgr8")->image;
            float new_error = getError(camera, this->distance * camera.rows);
            ROS_INFO_STREAM("Error: " << new_error);
            move(new_error);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'jpg'.", msg->encoding.c_str());
        }
    }

    // If none of the lasers sees anything tells the robot to move randomly, else tells it to follow the line
    void move(float new_error) {
        // See nothing, move randomly
        if (new_error == FLT_MAX) {
            if (this->state == seeLine) {
                setEmotion("sad");
                playSound("disapprove");
                this->state = noSeeLine;
                if (this->error != 0)
                    this->error = this->error/abs(this->error);
            }
            if (this->state != random) {
                lineMove();
            }
            else
                randomMove();
        }
        // See line, follow it
        else {
            if (this->state == noSeeLine || this->state == random) {
                setEmotion("happy");
                playSound("approve");
                this->state = seeLine;
            }
            this->error = new_error;
            lineMove();
        }
    }

    // Sends a random movement message to the robot
    void randomMove() {
        moveWheels(rand() % (5) + 1, rand() % (5) + 1);
    }

    // Sends a line following movement message to the robot
    void lineMove() {

        this->integral += this->error;

        int angle = this->kp * this->error + this->ki * this->integral 
            + this->kd * (this->error - this->previous_error);

        moveWheels(this->speed - angle, this->speed + angle);

        this->previous_error = this->error;

    }

    void setBackCamera() {
        std::vector<com_mytechia_robobo_ros_msgs::KeyValue> parameters;
        com_mytechia_robobo_ros_msgs::KeyValue key_value;
        key_value.key = "camera";
        key_value.value = "back";
        parameters.push_back(key_value);
        if (sendCommand("SET-CAMERA", parameters))
            ROS_INFO_STREAM("Back camera selected");
    }

    void moveTilt(int tilt) {
        std::vector<com_mytechia_robobo_ros_msgs::KeyValue> parameters;
        com_mytechia_robobo_ros_msgs::KeyValue key_value;
        key_value.key = "pos";
        key_value.value = SSTR(tilt);        
        parameters.push_back(key_value);
        key_value.key = "speed";
        key_value.value = "100";        
        parameters.push_back(key_value);
        if (sendCommand("MOVETILT", parameters))
            ROS_INFO_STREAM("Camera tilt position set to: " << parameters[0].value);        
    }

    void movePan(int pan) {
        std::vector<com_mytechia_robobo_ros_msgs::KeyValue> parameters;
        com_mytechia_robobo_ros_msgs::KeyValue key_value;
        key_value.key = "pos";
        key_value.value = SSTR(pan);        
        parameters.push_back(key_value);
        key_value.key = "speed";
        key_value.value = "40";        
        parameters.push_back(key_value);
        if (sendCommand("MOVEPAN", parameters))
            ROS_INFO_STREAM("Camera pan position set to: " << parameters[0].value);
    }

    void setEmotion(string emotion) {
        std::vector<com_mytechia_robobo_ros_msgs::KeyValue> parameters;
        com_mytechia_robobo_ros_msgs::KeyValue key_value;
        key_value.key = "emotion";
        key_value.value = emotion;
        parameters.push_back(key_value);
        sendCommand("SET-EMOTION", parameters);
    }

    void playSound(string sound) {
        std::vector<com_mytechia_robobo_ros_msgs::KeyValue> parameters;
        com_mytechia_robobo_ros_msgs::KeyValue key_value;
        key_value.key = "sound";
        key_value.value = sound;
        parameters.push_back(key_value);
        sendCommand("PLAY-SOUND", parameters);
    }

    void moveWheels(int left_speed, int right_speed) {
        std::vector<com_mytechia_robobo_ros_msgs::KeyValue> parameters;
        com_mytechia_robobo_ros_msgs::KeyValue key_value;
        key_value.key = "lspeed";
        key_value.value = SSTR(left_speed);
        parameters.push_back(key_value);
        key_value.key = "rspeed";
        key_value.value = SSTR(right_speed);
        parameters.push_back(key_value);
        key_value.key = "time";
        key_value.value = "1";
        parameters.push_back(key_value);
        if (sendCommand("MOVE", parameters))
            ROS_INFO_STREAM("Move Wheels: left = " << parameters[0].value << " right = " << parameters[1].value);
    }

    bool sendCommand(string name, std::vector<com_mytechia_robobo_ros_msgs::KeyValue> parameters) {
        com_mytechia_robobo_ros_msgs::Command::Request request;
        com_mytechia_robobo_ros_msgs::Command::Response response;
        request.id = this->command_id;
        this->command_id++;
        request.name = name;
        request.parameters = parameters;
        return client.call(request, response); 
    }

};

int main(int argc, char **argv) {
    
    // Seed the random number generator.
    srand((unsigned) time(NULL) * getpid());

    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "follow_line");

    // Create an object of the LineFollower class that will take care of everything.
    int color = atoi(argv[1]);
    int kp = atoi(argv[2]);
    int ki = atoi(argv[3]);
    int kd = atoi(argv[4]);
    int speed = atoi(argv[5]);
    float distance = atof(argv[6]);

    LineFollower *lineFollower = new LineFollower(color, kp, ki, kd, speed, distance);

    // Let ROS take over.
    while (ros::ok())
        ros::spin();

    return 0;

}

