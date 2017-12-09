// This program gets camera rgb images and makes turtlebot follow a black line on white background
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <float.h>

using namespace std;

const float MAX_ERROR = 0.5;

class Line {

public:
    Line(int bias) {

        this->random_signal = (rand() % 2) * 2 - 1;;
        this->counter = 0;
        this->next = (rand() % 10 + 1) * 10;

        this->bias = bias > 0;
        this->position = (this->bias ? - FLT_MAX : FLT_MAX);
        this->last_position = (this->bias ? - FLT_MAX : FLT_MAX);

        // Initiate the publisher.
        pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        // Initiate the subscriber.
        // We subscribe to the robots camera to detect the line to follow
        image_transport::ImageTransport it(nh);
        sub = it.subscribe("camera/rgb/image_raw", 1, &Line::imageHandler, this);
        
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    image_transport::Subscriber sub;   
    int random_signal; // Determines whether the robot rotates counterclockwise or clockwise.
    int counter; // Incremented in each robot movement published message.
    int next; // When to reset the counter and randomize the signal.
    bool bias; // True if robot follows left side of the line, false if robot follows right side of the line
    float position; // Line position in the robots camera image:
                    // 0 if it is in the center, 1 if it is on the left, -1 if it is on the right, 
                    // between -1 and 1 if the black line is in the image,
                    // -FLT_MAX or FLT_MAX otherwise
    float last_position; // Last known position

    float linePosition(cv::Mat image, int y) {

        cv::Mat roi, mono, blur, thresh;
        roi = image(cv::Rect(10, y, image.cols - 20, image.rows / 12));
        cv::cvtColor(roi, mono, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(mono, blur, cv::Size(9, 9), 2, 2);
        cv::threshold(blur, thresh, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        std::vector< std::vector< cv::Point > > contours;
        std::vector< cv::Vec4i > hierarchy;
        cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        double minMaxCx = (this->bias ? -DBL_MAX : DBL_MAX);
        for(size_t i = 0; i < contours.size(); i++) {
            cv::Moments mu = cv::moments(contours[i], false);
            if (mu.m00 > 100.0) {
                cv::Rect r = cv::boundingRect(contours[i]);
                double cx;
                if (this->bias) {
                    cx = r.x + r.width - 12;
                    if (cx > minMaxCx)
                        minMaxCx = cx;
                }
                else {
                    cx = r.x + 12;
                    if (minMaxCx > cx)
                        minMaxCx = cx;
                }
            }
        }

        if (minMaxCx == DBL_MAX)
            return FLT_MAX;
        if (minMaxCx == -DBL_MAX)
            return -FLT_MAX;
        return 1.0 - 2.0 * (float)minMaxCx/roi.cols;
    }

    void imageHandler(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv::Mat camera = cv_bridge::toCvShare(msg, "bgr8")->image;
            this->position = linePosition(camera, 0.7 * camera.rows);
            if (this->position != FLT_MAX && this->position != -FLT_MAX)
                this->last_position = this->position;
            else
                this->position = this->last_position;
            ROS_INFO_STREAM("Line position: " << this->position);
            move();
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'jpg'.", msg->encoding.c_str());
        }
    }

    // If none of the lasers sees anything tells the robot to move randomly, else tells it to follow the wall
    void move() {
        // See nothing, move randomly
        if (this->position == FLT_MAX || this->position == -FLT_MAX) 
            randomMove();
        // See line, follow it
        else
            lineMove();
    }

    // Sends a random movement message to the robot
    void randomMove() {

        this->counter++;    

        // Whenever the counter reaches next, reset it, randomize the signal 
        // and randomize the next check.
        if (this->counter == this->next) {
            this->counter = 0;
            this->random_signal = (rand() % 2) * 2 - 1;
            this->next = (rand() % 10 + 1) * 10;
        }

        // Create and fill in the message. 
        // The other four fields, which are ignore by the robot, default to 0.
        geometry_msgs::Twist out_msg;
        out_msg.linear.x = double(rand()) / double(RAND_MAX);
        out_msg.angular.z = this->random_signal * double(rand()) / double(RAND_MAX);

        // Publish the message.
        this->pub.publish(out_msg);

        // Send a message to rosout with the details.
        //ROS_INFO_STREAM("Random velocity: " << " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);

    }

    // Sends a line following movement message to the robot
    void lineMove() {

        // Create the message. 
        geometry_msgs::Twist out_msg;

        // if the line is in the center move faster, otherwise slower
        out_msg.linear.x = 1 - 0.5 * abs(this->position);
        // turn according to line position
        out_msg.angular.z = this->position * 0.5 * M_PI;

        this->pub.publish(out_msg);
        //ROS_INFO_STREAM("Velocity: "    << " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);

    }

};

int main(int argc, char **argv) {
    
    // Seed the random number generator.
    srand((unsigned) time(NULL) * getpid());

    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "follow_line");

    // Create an object of the Wall class that will take care of everything.
    int bias = atoi(argv[1]);
    Line *line = new Line(bias);

    // Let ROS take over.
    ros::spin();

    return 0;

}

