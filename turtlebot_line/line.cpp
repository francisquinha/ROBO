// This program gets camera rgb images and makes turtlebot follow a black line on white background
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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

using namespace std;

class LineFollower {

public:
    LineFollower(int color, float kp, float ki, float kd) {

        this->random_signal = (rand() % 2) * 2 - 1;;
        this->counter = 0;
        this->next = (rand() % 10 + 1) * 10;

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

        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        ROS_INFO_STREAM("PID Constants: " << this->kp << ", " << this->ki << ", " << this->kd);


        this->new_error = FLT_MAX;
        this->error = 0;
        this->previous_error = 0;
        this->integral = 0;

        // Initiate the publisher.
        this->pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        // Initiate the subscriber.
        // We subscribe to the robots camera to detect the line to follow
        image_transport::ImageTransport it(nh);
        ros::Duration(1).sleep();
        this->sub = it.subscribe("camera/rgb/image_raw", 1, &LineFollower::imageHandler, this);
        
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    image_transport::Subscriber sub;
    int random_signal; // Determines whether the robot rotates counterclockwise or clockwise.
    int counter; // Incremented in each robot movement published message.
    int next; // When to reset the counter and randomize the signal.
    enum Color {black, red, green, blue};
    Color color;
    float kp;
    float ki;
    float kd;
    float new_error;
    float error;
    float previous_error;
    float integral;

    float getError(cv::Mat image, int distance) {
        cv::Mat roi, hsv, mask1, mask2, mask, mono, blur, thresh;
        roi = image(cv::Rect(10, distance, image.cols - 20, image.rows / 12));
        switch(this->color){
            case red:
                cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), mask1);
                cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask2);
                cv::bitwise_not(mask1 + mask2, mask);
                roi.setTo(cv::Scalar(255, 255, 255), mask);
                break;
            case green:
                cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255), mask1);
                cv::bitwise_not(mask1, mask);
                roi.setTo(cv::Scalar(255, 255, 255), mask);
                break;
            case blue:
                cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, cv::Scalar(100, 100, 100), cv::Scalar(140, 255, 255), mask1);
                cv::bitwise_not(mask1, mask);
                roi.setTo(cv::Scalar(255, 255, 255), mask);
                break;
            default:
                break;            
        }
        cv::cvtColor(roi, mono, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(mono, blur, cv::Size(9, 9), 2, 2);
        cv::threshold(blur, thresh, 250, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

/*        cv::namedWindow("Thresh", cv::WINDOW_AUTOSIZE);        
        cv::imshow("Thresh", thresh);
        cv::waitKey(0);
*/
        std::vector< std::vector< cv::Point > > contours;
        std::vector< cv::Vec4i > hierarchy;
        cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        cv::Point position(roi.cols / 2, roi.rows);
        double error = DBL_MAX;
        cv::Point destination;
        for(size_t i = 0; i < contours.size(); i++) {
            cv::Moments moments = cv::moments(contours[i]);
            if (moments.m00 > 0) {
                cv::Point center(moments.m10/moments.m00, moments.m01/moments.m00);
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
            this->new_error = getError(camera, 0.8 * camera.rows);
            ROS_INFO_STREAM("Error: " << this->new_error);
            move();
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'jpg'.", msg->encoding.c_str());
        }
    }

    // If none of the lasers sees anything tells the robot to move randomly, else tells it to follow the wall
    void move() {
        // See nothing, move forward
        if (this->new_error == FLT_MAX) {
            randomMove();
        }
        // See line, follow it
        else {
            this->error = this->new_error;
            lineMove();
        }
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
        ROS_INFO_STREAM("Random velocity: " << " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);

    }

    // Sends a line following movement message to the robot
    void lineMove() {

        this->integral += this->error;

        // Create the message. 
        geometry_msgs::Twist out_msg;

        out_msg.linear.x = 0.6;
        // turn according to line position
        out_msg.angular.z = this->kp * this->error + this->ki * this->integral + this->kd * (this->error 
            - this->previous_error);

        this->pub.publish(out_msg);
        ROS_INFO_STREAM("Velocity: "    << " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);

        this->previous_error = this->error;

    }

};

int main(int argc, char **argv) {
    
    // Seed the random number generator.
    srand((unsigned) time(NULL) * getpid());

    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "follow_line");

    // Create an object of the LineFollower class that will take care of everything.
    int color = atoi(argv[1]);
    float kp = atof(argv[2]);
    float ki = atof(argv[3]);
    float kd = atof(argv[4]);
    LineFollower *lineFollower = new LineFollower(color, kp, ki, kd);

    // Let ROS take over.
    while (ros::ok())
        ros::spin();

    return 0;

}

