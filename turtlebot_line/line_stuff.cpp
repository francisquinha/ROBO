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

const float KP = 0.4 * M_PI;
const float KI = 0;
const float KD = 0;

class LineFollower {

public:
    LineFollower() {

        this->new_error = FLT_MAX;
        this->error = 0;
        this->previous_error = 0;
        this->integral = 0;

        // Initiate the publisher.
        pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        // Initiate the subscriber.
        // We subscribe to the robots camera to detect the line to follow
        image_transport::ImageTransport it(nh);
        sub = it.subscribe("camera/rgb/image_raw", 1, &LineFollower::imageHandler, this);
        
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    image_transport::Subscriber sub;   
    float new_error;
    float error;
    float previous_error;
    float integral;

    float getError(cv::Mat image, int distance) {

        cv::Mat roi, mono, blur, thresh;//, thin, sub;
        roi = image(cv::Rect(10, distance, image.cols - 20, image.rows / 12));
        cv::cvtColor(roi, mono, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(mono, blur, cv::Size(9, 9), 2, 2);
        cv::threshold(blur, thresh, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
//        cv::ximgproc::thinning(thresh, thin, cv::ximgproc::THINNING_GUOHALL );
//        sub = thin(cv::Rect(1, 1, thin.cols - 2, thin.rows - 2));    
//        vector< cv::Vec4i > lines;
//        cv::HoughLinesP(sub, lines, 1, CV_PI/180, 10, 1, 1);
//       cv::Vec4i lineLeft;
//       cv::Vec4i lineRight;
//        cv::Scalar yellow = cv::Scalar(0, 255, 255);
//        cv::Scalar green = cv::Scalar(0, 255, 0);
//        cv::Scalar red = cv::Scalar(0, 0, 255);
//        cv::Scalar blue = cv::Scalar(255, 0, 0);
//        cv::Mat drawing = cv::Mat::zeros(roi.size(), CV_8UC3);
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
/*        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i line = lines[i];
            cv::Point p1 = cv::Point(line[0], line[1]);
            cv::Point p2 = cv::Point(line[2], line[3]);
//            cv::line(drawing, p1, p2, yellow, 3, CV_AA);
            vector< cv::Point > v;
            v.push_back(p1);
            v.push_back(p2);
            cv::Rect rect = cv::boundingRect(v);
//            cv::rectangle(drawing, rect, green);
            double center = 1.0 - 2.0 * (rect.x + rect.width / 2) / sub.cols;
//            ROS_INFO_STREAM("Center: " << cent << " / " << center);
//            cv::line(drawing, cv::Point(cent, 0), cv::Point(cent, roi.rows), red, 1, CV_AA);
            if (0 <= center && center < centerLeft) {
                centerLeft = center;
//                lineLeft = line;
            }
            if (centerRight < center && center <= 0) {
                centerRight = center;
//                lineRight = line;
            }
        }*/
//        cv::line(drawing, cv::Point(lineLeft[0], lineLeft[1]), cv::Point(lineLeft[2], lineLeft[3]), blue, 2, CV_AA);
//        cv::line(drawing, cv::Point(lineRight[0], lineRight[1]), cv::Point(lineRight[2], lineRight[3]), blue, 2, CV_AA);
//        cv::namedWindow("Thin", cv::WINDOW_AUTOSIZE);
//        cv::imshow("Thin", thin);
//        cv::namedWindow("Lines", cv::WINDOW_AUTOSIZE);
//        cv::imshow("Lines", drawing);
//        cv::waitKey(0); 
        if (distance == DBL_MAX) return FLT_MAX;
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
            geometry_msgs::Twist out_msg;
            out_msg.linear.x = 0.6;
            this->pub.publish(out_msg);
        }
        // See line, follow it
        else {
            this->error = this->new_error;
            lineMove();
        }
    }

    // Sends a line following movement message to the robot
    void lineMove() {

        this->integral -= this->error;

        // Create the message. 
        geometry_msgs::Twist out_msg;

        // if the line is in the center move faster, otherwise slower
        out_msg.linear.x = 0.6 - 0.3 * abs(this->error);
        // turn according to line position
        out_msg.angular.z = KP * this->error + KI * this->integral + KD * (this->error - this->previous_error);

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

    // Create an object of the Wall class that will take care of everything.
    LineFollower *lineFollower = new LineFollower();

    // Let ROS take over.
    while (ros::ok())
        ros::spin();

    return 0;

}

