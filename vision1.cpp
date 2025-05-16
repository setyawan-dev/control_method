#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <newvis/info.h>
#include <newvis/koor.h>
#include <time.h>
#include <mutex>
#include <std_srvs/SetBool.h>

using namespace cv;
using namespace ros;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace newvis;
using namespace std_srvs;
using namespace std_msgs;


class Vision {
public:
    Vision() {
        NodeHandle nh;
        pub_image = nh.advertise<Image>("processed_image", 10);
        pub_info = nh.advertise<info>("kondisi", 10);
        pub_posisi = nh.advertise<koor>("koordinat", 10);
        pub_kamera1 = nh.advertise<Image>("/kamera1/image_raw", 10);
        pub_mask = nh.advertise<Image>("/kamera1/mask/image_raw", 10);
        pub_koor_kamera1 = nh.advertise<koor>("/koordinat/kamera1", 10);
        // sub_image = nh.subscribe("/kamera_depan/image_raw", 10, &Vision::imageCallback, this);
        sub_image = nh.subscribe("/usb_cam/image_raw", 10, &Vision::imageCallback, this);
        srv_control = nh.advertiseService("/kamera1/status", &Vision::kondisi_cb, this);
        timer = nh.createTimer(Duration(3.0), &Vision::timer_break, this, true);
        status = true;
        fps = 0.0;
        prev_time = (double)cv::getTickCount();

    }
    
    bool kondisi_cb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
        status = request.data;
        response.success = true;
        response.message = status ? "ON" : "OFF";
        if (status) {
            ROS_INFO("Kamera 1 ON");
        } else {
            ROS_WARN("Kamera 1 OFF");
        }
        return true;
    }

    void timer_break(const TimerEvent&){
        status = true;
        ROS_INFO("Kamera 1 Ready");
    }

    void imageCallback(const ImageConstPtr& msg) {
        int frame_count = 0;
        double start_time = 0;
        if(!status){
            return;
        }
        CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, image_encodings::BGR8);
            frame = cv_ptr->image;
            ROS_INFO_ONCE("Launch Kamera 1");
            resize(frame, frame, Size(320, 240));
            if (!frame.empty()) {
                ROS_INFO_ONCE("connect 1");
            } else {
                ROS_WARN("Tidak ada data gambar.");
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Konfigurasi HSV
        // int rh = 35, rs = 100, rv = 100;
        // int th = 85, ts = 255, tv = 255;

        int rh = 0, rs = 50, rv = 5;
        int th = 30, ts = 255, tv = 255;

        Mat blurred_frame, hsv, mask, erode_img, dilate_img;
        GaussianBlur(frame, blurred_frame, Size(5, 5), 0);
        cvtColor(blurred_frame, hsv, COLOR_BGR2HSV);
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        inRange(hsv, Scalar(rh, rs, rv), Scalar(th, ts, tv), mask);
        cv::erode(mask, erode_img, kernel);
        cv::dilate(erode_img, dilate_img, kernel);

        vector<vector<Point>> contours;
        findContours(dilate_img, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        koor mov;
        Point2f detek(-1, -1);
        int tolerance = 50;

        for (const auto& c : contours) {
            Moments M = moments(c);
            if (M.m00 > 1000) {
                Point2f center;
                float radius;
                minEnclosingCircle(c, center, radius);
                int x = static_cast<int>(M.m10 / M.m00);
                int y = static_cast<int>(M.m01 / M.m00);

                if (detek.x != -1 && abs(x - detek.x) < tolerance && abs(y - detek.y) < tolerance) {
                    continue;
                } else {
                    circle(frame, Point(x, y), 3, Scalar(0, 0, 0), -1);
                    circle(frame, center, static_cast<int>(radius), Scalar(0, 255, 255), 2);
                    detek = Point2f(x, y);
                }

                mov.posx = static_cast<int>(center.x - 160);
                mov.posy = static_cast<int>(center.y - 120);
                ROS_INFO("Koordinat X: %d", static_cast<int>(center.x - 160));
                ROS_INFO("Koordinat Y: %d", static_cast<int>(center.y - 120));
            }
        }

        double current_time = (double)cv::getTickCount();
        double fps = cv::getTickFrequency() / (current_time - prev_time);
        prev_time = current_time;

        ostringstream fps_text;
        fps_text << fixed << setprecision(1) << fps;
        putText(frame, "FPS: " + fps_text.str(), Point(10, 30),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
        static double last_log_time = 0;
        if (current_time / getTickFrequency() - last_log_time >= 1.0) {
            ROS_INFO("FPS: %.2f", fps);
            last_log_time = current_time / getTickFrequency();
        }
        line(frame, Point(160, 0), Point(160, 240), Scalar(255, 255, 255), 1);
        line(frame, Point(0, 120), Point(320, 120), Scalar(255, 255, 255), 1);
        // putText(frame, "KAMERA 1", Point(0, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
        // imshow("kamera1", frame);
        waitKey(1);

        try {
            pub_posisi.publish(mov);
            pub_kamera1.publish(CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg());
            // pub_mask.publish(CvImage(std_msgs::Header(), "mono8", mask).toImageMsg());
            pub_koor_kamera1.publish(mov);
        } catch (const exception& e) {
            ROS_ERROR("ERROR 1");
            Duration(3.0).sleep();
            ROS_ERROR("%s", e.what());
        }
    }

private:
Mat frame;
    Publisher pub_image, pub_posisi, pub_info, pub_kamera1, pub_mask, pub_koor_kamera1;
    Subscriber sub_image, sub_interkom, sub_info;
    ServiceServer srv_control;
    Timer timer;
    volatile int cam1 = 1;
    int a = 1;
    mutex cam_mutex;
    bool status;
    double fps, last_time, prev_time;
};

int main(int argc, char** argv) {
    init(argc, argv, "opencv_ros_example");
    Vision vision;
    spin();
    return 0;
}
