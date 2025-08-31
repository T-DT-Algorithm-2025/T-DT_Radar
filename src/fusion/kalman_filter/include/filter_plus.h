#include <opencv2/core/hal/interface.h>
#include <vector>
#include <opencv2/core/types.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "opencv2/opencv.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#pragma once

class Kalman_filter_plus {
private:
    cv::KalmanFilter KF;

public:
    float Distance(pcl::PointXY& a, pcl::PointXY& b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }
    float get_time()
    {
        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - timer);
        return duration.count() / 1000.0;
    }
    float                                        last_time = 0;
    std::chrono::steady_clock::time_point        timer;
    float                                        delete_time = 2.0;
    std::vector<std::pair<double, pcl::PointXY>> history;
    std::vector<std::pair<int, int>>             detect_history;
    int                                          max_history = 20;

    pcl::PointXY predict_point;
    float        detect_r = 1;
    float        car_speed = 2;
    float        car_max_speed = 2.5;
    cv::Scalar   color;
    bool         has_updated = false;
    cv::Mat      Q = cv::Mat::zeros(4, 4, CV_32F);
    cv::Mat      R = cv::Mat::zeros(2, 2, CV_32F);
    float        dt_ = 0.1f;
    float        sigma_q_x = 50.0f;
    float        sigma_q_y = 50.0f;
    float        sigma_r_x = 0.1f;
    float        sigma_r_y = 0.1f;

    Kalman_filter_plus(pcl::PointXY& input, rclcpp::Time time)
    {
        predict_point = input;
        history.push_back(std::make_pair(GetTimeByRosTime(time), input));
        timer = std::chrono::steady_clock::now();
        color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
        int          stateSize = 4;
        int          measSize = 2;
        int          contrSize = 0;
        unsigned int type = CV_32F;
        KF.init(stateSize, measSize, contrSize, type);

        cv::Mat state(stateSize, 1, type);
        cv::Mat meas(measSize, 1, type);
        meas.at<float>(0) = input.x;
        meas.at<float>(1) = input.y;

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(2) = meas.at<float>(1);
        KF.statePost = state;
        KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, dt_, 0, 0, 0, 1,
                               0, 0, 0, 0, 1, dt_, 0, 0, 0, 1);

        KF.measurementMatrix =
            (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 0, 1, 0);

        KF.processNoiseCov =
            (cv::Mat_<float>(4, 4) << sigma_q_x * pow(dt_, 3) / 3,
             sigma_q_x * pow(dt_, 2) / 2, 0, 0, sigma_q_x * pow(dt_, 2) / 2,
             sigma_q_x * pow(dt_, 1), 0, 0, 0, 0,
             sigma_q_y * pow(dt_, 3) / 3, sigma_q_y * pow(dt_, 2) / 2, 0, 0,
             sigma_q_y * pow(dt_, 2) / 2, sigma_q_y * pow(dt_, 1));

        KF.measurementNoiseCov =
            (cv::Mat_<float>(2, 2) << sigma_r_x, 0, 0, sigma_r_y);

        setIdentity(KF.errorCovPost, cv::Scalar::all(1));
        has_updated = true;
    }

    ~Kalman_filter_plus() {}

    int get_color()
    {
        if (detect_history.size() == 0) {
            return 1;
        }
        int red = 0;
        int blue = 0;
        for (auto& color : detect_history) {
            if (color.first == 0) {
                blue++;
            } else {
                red++;
            }
        }
        if (red > blue) {
            return 2;
        } else {
            return 0;
        }
    }

    int get_number()
    {
        int                color = get_color();
        std::map<int, int> number_map;
        for (auto& number : detect_history) {
            if (number.first == color) {
                number_map[number.second]++;
            }
        }
        int max_count = 0;
        int max_number = 0;
        for (auto& entry : number_map) {
            if (entry.second > max_count) {
                max_count = entry.second;
                max_number = entry.first;
            }
        }
        return max_number;
    }

    void update(pcl::PointXY& input, rclcpp::Time time)
    {
        cv::Mat meas = cv::Mat::zeros(2, 1, CV_32F);
        meas.at<float>(0) = input.x;
        meas.at<float>(1) = input.y;
        KF.correct(meas);
        predict_point.x = KF.statePost.at<float>(0);
        predict_point.y = KF.statePost.at<float>(2);
        has_updated = true;
        last_time = 0;
        auto temp_point = input;
        history.push_back(
            std::make_pair(GetTimeByRosTime(time), temp_point));
        if (history.size() > max_history) {
            history.erase(history.begin());
        }
    }

    void update_predict_point()
    {
        dt_ = get_time();
        timer = std::chrono::steady_clock::now();
        auto result = KF.predict();
        last_time += dt_;
        predict_point.x = result.at<float>(0);
        predict_point.y = result.at<float>(2);
    }

    bool match(pcl::PointXY& input)
    {
        if (Distance(predict_point, input) <
            car_max_speed * dt_ + detect_r) {
            return true;
        } else {
            return false;
        }
    }

    void camera_match(rclcpp::Time& time, pcl::PointXY& input, int color,
                      int number)
    {
        const double TIME_THRESHOLD = 1.0f;
        double       input_time = GetTimeByRosTime(time);
        double       differ_time = 1000;
        pcl::PointXY match_point;
        for (auto& point : history) {
            auto differ = abs(point.first - input_time);
            if (differ < differ_time) {
                differ_time = differ;
                match_point = point.second;
            }
        }
        if (differ_time > TIME_THRESHOLD) {
            return;
        }
        if (Distance(match_point, input) < detect_r) {
            detect_history.push_back(std::make_pair(color, number));
            if (detect_history.size() > max_history) {
                detect_history.erase(detect_history.begin());
            }
        }
    }
    static double GetTimeByRosTime(rclcpp::Time& ros_time)
    {
        double ros_time_value = ros_time.nanoseconds() / 1e9;
        return ros_time_value;
    }
};
