#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


namespace cone_threshold {
    void thresh_blue(cv::Mat hsv_in, cv::Mat out) {
        cv::Scalar lower(100, 100, 100);  // H S V
        cv::Scalar upper(100, 100, 100);  // H S V
        cv::inRange(out, lower, upper, out);
    }

    void cone_threshold::thresh_yellow(cv::Mat hsv_in, cv::Mat out) {
        cv::Scalar lower(100, 100, 100);  // H S V
        cv::Scalar upper(100, 100, 100);  // H S V
        cv::inRange(out, lower, upper, out);
    }
}
