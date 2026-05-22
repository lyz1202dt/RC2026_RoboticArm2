#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <thread>
#include <mutex>
#include <optional>
#include <sstream>
#include <iomanip>

// ============================================================
// 卡尔曼滤波器 (纯视觉算法部分保持不变)
// ============================================================
struct KalmanFilter2D {
    float pos_noise  = 0.1f;
    float vel_noise  = 2.0f;
    float meas_noise = 20.0f;

    cv::KalmanFilter kf;
    bool initialized = false;

    KalmanFilter2D() {
        kf.init(4, 2, 1, CV_32F);
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);
        kf.measurementMatrix = (cv::Mat_<float>(2, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0);
    }

    void initialize(const cv::Point2f& pt) {
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(0));
        kf.processNoiseCov.at<float>(0, 0) = pos_noise;
        kf.processNoiseCov.at<float>(1, 1) = pos_noise;
        kf.processNoiseCov.at<float>(2, 2) = vel_noise;
        kf.processNoiseCov.at<float>(3, 3) = vel_noise;

        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(meas_noise));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(100.0));

        kf.statePost.at<float>(0) = pt.x;
        kf.statePost.at<float>(1) = pt.y;
        kf.statePost.at<float>(2) = 0.0f;
        kf.statePost.at<float>(3) = 0.0f;

        initialized = true;
    }

    void reset() {
        initialized = false;
        kf.statePost    = cv::Mat::zeros(4, 1, CV_32F);
        kf.errorCovPost = cv::Mat::eye(4, 4, CV_32F) * 100.0f;
    }
};

class RedSegmentation {
public:
    struct Params {
        int h_low_min = 0, h_low_max = 10;
        int h_high_min = 160, h_high_max = 180;
        int s_min = 50, s_max = 255;
        int v_min = 85, v_max = 255;
        float max_vertical_deviation = 30.0f;
        float min_vertical_fill_ratio = 0.9f;
        float min_aspect_ratio = 8.0f;
        double min_compactness = 0.7;
    };

    RedSegmentation() {
        try {
            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,19));
        } catch (...) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("RedSegmentation"), "Failed to create morphology kernel");
        }
    }

    void process(const cv::Mat& input, cv::Mat& mask) {
        if (input.empty()) return;
        cv::Mat hsv;
        cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);
        cv::Mat m1, m2;
        cv::inRange(hsv, cv::Scalar(p.h_low_min, p.s_min, p.v_min),
                         cv::Scalar(p.h_low_max, p.s_max, p.v_max), m1);
        cv::inRange(hsv, cv::Scalar(p.h_high_min, p.s_min, p.v_min),
                         cv::Scalar(p.h_high_max, p.s_max, p.v_max), m2);
        cv::bitwise_or(m1, m2, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    }

    Params p;
private:
    cv::Mat kernel;
};

// ============================================================
// ROS2 节点类定义
// ============================================================
class VisionPositionPublisher : public rclcpp::Node {
public:
    VisionPositionPublisher() : Node("vision_position_publisher"), is_running_(false), show_visualization_(true) {
        RCLCPP_INFO(this->get_logger(), "1. 创建TF buffer...");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        RCLCPP_INFO(this->get_logger(), "2. 创建TF listener...");
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "3. 创建TF broadcaster...");
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "4. 创建静态TF broadcaster...");
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // ==================== HANDEYE CALIBRATION RESULT ====================
        // 在这里填入手眼标定矩阵 (Camera in Flange Frame)
        // 标定完成后替换为实际矩阵，格式：前3x3旋转，第4列前3行平移 (单位：米)
        // 【注释】手眼标定：以下两行被注释以跳过标定。恢复方法：
        //   1. 取消下面两行的注释
        //   2. 填入标定结果矩阵替换 T_flange2cam_ 的 Identity()
        //   3. 取消构造函数中 publishStaticHandEyeTransform() 的注释
        // RCLCPP_INFO(this->get_logger(), "5. 初始化手眼矩阵...");
        // T_flange2cam_ = Eigen::Matrix4d::Identity();
        // 例如:
        // T_flange2cam_ << 1.0, 0.0, 0.0, 0.05,
        //                  0.0, 1.0, 0.0, 0.02,
        //                  0.0, 0.0, 1.0, 0.10,
        //                  0.0, 0.0, 0.0, 1.0;
        // ====================================================================

        // 发布相机的静态 TF，方便在 RViz 里查看
        // 【注释】手眼标定：以下代码需要手眼标定矩阵 T_flange2cam_ 正确赋值后才能启用。
        // 要恢复：取消下行注释，并确保 T_flange2cam_ 填入标定结果。
        // publishStaticHandEyeTransform();

        // 启动视觉处理线程，避免阻塞 ROS 的 executor
        is_running_ = true;
        vision_thread_ = std::thread(&VisionPositionPublisher::visionLoop, this);
        RCLCPP_INFO(this->get_logger(), "Vision Position Publisher node initialized and started.");
    }

    ~VisionPositionPublisher() {
        is_running_ = false;
        if (vision_thread_.joinable()) {
            vision_thread_.join();
        }
    }

private:
    /* ========== 手眼标定发布函数（当前已注释，未使用） ==========
    // 恢复方法：
    //   1. 取消整个函数体的注释
    //   2. 确保 T_flange2cam_ 填入标定结果（见构造函数中的注释）
    //   3. 取消构造函数中 publishStaticHandEyeTransform() 的注释
    void publishStaticHandEyeTransform() {
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = this->now();
        static_tf.header.frame_id = "tool0";
        static_tf.child_frame_id = "camera_link";

        Eigen::Affine3d affine_flange2cam(T_flange2cam_);
        static_tf.transform = tf2::eigenToTransform(affine_flange2cam).transform;

        static_tf_broadcaster_->sendTransform(static_tf);
    }
    ================================================================ */

    void visionLoop() {
        // ============================================================
        // Step A: 启动相机 (带重试)
        // ============================================================
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
        rs2::align align_to_color(RS2_STREAM_COLOR);
        rs2::pipeline_profile profile;

        RCLCPP_INFO(this->get_logger(), "[1/5] Opening RealSense pipeline...");
        bool camera_started = false;
        for (int retry = 0; retry < 30 && is_running_ && rclcpp::ok(); ++retry) {
            try {
                profile = pipe.start(cfg);
                camera_started = true;
                break;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Camera start 失败(重试 %d/30): %s", retry + 1, e.what());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        if (!camera_started) {
            RCLCPP_FATAL(this->get_logger(), "[FAIL] 无法启动 RealSense 相机，退出视觉线程");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "[2/5] Camera started");

        auto stream_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        rs2_intrinsics intrin = stream_profile.get_intrinsics();

        RedSegmentation detector;
        cv::Mat color_image, mask_image;

        const int   MAX_TARGETS    = 10;
        const int   MAX_LOST_FRAMES = 10;
        const float MATCH_DIST_THR = 80.0f;

        struct TrackedTarget {
            KalmanFilter2D kf;
            int   lost_frames = 0;
            bool  active      = false;
        };
        std::vector<TrackedTarget> trackers(MAX_TARGETS);

        RCLCPP_INFO(this->get_logger(), "[3/5] Entering vision loop");

        while (is_running_ && rclcpp::ok()) {
            try {
                // ========================================================
                // Step B: 等待帧
                // ========================================================
                rs2::frameset frames;
                try {
                    frames = pipe.wait_for_frames(5000);
                } catch (const std::exception&) {
                    continue;
                }

                frames = align_to_color.process(frames);
                rs2::video_frame color_frame = frames.get_color_frame();
                rs2::depth_frame depth_frame = frames.get_depth_frame();
                if (!color_frame || !depth_frame) continue;

                color_image = cv::Mat(cv::Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                detector.process(color_image, mask_image);

                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                std::vector<cv::RotatedRect> valid_sticks;
                int rightmost_idx = -1;
                float max_x = -1.0f;

                for (const auto& cnt : contours) {
                    cv::RotatedRect rect = cv::minAreaRect(cnt);
                    float w = rect.size.width;
                    float h = rect.size.height;

                    if (std::max(w, h) < 50.0f) continue;
                    if (w * h < 3000.0f) continue;

                    float aspect_ratio = std::max(w, h) / std::max(std::min(w, h), 1.0f);
                    if (aspect_ratio < detector.p.min_aspect_ratio) continue;

                    float long_axis_angle = (w < h) ? rect.angle : rect.angle + 90.0f;
                    if (long_axis_angle > 90.0f) long_axis_angle -= 180.0f;
                    if (long_axis_angle < -90.0f) long_axis_angle += 180.0f;
                    if (std::abs(long_axis_angle) > detector.p.max_vertical_deviation) continue;

                    cv::Size rot_size(static_cast<int>(std::min(w, h)), static_cast<int>(std::max(w, h)));
                    if (rot_size.width <= 0 || rot_size.height <= 0) continue;

                    float rot_angle = long_axis_angle;
                    cv::Mat rot_mat = cv::getRotationMatrix2D(rect.center, rot_angle, 1.0);
                    rot_mat.at<double>(0, 2) += (rot_size.width / 2.0 - rect.center.x);
                    rot_mat.at<double>(1, 2) += (rot_size.height / 2.0 - rect.center.y);

                    cv::Mat rotated_roi;
                    cv::warpAffine(mask_image, rotated_roi, rot_mat, rot_size, cv::INTER_NEAREST);

                    int white_pixels = cv::countNonZero(rotated_roi);
                    double rot_rect_area = rotated_roi.cols * rotated_roi.rows;
                    if (white_pixels / rot_rect_area < detector.p.min_compactness) continue;

                    int filled_rows = 0;
                    for (int r = 0; r < rotated_roi.rows; ++r) {
                        if (cv::countNonZero(rotated_roi.row(r)) > 0) filled_rows++;
                    }
                    if ((float)filled_rows / rotated_roi.rows < detector.p.min_vertical_fill_ratio) continue;

                    valid_sticks.push_back(rect);
                }

            int cur_count = (int)valid_sticks.size();

            for (auto& t : trackers) {
                if (t.active) t.kf.kf.predict();
            }

            std::vector<int> det_to_tracker(cur_count, -1);
            std::vector<bool> tracker_matched(MAX_TARGETS, false);

            for (int d = 0; d < cur_count; ++d) {
                cv::Point2f det_pt = valid_sticks[d].center;
                float best_dist = MATCH_DIST_THR;
                int   best_t    = -1;
                for (int t = 0; t < MAX_TARGETS; ++t) {
                    if (!trackers[t].active) continue;
                    if (tracker_matched[t]) continue;
                    float px = trackers[t].kf.kf.statePre.at<float>(0);
                    float py = trackers[t].kf.kf.statePre.at<float>(1);
                    float dist = std::hypot(det_pt.x - px, det_pt.y - py);
                    if (dist < best_dist) {
                        best_dist = dist;
                        best_t    = t;
                    }
                }
                if (best_t >= 0) {
                    det_to_tracker[d] = best_t;
                    tracker_matched[best_t] = true;
                }
            }

            for (int d = 0; d < cur_count; ++d) {
                int t = det_to_tracker[d];
                if (t >= 0) {
                    cv::Mat meas = (cv::Mat_<float>(2, 1) << valid_sticks[d].center.x, valid_sticks[d].center.y);
                    cv::Mat est = trackers[t].kf.kf.correct(meas);
                    trackers[t].kf.initialized = true;
                    trackers[t].lost_frames = 0;
                    valid_sticks[d].center.x = est.at<float>(0);
                    valid_sticks[d].center.y = est.at<float>(1);
                } else {
                    for (int t2 = 0; t2 < MAX_TARGETS; ++t2) {
                        if (!trackers[t2].active) {
                            trackers[t2].kf.reset();
                            trackers[t2].kf.initialize(valid_sticks[d].center);
                            trackers[t2].active      = true;
                            trackers[t2].lost_frames = 0;
                            break;
                        }
                    }
                }
            }

            for (int t = 0; t < MAX_TARGETS; ++t) {
                if (!trackers[t].active) continue;
                if (tracker_matched[t]) continue;
                trackers[t].lost_frames++;
                if (trackers[t].lost_frames > MAX_LOST_FRAMES) {
                    trackers[t].kf.reset();
                    trackers[t].active = false;
                }
            }

            for (auto& s : valid_sticks) {
                s.center.x = std::max(0.0f, std::min(s.center.x, (float)(color_image.cols - 1)));
                s.center.y = std::max(0.0f, std::min(s.center.y, (float)(color_image.rows - 1)));
            }

            rightmost_idx = -1;
            max_x = -1.0f;
            for (int i = 0; i < (int)valid_sticks.size(); ++i) {
                if (valid_sticks[i].center.x > max_x) {
                    max_x = valid_sticks[i].center.x;
                    rightmost_idx = i;
                }
            }

            // -------------------------------------------------------------
            // 核心 ROS2 TF 发布逻辑
            // -------------------------------------------------------------
            bool target_detected_this_frame = false;
            Eigen::Vector3d current_target_base;

            if (rightmost_idx != -1) {
                int cx = std::max(0, std::min((int)valid_sticks[rightmost_idx].center.x, depth_frame.get_width()  - 1));
                int cy = std::max(0, std::min((int)valid_sticks[rightmost_idx].center.y, depth_frame.get_height() - 1));

                const int DEPTH_PATCH = 10;
                std::vector<float> depth_samples;
                for (int dy = -DEPTH_PATCH; dy <= DEPTH_PATCH; ++dy) {
                    for (int dx = -DEPTH_PATCH; dx <= DEPTH_PATCH; ++dx) {
                        int sx = std::max(0, std::min(cx + dx, depth_frame.get_width()  - 1));
                        int sy = std::max(0, std::min(cy + dy, depth_frame.get_height() - 1));
                        float d = depth_frame.get_distance(sx, sy);
                        if (d > 0.01f) depth_samples.push_back(d);
                    }
                }

                float dist_m = 0.0f;
                if (!depth_samples.empty()) {
                    std::nth_element(depth_samples.begin(),
                                     depth_samples.begin() + depth_samples.size() / 2,
                                     depth_samples.end());
                    dist_m = depth_samples[depth_samples.size() / 2];
                }

                if (dist_m > 0.01f) {
                    // 增加距离筛选：小于30厘米或大于6米的目标不处理
                    const float MIN_DISTANCE = 0.3f;  // 30厘米
                    const float MAX_DISTANCE = 6.0f;  // 6米
                    
                    if (dist_m < MIN_DISTANCE || dist_m > MAX_DISTANCE) {
                        RCLCPP_WARN(this->get_logger(), "目标距离 %.3f 米超出范围 (%.1f-%.1f 米)，跳过处理",
                                   dist_m, MIN_DISTANCE, MAX_DISTANCE);
                    } else {
                        float p3[3], pix[2] = {(float)cx, (float)cy};
                        rs2_deproject_pixel_to_point(p3, &intrin, pix, dist_m);

                        // Step 1: 视觉检测 → T_cam2target (相机坐标系下的目标位置)
                        Eigen::Vector4d P_cam(p3[0], p3[1], p3[2], 1.0);

                        // 【测试模式】总是打印当前检测到的相机坐标系坐标
                        RCLCPP_INFO(this->get_logger(), "检测到目标 (camera_frame): (%.3f, %.3f, %.3f) 米, 距离: %.3f 米",
                                    p3[0], p3[1], p3[2], dist_m);

                        // 【测试模式】发布 camera_link → target_camera 变换（无需手眼标定即可在 RViz 查看）
                        {
                            geometry_msgs::msg::TransformStamped cam_tf;
                            cam_tf.header.stamp = this->now();
                            cam_tf.header.frame_id = "camera_link";
                            cam_tf.child_frame_id = "target_camera";
                            cam_tf.transform.translation.x = p3[0];
                            cam_tf.transform.translation.y = p3[1];
                            cam_tf.transform.translation.z = p3[2];
                            cam_tf.transform.rotation.w = 1.0;
                            tf_broadcaster_->sendTransform(cam_tf);
                        }

                        // Step 2: 查 TF → T_base2flange (机械臂基座到末端法兰)
                        geometry_msgs::msg::TransformStamped tf_base2flange;
                        try {
                            tf_base2flange = tf_buffer_->lookupTransform("base_link", "tool0", tf2::TimePointZero);
                            Eigen::Affine3d affine_base2flange = tf2::transformToEigen(tf_base2flange.transform);
                            Eigen::Matrix4d T_base2flange = affine_base2flange.matrix();

                            // Step 3: 手眼变换 → T_base2target = T_base2flange × T_flange2cam × T_cam2target
                            Eigen::Vector4d P_base = T_base2flange * T_flange2cam_ * P_cam;

                            current_target_base = Eigen::Vector3d(P_base.x(), P_base.y(), P_base.z());
                            last_target_base_ = current_target_base;
                            target_detected_this_frame = true;
                        } catch (const tf2::TransformException& ex) {
                            RCLCPP_WARN(this->get_logger(), "TF查询失败(无机器人连接), 仅发布 camera_frame 坐标: %s", ex.what());
                        }
                    }
                }
            }

            if (!target_detected_this_frame) {
                RCLCPP_DEBUG(this->get_logger(), "Target not detected, skipping TF publish (using old data if available)");
            }

            // Step 4: 发布 TF → base_link → target_position
            if (last_target_base_.has_value()) {
                geometry_msgs::msg::TransformStamped target_tf;
                target_tf.header.stamp = this->now();
                target_tf.header.frame_id = "base_link";
                target_tf.child_frame_id = "target_position";

                // 只发布位置
                target_tf.transform.translation.x = last_target_base_->x();
                target_tf.transform.translation.y = last_target_base_->y();
                target_tf.transform.translation.z = last_target_base_->z();

                // 姿态给单位四元数（无旋转）
                target_tf.transform.rotation.w = 1.0;
                target_tf.transform.rotation.x = 0.0;
                target_tf.transform.rotation.y = 0.0;
                target_tf.transform.rotation.z = 0.0;

                tf_broadcaster_->sendTransform(target_tf);
            }

            // ============================================================
            // 可视化绘制
            // ============================================================
            // 绘制通过筛选的目标 (亮蓝色旋转矩形)
            for (int i = 0; i < (int)valid_sticks.size(); ++i) {
                cv::Point2f vertices[4];
                valid_sticks[i].points(vertices);
                for (int j = 0; j < 4; j++) {
                    cv::line(color_image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
                }

                // 绘制中心实心圆：蓝色 (BGR: 255, 0, 0)
                int radius = (i == rightmost_idx) ? 8 : 4;
                cv::circle(color_image, valid_sticks[i].center, radius, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);

                // 最右侧杆：读取中心点深度，反投影为相机坐标系 XYZ，显示在实心圆右侧
                if (i == rightmost_idx) {
                    int cx = std::max(0, std::min((int)valid_sticks[i].center.x, depth_frame.get_width()  - 1));
                    int cy = std::max(0, std::min((int)valid_sticks[i].center.y, depth_frame.get_height() - 1));

                    const int DEPTH_PATCH = 10;
                    std::vector<float> depth_samples;
                    for (int dy = -DEPTH_PATCH; dy <= DEPTH_PATCH; ++dy) {
                        for (int dx = -DEPTH_PATCH; dx <= DEPTH_PATCH; ++dx) {
                            int sx = std::max(0, std::min(cx + dx, depth_frame.get_width()  - 1));
                            int sy = std::max(0, std::min(cy + dy, depth_frame.get_height() - 1));
                            float d = depth_frame.get_distance(sx, sy);
                            if (d > 0.01f) depth_samples.push_back(d);
                        }
                    }

                    float dist_m = 0.0f;
                    if (!depth_samples.empty()) {
                        std::nth_element(depth_samples.begin(),
                                         depth_samples.begin() + depth_samples.size() / 2,
                                         depth_samples.end());
                        dist_m = depth_samples[depth_samples.size() / 2];
                    }

                    if (dist_m > 0.01f) {
                        // 增加距离筛选：小于30厘米或大于6米的目标不显示深度信息
                        const float MIN_DISTANCE = 0.3f;  // 30厘米
                        const float MAX_DISTANCE = 6.0f;  // 6米
                        
                        if (dist_m < MIN_DISTANCE || dist_m > MAX_DISTANCE) {
                            std::ostringstream oss_warning;
                            oss_warning << std::fixed << std::setprecision(2) << "Z: " << dist_m << "m (out of range)";
                            cv::putText(color_image, oss_warning.str(),
                                        cv::Point(cx + 12, cy),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                        cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                        } else {
                            float p3[3], pix[2] = {(float)cx, (float)cy};
                            rs2_deproject_pixel_to_point(p3, &intrin, pix, dist_m);

                            std::ostringstream oss_dist, oss_xyz;
                            oss_dist << std::fixed << std::setprecision(2) << "Z: " << dist_m << "m";
                            oss_xyz  << std::fixed << std::setprecision(2)
                                     << "(" << p3[0] << ", " << p3[1] << ", " << p3[2] << ")";

                            cv::putText(color_image, oss_dist.str(),
                                        cv::Point(cx + 12, cy),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                        cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
                            cv::putText(color_image, oss_xyz.str(),
                                        cv::Point(cx + 12, cy + 24),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                        cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

                            RCLCPP_INFO(this->get_logger(), "最右侧杆坐标: (%.3f, %.3f, %.3f) 米", p3[0], p3[1], p3[2]);
                        }
                    } else {
                        cv::putText(color_image, "depth invalid",
                                    cv::Point(cx + 12, cy),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                    }
                }
            }

            if (show_visualization_) {
                cv::imshow("Original BGR", color_image);
                cv::imshow("Red Segmentation Mask", mask_image);
                cv::waitKey(1);
            }
        } catch (const rs2::error& e) {
            RCLCPP_ERROR(this->get_logger(), "[VISION] RealSense error: %s  (func=%s, args=%s)",
                         e.what(), e.get_failed_function().c_str(), e.get_failed_args().c_str());
            break;
        } catch (const std::bad_array_new_length& e) {
            RCLCPP_ERROR(this->get_logger(), "[VISION] bad_array_new_length: %s — 可能分配了大小为0的数组", e.what());
            break;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[VISION] Exception: %s", e.what());
            break;
        }
        }
    }

    std::atomic<bool> is_running_;
    bool show_visualization_;
    std::thread vision_thread_;
    Eigen::Matrix4d T_flange2cam_;
    std::optional<Eigen::Vector3d> last_target_base_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionPositionPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}