#ifndef CONE_DETECTOR
#define CONE_DETECTOR
#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "yolo_v2_class.hpp"

struct detection_data_t {
    cv::Mat cap_frame;
    std::shared_ptr<image_t> det_image;
    std::vector<bbox_t> result_vec;
    cv::Mat draw_frame;
    bool exit_flag;
    bool new_detection;
    uint64_t frame_id;
    cv::Mat zed_cloud;
    std::queue<cv::Mat> track_optflow_queue;
    detection_data_t() : exit_flag(false), new_detection(false) {}
};

template<typename T>
class send_one_replaceable_object_t {
    const bool sync;
    std::atomic<T *> a_ptr;
public:

    void send(T const& _obj) {
        T *new_ptr = new T;
        *new_ptr = _obj;
        if (sync) {
            while (a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        std::unique_ptr<T> old_ptr(a_ptr.exchange(new_ptr));
    }

    T receive() {
        std::unique_ptr<T> ptr;
        do {
            while(!a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
            ptr.reset(a_ptr.exchange(NULL));
        } while (!ptr);
        T obj = *ptr;
        return obj;
    }

    bool is_object_present() {
        return (a_ptr.load() != NULL);
    }

    send_one_replaceable_object_t(bool _sync) : sync(_sync), a_ptr(NULL)
    {}
};

int detectCones(send_one_replaceable_object_t<detection_data_t> &detection_data,
  std::string net_names = "/usr/lib/formula.names",
  std::string net_cfg = "/usr/lib/formula_new.cfg",
  std::string net_weights = "/usr/lib/formula_new_final.weights");
void show_console_result(std::vector<bbox_t> const result_vec, int frame_id = -1);

#endif // CONE_DETECTOR
