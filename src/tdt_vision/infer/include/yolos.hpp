#ifndef __YOLO_HPP__
#define __YOLO_HPP__

#include <NvInfer.h>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include "BaseInfer.hpp"
#include "InferTool.hpp"
#include "logging.h"

using namespace nvinfer1;

static tdt_yolov5_predictor::Logger gLogger;

namespace yolo {

enum class Type : int {
    V5 = 0,
    X = 1,
    V3 = 2,
    V7 = 3,
    V8 = 5,
    V8Seg = 6,
    V5Face = 7
};

struct InstanceSegmentMap {
    int            width = 0, height = 0;
    unsigned char* data = nullptr;

    InstanceSegmentMap(int width, int height);
    virtual ~InstanceSegmentMap();
};

struct Box {
    float left, top, right, bottom, confidence;
    int   class_label;
    std::shared_ptr<InstanceSegmentMap> seg;
    std::vector<float>                  points;
    Box() = default;
    Box(float left, float top, float right, float bottom, float confidence,
        int class_label)
        : left(left),
          top(top),
          right(right),
          bottom(bottom),
          confidence(confidence),
          class_label(class_label)
    {
    }
    Box(float left, float top, float right, float bottom, float confidence,
        int class_label, std::vector<float> points)
        : left(left),
          top(top),
          right(right),
          bottom(bottom),
          confidence(confidence),
          class_label(class_label),
          points(points)
    {
    }
};

typedef std::vector<Box> BoxArray;

std::shared_ptr<tdt_radar::Infer<BoxArray>>
load(const std::string& engine_file, Type type,
     float confidence_threshold = 0.45f, float nms_threshold = 0.6f);

const char*                           type_name(Type type);
std::tuple<uint8_t, uint8_t, uint8_t> hsv2bgr(float h, float s, float v);
std::tuple<uint8_t, uint8_t, uint8_t> random_color(int id);
};  // namespace yolo

#endif  // __YOLO_HPP__