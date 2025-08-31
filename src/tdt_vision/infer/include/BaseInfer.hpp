#ifndef __BASEINFER_HPP__
#define __BASEINFER_HPP__

#include <tuple>
#include <vector>

namespace tdt_radar {

#define GPU_BLOCK_THREADS 512

struct Image {
    const void* bgrptr = nullptr;
    int         width = 0, height = 0;

    Image() = default;
    Image(const void* bgrptr, int width, int height)
        : bgrptr(bgrptr), width(width), height(height)
    {
    }
};

enum class NormType : int { None = 0, MeanStd = 1, AlphaBeta = 2 };

enum class ChannelType : int { None = 0, SwapRB = 1 };

template <typename T>
class Infer {
public:
    virtual T forward(const Image& image, void* stream = nullptr) = 0;
    virtual std::vector<T> forwards(const std::vector<Image>& images,
                                    void* stream = nullptr) = 0;
};

struct Norm {
    float       mean[3];
    float       std[3];
    float       alpha, beta;
    NormType    type = NormType::None;
    ChannelType channel_type = ChannelType::None;

    static Norm mean_std(const float mean[3], const float std[3],
                         float       alpha = 1 / 255.0f,
                         ChannelType channel_type = ChannelType::None);

    static Norm alpha_beta(float alpha, float beta = 0,
                           ChannelType channel_type = ChannelType::None);

    static Norm None();
};

struct AffineMatrix {
    float i2d[6];
    float d2i[6];

    void compute(const std::tuple<int, int>& from,
                 const std::tuple<int, int>& to);
};

}  // namespace tdt_radar

#endif  //__BASEINFER_HPP__
