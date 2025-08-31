#include "BaseInfer.hpp"
#include "NvidiaInterface.hpp"
#include "yolos.hpp"

namespace yolo {

using namespace std;
using namespace tdt_radar;
static const int point_num = 4;

const int NUM_BOX_ELEMENT = 15;
const int MAX_IMAGE_BOXES = 1024;

inline int upbound(int n, int align = 32)
{
    return (n + align - 1) / align * align;
}

static __host__ __device__ void
affine_project(float* matrix, float x, float y, float* ox, float* oy)
{
    *ox = matrix[0] * x + matrix[1] * y + matrix[2];
    *oy = matrix[3] * x + matrix[4] * y + matrix[5];
}

static __global__ void
decode_kernel_common(float* predict, int num_bboxes, int num_classes,
                     int output_cdim, float confidence_threshold,
                     float* invert_affine_matrix, float* parray,
                     int MAX_IMAGE_BOXES)
{
    int position = blockDim.x * blockIdx.x + threadIdx.x;
    if (position >= num_bboxes)
        return;

    float* pitem = predict + output_cdim * position;

    float objectness = pitem[4];
    if (objectness < confidence_threshold)
        return;

    float* class_confidence = pitem + 5;
    float  confidence = *class_confidence++;
    int    label = 0;
    for (int i = 1; i < num_classes; ++i, ++class_confidence) {
        if (*class_confidence > confidence) {
            confidence = *class_confidence;
            label = i;
        }
    }

    confidence *= objectness;
    if (confidence < confidence_threshold)
        return;

    int index = atomicAdd(parray, 1);
    if (index >= MAX_IMAGE_BOXES)
        return;

    float cx = *pitem++;
    float cy = *pitem++;
    float width = *pitem++;
    float height = *pitem++;

    float left = cx - width * 0.5f;
    float top = cy - height * 0.5f;
    float right = cx + width * 0.5f;
    float bottom = cy + height * 0.5f;
    affine_project(invert_affine_matrix, left, top, &left, &top);
    affine_project(invert_affine_matrix, right, bottom, &right, &bottom);

    float* pout_item = parray + 1 + index * NUM_BOX_ELEMENT;
    *pout_item++ = left;
    *pout_item++ = top;
    *pout_item++ = right;
    *pout_item++ = bottom;
    *pout_item++ = confidence;
    *pout_item++ = label;
    *pout_item++ = 1;  // 1 = keep, 0 = ignore
}

static __global__ void
decode_kernel_v5_face(float* predict, int num_bboxes, int num_classes,
                      int output_cdim, float confidence_threshold,
                      float* invert_affine_matrix, float* parray,
                      int MAX_IMAGE_BOXES)
{
    int position = blockDim.x * blockIdx.x + threadIdx.x;
    if (position >= num_bboxes)
        return;

    float* pitem = predict + output_cdim * position;

    float objectness = pitem[4];
    if (objectness < confidence_threshold)
        return;

    float* class_confidence = pitem + 5 + 2 * point_num;
    float  confidence = *class_confidence++;
    int    label = 0;
    for (int i = 1; i < num_classes; ++i, ++class_confidence) {
        if (*class_confidence > confidence) {
            confidence = *class_confidence;
            label = i;
        }
    }

    confidence *= objectness;
    if (confidence < confidence_threshold)
        return;

    int index = atomicAdd(parray, 1);
    if (index >= MAX_IMAGE_BOXES)
        return;

    float cx = *pitem++;
    float cy = *pitem++;
    float width = *pitem++;
    float height = *pitem++;
    objectness = *pitem++;
    float left = cx - width * 0.5f;
    float top = cy - height * 0.5f;
    float right = cx + width * 0.5f;
    float bottom = cy + height * 0.5f;

    float landmark_array[point_num * 2];

    for (int i = 0; i < point_num; i++) {
        landmark_array[2 * i] = *pitem++;
        landmark_array[2 * i + 1] = *pitem++;
    }

    affine_project(invert_affine_matrix, left, top, &left, &top);
    affine_project(invert_affine_matrix, right, bottom, &right, &bottom);
    for (int i = 0; i < point_num; i++) {
        affine_project(invert_affine_matrix, landmark_array[2 * i],
                       landmark_array[2 * i + 1], &landmark_array[2 * i],
                       &landmark_array[2 * i + 1]);
    }
    float* pout_item = parray + 1 + index * NUM_BOX_ELEMENT;
    *pout_item++ = left;
    *pout_item++ = top;
    *pout_item++ = right;
    *pout_item++ = bottom;
    *pout_item++ = confidence;
    *pout_item++ = label;
    *pout_item++ = 1;  // 1 = keep, 0 = ignore
    for (int i = 0; i < point_num; i++) {
        *pout_item++ = landmark_array[2 * i];
        *pout_item++ = landmark_array[2 * i + 1];
    }
}

static __global__ void decode_kernel_v8(float* predict, int num_bboxes,
                                        int num_classes, int output_cdim,
                                        float  confidence_threshold,
                                        float* invert_affine_matrix,
                                        float* parray, int MAX_IMAGE_BOXES)
{
    int position = blockDim.x * blockIdx.x + threadIdx.x;
    if (position >= num_bboxes)
        return;

    float* pitem = predict + output_cdim * position;
    float* class_confidence = pitem + 4;
    float  confidence = *class_confidence++;
    int    label = 0;
    for (int i = 1; i < num_classes; ++i, ++class_confidence) {
        if (*class_confidence > confidence) {
            confidence = *class_confidence;
            label = i;
        }
    }
    if (confidence < confidence_threshold)
        return;

    int index = atomicAdd(parray, 1);
    if (index >= MAX_IMAGE_BOXES)
        return;

    float cx = *pitem++;
    float cy = *pitem++;
    float width = *pitem++;
    float height = *pitem++;
    float left = cx - width * 0.5f;
    float top = cy - height * 0.5f;
    float right = cx + width * 0.5f;
    float bottom = cy + height * 0.5f;
    affine_project(invert_affine_matrix, left, top, &left, &top);
    affine_project(invert_affine_matrix, right, bottom, &right, &bottom);

    float* pout_item = parray + 1 + index * NUM_BOX_ELEMENT;
    *pout_item++ = left;
    *pout_item++ = top;
    *pout_item++ = right;
    *pout_item++ = bottom;
    *pout_item++ = confidence;
    *pout_item++ = label;
    *pout_item++ = 1;  // 1 = keep, 0 = ignore
    *pout_item++ = position;
}

static __device__ float box_iou(float aleft, float atop, float aright,
                                float abottom, float bleft, float btop,
                                float bright, float bbottom)
{
    float cleft = max(aleft, bleft);
    float ctop = max(atop, btop);
    float cright = min(aright, bright);
    float cbottom = min(abottom, bbottom);

    float c_area = max(cright - cleft, 0.0f) * max(cbottom - ctop, 0.0f);
    if (c_area == 0.0f)
        return 0.0f;

    float a_area = max(0.0f, aright - aleft) * max(0.0f, abottom - atop);
    float b_area = max(0.0f, bright - bleft) * max(0.0f, bbottom - btop);
    return c_area / (a_area + b_area - c_area);
}

static __global__ void fast_nms_kernel(float* bboxes, int MAX_IMAGE_BOXES,
                                       float threshold)
{
    int position = (blockDim.x * blockIdx.x + threadIdx.x);
    int count = min((int)*bboxes, MAX_IMAGE_BOXES);
    if (position >= count)
        return;

    float* pcurrent = bboxes + 1 + position * NUM_BOX_ELEMENT;
    for (int i = 0; i < count; ++i) {
        float* pitem = bboxes + 1 + i * NUM_BOX_ELEMENT;
        if (i == position || pcurrent[5] != pitem[5])
            continue;

        if (pitem[4] >= pcurrent[4]) {
            if (pitem[4] == pcurrent[4] && i < position)
                continue;

            float iou =
                box_iou(pcurrent[0], pcurrent[1], pcurrent[2], pcurrent[3],
                        pitem[0], pitem[1], pitem[2], pitem[3]);

            if (iou > threshold) {
                pcurrent[6] = 0;  // 1=keep, 0=ignore
                return;
            }
        }
    }
}

static dim3 grid_dims(int numJobs)
{
    int numBlockThreads =
        numJobs < GPU_BLOCK_THREADS ? numJobs : GPU_BLOCK_THREADS;
    return dim3(((numJobs + numBlockThreads - 1) / (float)numBlockThreads));
}

static dim3 block_dims(int numJobs)
{
    return numJobs < GPU_BLOCK_THREADS ? numJobs : GPU_BLOCK_THREADS;
}

static void decode_kernel_invoker(float* predict, int num_bboxes,
                                  int num_classes, int output_cdim,
                                  float  confidence_threshold,
                                  float  nms_threshold,
                                  float* invert_affine_matrix,
                                  float* parray, int MAX_IMAGE_BOXES,
                                  Type type, cudaStream_t stream)
{
    auto grid = grid_dims(num_bboxes);
    auto block = block_dims(num_bboxes);

    if (type == Type::V8 || type == Type::V8Seg) {
        checkKernel(decode_kernel_v8<<<grid, block, 0, stream>>>(
            predict, num_bboxes, num_classes, output_cdim,
            confidence_threshold, invert_affine_matrix, parray,
            MAX_IMAGE_BOXES));
    } else if (type == Type::V5Face) {
        checkKernel(decode_kernel_v5_face<<<grid, block, 0, stream>>>(
            predict, num_bboxes, num_classes, output_cdim,
            confidence_threshold, invert_affine_matrix, parray,
            MAX_IMAGE_BOXES));
    } else {
        checkKernel(decode_kernel_common<<<grid, block, 0, stream>>>(
            predict, num_bboxes, num_classes, output_cdim,
            confidence_threshold, invert_affine_matrix, parray,
            MAX_IMAGE_BOXES));
    }

    grid = grid_dims(MAX_IMAGE_BOXES);
    block = block_dims(MAX_IMAGE_BOXES);
    checkKernel(fast_nms_kernel<<<grid, block, 0, stream>>>(
        parray, MAX_IMAGE_BOXES, nms_threshold));
}

static __global__ void
decode_single_mask_kernel(int left, int top, float* mask_weights,
                          float* mask_predict, int mask_width,
                          int mask_height, unsigned char* mask_out,
                          int mask_dim, int out_width, int out_height)
{
    int dx = blockDim.x * blockIdx.x + threadIdx.x;
    int dy = blockDim.y * blockIdx.y + threadIdx.y;
    if (dx >= out_width || dy >= out_height)
        return;

    int sx = left + dx;
    int sy = top + dy;
    if (sx < 0 || sx >= mask_width || sy < 0 || sy >= mask_height) {
        mask_out[dy * out_width + dx] = 0;
        return;
    }

    float cumprod = 0;
    for (int ic = 0; ic < mask_dim; ++ic) {
        float cval =
            mask_predict[(ic * mask_height + sy) * mask_width + sx];
        float wval = mask_weights[ic];
        cumprod += cval * wval;
    }

    float alpha = 1.0f / (1.0f + exp(-cumprod));
    mask_out[dy * out_width + dx] = alpha * 255;
}

static void decode_single_mask(float left, float top, float* mask_weights,
                               float* mask_predict, int mask_width,
                               int mask_height, unsigned char* mask_out,
                               int mask_dim, int out_width, int out_height,
                               cudaStream_t stream)
{
    // mask_weights is mask_dim(32 element) gpu pointer
    dim3 grid((out_width + 31) / 32, (out_height + 31) / 32);
    dim3 block(32, 32);

    checkKernel(decode_single_mask_kernel<<<grid, block, 0, stream>>>(
        left, top, mask_weights, mask_predict, mask_width, mask_height,
        mask_out, mask_dim, out_width, out_height));
}

const char* type_name(Type type)
{
    switch (type) {
    case Type::V5:
        return "YoloV5";
    case Type::V3:
        return "YoloV3";
    case Type::V7:
        return "YoloV7";
    case Type::X:
        return "YoloX";
    case Type::V8:
        return "YoloV8";
    case Type::V5Face:
        return "YoloV5Face";
    default:
        return "Unknow";
    }
}

struct AffineMatrix {
    float i2d[6];
    float d2i[6];

    void compute(const std::tuple<int, int>& from,
                 const std::tuple<int, int>& to)
    {
        float scale_x = get<0>(to) / (float)get<0>(from);
        float scale_y = get<1>(to) / (float)get<1>(from);
        float scale = std::min(scale_x, scale_y);
        i2d[0] = scale;
        i2d[1] = 0;
        i2d[2] = -scale * get<0>(from) * 0.5 + get<0>(to) * 0.5 +
                 scale * 0.5 - 0.5;
        i2d[3] = 0;
        i2d[4] = scale;
        i2d[5] = -scale * get<1>(from) * 0.5 + get<1>(to) * 0.5 +
                 scale * 0.5 - 0.5;

        double D = i2d[0] * i2d[4] - i2d[1] * i2d[3];
        D = D != 0. ? double(1.) / D : double(0.);
        double A11 = i2d[4] * D, A22 = i2d[0] * D, A12 = -i2d[1] * D,
               A21 = -i2d[3] * D;
        double b1 = -A11 * i2d[2] - A12 * i2d[5];
        double b2 = -A21 * i2d[2] - A22 * i2d[5];

        d2i[0] = A11;
        d2i[1] = A12;
        d2i[2] = b1;
        d2i[3] = A21;
        d2i[4] = A22;
        d2i[5] = b2;
    }
};

InstanceSegmentMap::InstanceSegmentMap(int width, int height)
{
    this->width = width;
    this->height = height;
    checkRuntime(cudaMallocHost(&this->data, width * height));
}

InstanceSegmentMap::~InstanceSegmentMap()
{
    if (this->data) {
        checkRuntime(cudaFreeHost(this->data));
        this->data = nullptr;
    }
    this->width = 0;
    this->height = 0;
}

class InferImpl : public Infer<BoxArray> {
public:
    shared_ptr<trt::Infer>                         trt_;
    string                                         engine_file_;
    Type                                           type_;
    float                                          confidence_threshold_;
    float                                          nms_threshold_;
    vector<shared_ptr<trt::Memory<unsigned char>>> preprocess_buffers_;
    trt::Memory<float> input_buffer_, bbox_predict_, output_boxarray_;
    trt::Memory<float> segment_predict_;
    int                network_input_width_, network_input_height_;
    Norm               normalize_;
    vector<int>        bbox_head_dims_;
    vector<int>        segment_head_dims_;
    int                num_classes_ = 0;
    bool               has_segment_ = false;
    bool               has_keyPoint = false;
    bool               isdynamic_model_ = false;
    vector<shared_ptr<trt::Memory<unsigned char>>> box_segment_cache_;

    virtual ~InferImpl() = default;

    void adjust_memory(int batch_size)
    {
        size_t input_numel =
            network_input_width_ * network_input_height_ * 3;
        input_buffer_.gpu(batch_size * input_numel);
        bbox_predict_.gpu(batch_size * bbox_head_dims_[1] *
                          bbox_head_dims_[2]);
        output_boxarray_.gpu(batch_size *
                             (32 + MAX_IMAGE_BOXES * NUM_BOX_ELEMENT));
        output_boxarray_.cpu(batch_size *
                             (32 + MAX_IMAGE_BOXES * NUM_BOX_ELEMENT));

        if (has_segment_)
            segment_predict_.gpu(batch_size * segment_head_dims_[1] *
                                 segment_head_dims_[2] *
                                 segment_head_dims_[3]);

        if ((int)preprocess_buffers_.size() < batch_size) {
            for (int i = preprocess_buffers_.size(); i < batch_size; ++i)
                preprocess_buffers_.push_back(
                    make_shared<trt::Memory<unsigned char>>());
        }
    }

    void
    preprocess(int ibatch, const Image& image,
               shared_ptr<trt::Memory<unsigned char>> preprocess_buffer,
               AffineMatrix& affine, void* stream = nullptr)
    {
        affine.compute(
            make_tuple(image.width, image.height),
            make_tuple(network_input_width_, network_input_height_));

        size_t input_numel =
            network_input_width_ * network_input_height_ * 3;
        float*   input_device = input_buffer_.gpu() + ibatch * input_numel;
        size_t   size_image = image.width * image.height * 3;
        size_t   size_matrix = upbound(sizeof(affine.d2i), 32);
        uint8_t* gpu_workspace =
            preprocess_buffer->gpu(size_matrix + size_image);
        float*   affine_matrix_device = (float*)gpu_workspace;
        uint8_t* image_device = gpu_workspace + size_matrix;

        uint8_t* cpu_workspace =
            preprocess_buffer->cpu(size_matrix + size_image);
        float*   affine_matrix_host = (float*)cpu_workspace;
        uint8_t* image_host = cpu_workspace + size_matrix;

        cudaStream_t stream_ = (cudaStream_t)stream;
        std::chrono ::high_resolution_clock::time_point a1 =
            std::chrono::high_resolution_clock::now();
        memcpy(image_host, image.bgrptr, size_image);
        memcpy(affine_matrix_host, affine.d2i, sizeof(affine.d2i));
        std::chrono ::high_resolution_clock::time_point a2 =
            std::chrono::high_resolution_clock::now();
        auto time_used2 =
            std::chrono::duration_cast<std::chrono::duration<double>>(a2 -
                                                                      a1);
        checkRuntime(cudaMemcpyAsync(image_device, image_host, size_image,
                                     cudaMemcpyHostToDevice, stream_));
        checkRuntime(cudaMemcpyAsync(affine_matrix_device,
                                     affine_matrix_host, sizeof(affine.d2i),
                                     cudaMemcpyHostToDevice, stream_));
        warp_affine_bilinear_and_normalize_plane(
            image_device, image.width * 3, image.width, image.height,
            input_device, network_input_width_, network_input_height_,
            affine_matrix_device, 114, normalize_, stream_);
    }

    bool load(const string& engine_file, Type type,
              float confidence_threshold, float nms_threshold)
    {
        trt_ = trt::load(engine_file);
        if (trt_ == nullptr)
            return false;

        trt_->print();

        this->type_ = type;
        this->confidence_threshold_ = confidence_threshold;
        this->nms_threshold_ = nms_threshold;

        auto input_dim = trt_->static_dims(0);
        bbox_head_dims_ = trt_->static_dims(1);

        has_segment_ = type == Type::V8Seg;
        has_keyPoint = type == Type::V5Face;
        if (has_segment_) {
            bbox_head_dims_ = trt_->static_dims(2);
            segment_head_dims_ = trt_->static_dims(1);
        }
        network_input_width_ = input_dim[3];
        network_input_height_ = input_dim[2];
        isdynamic_model_ = trt_->has_dynamic_dim();

        if (type == Type::V5 || type == Type::V3 || type == Type::V7) {
            normalize_ =
                Norm::alpha_beta(1 / 255.0f, 0.0f, ChannelType::SwapRB);
            num_classes_ = bbox_head_dims_[2] - 5;
        } else if (type == Type::V8) {
            normalize_ =
                Norm::alpha_beta(1 / 255.0f, 0.0f, ChannelType::SwapRB);
            num_classes_ = bbox_head_dims_[2] - 4;
        } else if (type == Type::V8Seg) {
            normalize_ =
                Norm::alpha_beta(1 / 255.0f, 0.0f, ChannelType::SwapRB);
            num_classes_ = bbox_head_dims_[2] - 4 - segment_head_dims_[1];
        } else if (type == Type::X) {
            normalize_ = Norm::None();
            num_classes_ = bbox_head_dims_[2] - 5;
        } else if (type == Type::V5Face) {
            normalize_ =
                Norm::alpha_beta(1 / 255.0f, 0.0f, ChannelType::SwapRB);
            num_classes_ = bbox_head_dims_[2] - 5 - 2 * point_num;
        } else {
            INFO("Unsupport type %d", type);
        }
        return true;
    }

    virtual BoxArray forward(const Image& image,
                             void*        stream = nullptr) override
    {
        auto output = forwards({image}, stream);
        if (output.empty())
            return {};
        return output[0];
    }

    virtual vector<BoxArray> forwards(const vector<Image>& images,
                                      void* stream = nullptr) override
    {
        int num_image = images.size();
        if (num_image == 0)
            return {};

        auto input_dims = trt_->static_dims(0);
        int  infer_batch_size = input_dims[0];
        if (infer_batch_size != num_image) {
            if (isdynamic_model_) {
                infer_batch_size = num_image;
                input_dims[0] = num_image;
                if (!trt_->set_run_dims(0, input_dims))
                    return {};
            } else {
                if (infer_batch_size < num_image) {
                    INFO("When using static shape model, number of "
                         "images[%d] must be "
                         "less than or equal to the maximum batch[%d].",
                         num_image, infer_batch_size);
                    return {};
                }
            }
        }

        std::chrono ::high_resolution_clock::time_point a1 =
            std::chrono::high_resolution_clock::now();
        adjust_memory(infer_batch_size);
        std::chrono ::high_resolution_clock::time_point a2 =
            std::chrono::high_resolution_clock::now();
        auto time_used2 =
            std::chrono::duration_cast<std::chrono::duration<double>>(a2 -
                                                                      a1);

        vector<AffineMatrix> affine_matrixs(num_image);
        std::chrono ::high_resolution_clock::time_point a3 =
            std::chrono::high_resolution_clock::now();
        cudaStream_t stream_ = (cudaStream_t)stream;
        for (int i = 0; i < num_image; ++i)
            preprocess(i, images[i], preprocess_buffers_[i],
                       affine_matrixs[i], stream);

        float*        bbox_output_device = bbox_predict_.gpu();
        vector<void*> bindings{input_buffer_.gpu(), bbox_output_device};

        if (has_segment_) {
            bindings = {input_buffer_.gpu(), segment_predict_.gpu(),
                        bbox_output_device};
        }
        std::chrono ::high_resolution_clock::time_point a3d1 =
            std::chrono::high_resolution_clock::now();
        auto time_used3 =
            std::chrono::duration_cast<std::chrono::duration<double>>(a3d1 -
                                                                      a3);
        if (!trt_->forward(bindings, stream)) {
            INFO("Failed to tensorRT forward.");
            return {};
        }
        std::chrono ::high_resolution_clock::time_point a3d2 =
            std::chrono::high_resolution_clock::now();
        auto time_used3d2 =
            std::chrono::duration_cast<std::chrono::duration<double>>(a3d2 -
                                                                      a3d1);

        for (int ib = 0; ib < num_image; ++ib) {
            float* boxarray_device =
                output_boxarray_.gpu() +
                ib * (32 + MAX_IMAGE_BOXES * NUM_BOX_ELEMENT);
            float* affine_matrix_device =
                (float*)preprocess_buffers_[ib]->gpu();
            float* image_based_bbox_output =
                bbox_output_device +
                ib * (bbox_head_dims_[1] * bbox_head_dims_[2]);

            checkRuntime(
                cudaMemsetAsync(boxarray_device, 0, sizeof(int), stream_));
            decode_kernel_invoker(
                image_based_bbox_output, bbox_head_dims_[1], num_classes_,
                bbox_head_dims_[2], confidence_threshold_, nms_threshold_,
                affine_matrix_device, boxarray_device, MAX_IMAGE_BOXES,
                type_, stream_);
        }
        std::chrono ::high_resolution_clock::time_point a3d3 =
            std::chrono::high_resolution_clock::now();
        auto time_used3d3 =
            std::chrono::duration_cast<std::chrono::duration<double>>(a3d3 -
                                                                      a3d2);
        std::chrono ::high_resolution_clock::time_point a3d4 =
            std::chrono::high_resolution_clock::now();
        checkRuntime(cudaMemcpyAsync(
            output_boxarray_.cpu(), output_boxarray_.gpu(),
            output_boxarray_.gpu_bytes(), cudaMemcpyDeviceToHost, stream_));
        checkRuntime(cudaStreamSynchronize(stream_));
        std::chrono ::high_resolution_clock::time_point a4 =
            std::chrono::high_resolution_clock::now();
        auto time_used3d4 =
            std::chrono::duration_cast<std::chrono::duration<double>>(a4 -
                                                                      a3d4);
        auto time_used4 =
            std::chrono::duration_cast<std::chrono::duration<double>>(a4 -
                                                                      a3);
        INFO("forward and decode_kernel_invoker time: %f",
             time_used4.count() * 1000);
        std::chrono ::high_resolution_clock::time_point a5 =
            std::chrono::high_resolution_clock::now();
        vector<BoxArray> arrout(num_image);
        int              imemory = 0;
        for (int ib = 0; ib < num_image; ++ib) {
            float* parray = output_boxarray_.cpu() +
                            ib * (32 + MAX_IMAGE_BOXES * NUM_BOX_ELEMENT);
            int       count = min(MAX_IMAGE_BOXES, (int)*parray);
            BoxArray& output = arrout[ib];
            output.reserve(count);
            for (int i = 0; i < count; ++i) {
                float*             pbox = parray + 1 + i * NUM_BOX_ELEMENT;
                int                a1 = pbox[0];
                int                a2 = pbox[1];
                int                a3 = pbox[2];
                int                a4 = pbox[3];
                int                a5 = pbox[4];
                int                label = pbox[5];
                int                keepflag = pbox[6];
                std::vector<float> points;
                for (int i = 1; i < 9; i++) {
                    points.push_back(pbox[6 + i]);
                }
                if (keepflag == 1) {
                    Box result_object_box(pbox[0], pbox[1], pbox[2],
                                          pbox[3], pbox[4], label, points);

                    if (has_segment_) {
                        int    row_index = pbox[7];
                        int    mask_dim = segment_head_dims_[1];
                        float* mask_weights =
                            bbox_output_device +
                            (ib * bbox_head_dims_[1] + row_index) *
                                bbox_head_dims_[2] +
                            num_classes_ + 4;

                        float* mask_head_predict = segment_predict_.gpu();
                        float  left, top, right, bottom;
                        float* i2d = affine_matrixs[ib].i2d;
                        affine_project(i2d, pbox[0], pbox[1], &left, &top);
                        affine_project(i2d, pbox[2], pbox[3], &right,
                                       &bottom);

                        float box_width = right - left;
                        float box_height = bottom - top;

                        float scale_to_predict_x =
                            segment_head_dims_[3] /
                            (float)network_input_width_;
                        float scale_to_predict_y =
                            segment_head_dims_[2] /
                            (float)network_input_height_;
                        int mask_out_width =
                            box_width * scale_to_predict_x + 0.5f;
                        int mask_out_height =
                            box_height * scale_to_predict_y + 0.5f;

                        if (mask_out_width > 0 && mask_out_height > 0) {
                            if (imemory >= (int)box_segment_cache_.size()) {
                                box_segment_cache_.push_back(
                                    std::make_shared<
                                        trt::Memory<unsigned char>>());
                            }

                            int bytes_of_mask_out =
                                mask_out_width * mask_out_height;
                            auto box_segment_output_memory =
                                box_segment_cache_[imemory];
                            result_object_box.seg =
                                make_shared<InstanceSegmentMap>(
                                    mask_out_width, mask_out_height);

                            unsigned char* mask_out_device =
                                box_segment_output_memory->gpu(
                                    bytes_of_mask_out);
                            unsigned char* mask_out_host =
                                result_object_box.seg->data;
                            decode_single_mask(
                                left * scale_to_predict_x,
                                top * scale_to_predict_y, mask_weights,
                                mask_head_predict +
                                    ib * segment_head_dims_[1] *
                                        segment_head_dims_[2] *
                                        segment_head_dims_[3],
                                segment_head_dims_[3],
                                segment_head_dims_[2], mask_out_device,
                                mask_dim, mask_out_width, mask_out_height,
                                stream_);
                            checkRuntime(cudaMemcpyAsync(
                                mask_out_host, mask_out_device,
                                box_segment_output_memory->gpu_bytes(),
                                cudaMemcpyDeviceToHost, stream_));
                        }
                    }
                    output.emplace_back(result_object_box);
                }
            }
        }
        std::chrono ::high_resolution_clock::time_point a6 =
            std::chrono::high_resolution_clock::now();
        auto time_used6 =
            std::chrono::duration_cast<std::chrono::duration<double>>(a6 -
                                                                      a5);
        if (has_segment_)
            checkRuntime(cudaStreamSynchronize(stream_));

        return arrout;
    }
};

Infer<BoxArray>* loadraw(const std::string& engine_file, Type type,
                         float confidence_threshold, float nms_threshold)
{
    InferImpl* impl = new InferImpl();
    if (!impl->load(engine_file, type, confidence_threshold,
                    nms_threshold)) {
        delete impl;
        impl = nullptr;
    }
    return impl;
}

shared_ptr<Infer<BoxArray>> load(const string& engine_file, Type type,
                                 float confidence_threshold,
                                 float nms_threshold)
{
    return std::shared_ptr<InferImpl>((InferImpl*)loadraw(
        engine_file, type, confidence_threshold, nms_threshold));
}

std::tuple<uint8_t, uint8_t, uint8_t> hsv2bgr(float h, float s, float v)
{
    const int   h_i = static_cast<int>(h * 6);
    const float f = h * 6 - h_i;
    const float p = v * (1 - s);
    const float q = v * (1 - f * s);
    const float t = v * (1 - (1 - f) * s);
    float       r, g, b;
    switch (h_i) {
    case 0:
        r = v, g = t, b = p;
        break;
    case 1:
        r = q, g = v, b = p;
        break;
    case 2:
        r = p, g = v, b = t;
        break;
    case 3:
        r = p, g = q, b = v;
        break;
    case 4:
        r = t, g = p, b = v;
        break;
    case 5:
        r = v, g = p, b = q;
        break;
    default:
        r = 1, g = 1, b = 1;
        break;
    }
    return make_tuple(static_cast<uint8_t>(b * 255),
                      static_cast<uint8_t>(g * 255),
                      static_cast<uint8_t>(r * 255));
}

std::tuple<uint8_t, uint8_t, uint8_t> random_color(int id)
{
    float h_plane = ((((unsigned int)id << 2) ^ 0x937151) % 100) / 100.0f;
    float s_plane = ((((unsigned int)id << 3) ^ 0x315793) % 100) / 100.0f;
    return hsv2bgr(h_plane, s_plane, 1);
}

};  // namespace yolo