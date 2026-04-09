#include "mlx90640frame.h"

#include <cstring>

#ifndef THERMAL_CAMERA_FRAME_PROCESS_VARIANT
#define THERMAL_CAMERA_FRAME_PROCESS_VARIANT 0
#endif

#define THERMAL_CAMERA_FRAME_PROCESS_SERIAL 0
#define THERMAL_CAMERA_FRAME_PROCESS_OPENMP 1
#define THERMAL_CAMERA_FRAME_PROCESS_SYCL 2

#if THERMAL_CAMERA_FRAME_PROCESS_VARIANT == THERMAL_CAMERA_FRAME_PROCESS_SYCL
#include <sycl/sycl.hpp>
#endif

namespace {

constexpr int kSubframeCount=2;
constexpr int kSubframeWords=834;
constexpr float kTaShift=8.f; //Default shift for MLX90640 in open air

void processSubframeSerial(const uint16_t *subframe, const paramsMLX90640 *params,
    float emissivity, short *temperature)
{
    const float vdd=mlx90640_detail::getVdd(subframe,params);
    const float ta=mlx90640_detail::getTa(subframe,params,vdd);
    const float tr=ta-kTaShift;
    const auto context=mlx90640_detail::buildToContext(subframe,params,emissivity,vdd,ta,tr);
    mlx90640_detail::calculateToShort(subframe,params,context,temperature);
}

#if THERMAL_CAMERA_FRAME_PROCESS_VARIANT == THERMAL_CAMERA_FRAME_PROCESS_SYCL
sycl::queue& processQueue()
{
    static sycl::queue queue;
    return queue;
}

void processFrameSycl(const MLX90640RawFrame& input, MLX90640Frame *output,
    const paramsMLX90640& params, float emissivity)
{
    auto& queue=processQueue();
    auto *subframes=sycl::malloc_shared<uint16_t>(kSubframeCount * kSubframeWords, queue);
    auto *syclParams=sycl::malloc_shared<paramsMLX90640>(1, queue);
    auto *contexts=sycl::malloc_shared<mlx90640_detail::ToCalculationContext>(kSubframeCount, queue);
    auto *temperature=sycl::malloc_shared<short>(mlx90640_detail::kPixelCount, queue);

    if(subframes==nullptr || syclParams==nullptr || contexts==nullptr || temperature==nullptr)
    {
        if(subframes) sycl::free(subframes, queue);
        if(syclParams) sycl::free(syclParams, queue);
        if(contexts) sycl::free(contexts, queue);
        if(temperature) sycl::free(temperature, queue);
        for(int i=0;i<kSubframeCount;i++) processSubframeSerial(input.subframe[i],&params,emissivity,output->temperature);
        return;
    }

    std::memcpy(subframes,input.subframe,sizeof(input.subframe));
    *syclParams=params;
    for(int i=0;i<kSubframeCount;i++)
    {
        const float vdd=mlx90640_detail::getVdd(input.subframe[i],&params);
        const float ta=mlx90640_detail::getTa(input.subframe[i],&params,vdd);
        contexts[i]=mlx90640_detail::buildToContext(input.subframe[i],&params,emissivity,vdd,ta,ta-kTaShift);
    }

    queue.parallel_for(sycl::range<1>(kSubframeCount * mlx90640_detail::kPixelCount), [=](sycl::item<1> item) {
        const size_t index=item.get_id(0);
        const size_t subframeIndex=index / mlx90640_detail::kPixelCount;
        const int pixelNumber=index % mlx90640_detail::kPixelCount;
        mlx90640_detail::calculatePixelToShort(
            subframes + subframeIndex * kSubframeWords,
            syclParams,
            contexts[subframeIndex],
            pixelNumber,
            temperature + pixelNumber);
    });
    queue.wait();

    std::memcpy(output->temperature,temperature,sizeof(output->temperature));
    sycl::free(temperature, queue);
    sycl::free(contexts, queue);
    sycl::free(syclParams, queue);
    sycl::free(subframes, queue);
}
#endif

} // namespace

void MLX90640RawFrame::process(MLX90640Frame *output, paramsMLX90640& params, float emissivity) const
{
#if THERMAL_CAMERA_FRAME_PROCESS_VARIANT == THERMAL_CAMERA_FRAME_PROCESS_OPENMP
    #pragma omp parallel sections num_threads(2)
    {
        #pragma omp section
        processSubframeSerial(this->subframe[0],&params,emissivity,output->temperature);
        #pragma omp section
        processSubframeSerial(this->subframe[1],&params,emissivity,output->temperature);
    }
#elif THERMAL_CAMERA_FRAME_PROCESS_VARIANT == THERMAL_CAMERA_FRAME_PROCESS_SYCL
    processFrameSycl(*this,output,params,emissivity);
#else
    for(int i=0;i<kSubframeCount;i++) processSubframeSerial(this->subframe[i],&params,emissivity,output->temperature);
#endif
}
