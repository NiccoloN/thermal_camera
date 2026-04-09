/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * Modified by TFT: added MLX90640_CalculateToShort, some optimizations,
 * made compatible with Miosix
 * Modified by DC: Removed I2C interface functions, now replaced with a new
 * optimized driver in mlx90640.h.
 */

#ifndef _MLX640_API_H_
#define _MLX640_API_H_

#include <stdint.h>

/*
 * Temperature returned by MLX90640_CalculateToShort is multipled by this
 */
const int scaleFactor=4;


typedef struct
{
    int16_t kVdd;
    int16_t vdd25;
    float KvPTAT;
    float KtPTAT;
    uint16_t vPTAT25;
    float alphaPTAT;
    int16_t gainEE;
    float tgc;
    float cpKv;
    float cpKta;
    uint8_t resolutionEE;
    uint8_t calibrationModeEE;
    float KsTa;
    float ksTo[4];
    int16_t ct[4];
    float alpha[768];    
    int16_t offset[768];    
    float kta[768];    
    float kv[768];
    float cpAlpha[2];
    int16_t cpOffset[2];
    float ilChessC[3]; 
    uint16_t brokenPixels[5];
    uint16_t outlierPixels[5];  
} paramsMLX90640;

namespace mlx90640_detail {

// Keep these helpers inline and header-visible so the SYCL variant can
// reuse the exact same math instead of maintaining a local copy.
constexpr int kPixelCount = 768;

struct ToCalculationContext
{
    uint16_t subPage;
    uint8_t mode;
    float emissivity;
    float vdd;
    float ta;
    float taTr;
    float gain;
    float irDataCP[2];
    float alphaCorrR[4];
};

inline float fast_rsqrtf(float number)
{
    union FloatUint32
    {
        float f;
        uint32_t i;
    };

    FloatUint32 conv;
    conv.f = number;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5F - (number * 0.5F * conv.f * conv.f);
    conv.f *= 1.5F - (number * 0.5F * conv.f * conv.f);
    return conv.f;
}

inline float quadrtf(float number)
{
    return fast_rsqrtf(fast_rsqrtf(number));
}

inline float pow4(float number)
{
    const float squared = number * number;
    return squared * squared;
}

inline float getVdd(const uint16_t *frameData, const paramsMLX90640 *params)
{
    float vdd = frameData[810];
    if(vdd > 32767) vdd -= 65536;

    const int resolutionRAM = (frameData[832] & 0x0C00) >> 10;
    const float resolutionCorrection =
        static_cast<float>(1u << params->resolutionEE) /
        static_cast<float>(1u << resolutionRAM);
    return (resolutionCorrection * vdd - params->vdd25) / params->kVdd + 3.3f;
}

inline float getTa(const uint16_t *frameData, const paramsMLX90640 *params, float vdd)
{
    float ptat = frameData[800];
    if(ptat > 32767) ptat -= 65536;

    float ptatArt = frameData[768];
    if(ptatArt > 32767) ptatArt -= 65536;
    ptatArt = (ptat / (ptat * params->alphaPTAT + ptatArt)) * 262144.f;//pow(2, (double)18);

    float ta = (ptatArt / (1.f + params->KvPTAT * (vdd - 3.3f)) - params->vPTAT25);
    ta = ta / params->KtPTAT + 25.f;

    return ta;
}

inline short convertTemperatureToShort(float to)
{
    //Clamp to -99..999°C multiplied by scaleFactor
    float rounded = to > 0.f ? to + 0.5f : to - 0.5f;
    if(rounded > 999.f) rounded = 999.f;
    if(rounded < -99.f) rounded = -99.f;
    return static_cast<short>(static_cast<float>(scaleFactor) * rounded);
}

inline ToCalculationContext buildToContext(const uint16_t *frameData,
    const paramsMLX90640 *params, float emissivity, float vdd, float ta, float tr)
{
    ToCalculationContext context = {};

    context.subPage = frameData[833];
    context.emissivity = emissivity;
    context.vdd = vdd;
    context.ta = ta;
    context.taTr = pow4(tr + 273.15f) - (pow4(tr + 273.15f) - pow4(ta + 273.15f)) / emissivity;

    context.alphaCorrR[0] = 1.f / (1.f + params->ksTo[0] * 40.f);
    context.alphaCorrR[1] = 1.f;
    context.alphaCorrR[2] = 1.f + params->ksTo[2] * params->ct[2];
    context.alphaCorrR[3] =
        context.alphaCorrR[2] * (1.f + params->ksTo[3] * (params->ct[3] - params->ct[2]));

//------------------------- Gain calculation -----------------------------------
    context.gain = frameData[778];
    if(context.gain > 32767) context.gain -= 65536;
    context.gain = params->gainEE / context.gain;

//------------------------- To calculation -------------------------------------
    context.mode = (frameData[832] & 0x1000) >> 5;

    context.irDataCP[0] = frameData[776];
    context.irDataCP[1] = frameData[808];
    for(int i = 0; i < 2; i++)
    {
        if(context.irDataCP[i] > 32767) context.irDataCP[i] -= 65536;
        context.irDataCP[i] = context.irDataCP[i] * context.gain;
    }
    context.irDataCP[0] = context.irDataCP[0] - params->cpOffset[0] *
        (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    if(context.mode == params->calibrationModeEE)
    {
        context.irDataCP[1] = context.irDataCP[1] - params->cpOffset[1] *
            (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    }
    else
    {
        context.irDataCP[1] = context.irDataCP[1] - (params->cpOffset[1] + params->ilChessC[0]) *
            (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    }

    return context;
}

inline bool calculatePixelTo(const uint16_t *frameData, const paramsMLX90640 *params,
    const ToCalculationContext& context, int pixelNumber, float *result)
{
    const int8_t ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
    const int8_t chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
    const int8_t conversionPattern =
        ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) *
        (1 - 2 * ilPattern);

    int8_t pattern;
    if(context.mode == 0)
    {
      pattern = ilPattern;
    }
    else
    {
      pattern = chessPattern;
    }

    if(pattern != static_cast<int8_t>(context.subPage)) return false;

    float irData = frameData[pixelNumber];
    if(irData > 32767) irData = irData - 65536;
    irData = irData * context.gain;

    irData = irData - params->offset[pixelNumber] * (1 + params->kta[pixelNumber] * (context.ta - 25)) *
        (1 + params->kv[pixelNumber] * (context.vdd - 3.3));
    if(context.mode != params->calibrationModeEE)
    {
      irData = irData + params->ilChessC[2] * (2 * ilPattern - 1) - params->ilChessC[1] * conversionPattern;
    }

    irData = irData / context.emissivity;

    irData = irData - params->tgc * context.irDataCP[context.subPage];

    const float alphaCompensated =
        (params->alpha[pixelNumber] - params->tgc * params->cpAlpha[context.subPage]) *
        (1 + params->KsTa * (context.ta - 25));

    float Sx = alphaCompensated * alphaCompensated * alphaCompensated *
        (irData + alphaCompensated * context.taTr);
    Sx = quadrtf(Sx) * params->ksTo[1];

    float To = quadrtf(irData / (alphaCompensated * (1 - params->ksTo[1] * 273.15f) + Sx) +
        context.taTr) - 273.15f;

    int8_t range;
    if(To < params->ct[1])
    {
        range = 0;
    }
    else if(To < params->ct[2])
    {
        range = 1;
    }
    else if(To < params->ct[3])
    {
        range = 2;
    }
    else
    {
        range = 3;
    }

    To = quadrtf(irData / (alphaCompensated * context.alphaCorrR[range] *
        (1 + params->ksTo[range] * (To - params->ct[range]))) + context.taTr) - 273.15f;

    *result = To;
    return true;
}

inline bool calculatePixelToShort(const uint16_t *frameData, const paramsMLX90640 *params,
    const ToCalculationContext& context, int pixelNumber, short *result)
{
    float to;
    if(calculatePixelTo(frameData, params, context, pixelNumber, &to) == false) return false;
    *result = convertTemperatureToShort(to);
    return true;
}

inline void calculateTo(const uint16_t *frameData, const paramsMLX90640 *params,
    const ToCalculationContext& context, float *result)
{
    for(int pixelNumber = 0; pixelNumber < kPixelCount; pixelNumber++)
    {
        calculatePixelTo(frameData, params, context, pixelNumber, result + pixelNumber);
    }
}

inline void calculateToShort(const uint16_t *frameData, const paramsMLX90640 *params,
    const ToCalculationContext& context, short *result)
{
    for(int pixelNumber = 0; pixelNumber < kPixelCount; pixelNumber++)
    {
        calculatePixelToShort(frameData, params, context, pixelNumber, result + pixelNumber);
    }
}

} // namespace mlx90640_detail

int MLX90640_ExtractParameters(const uint16_t *eeData, paramsMLX90640 *mlx90640);
float MLX90640_GetVdd(const uint16_t *frameData, const paramsMLX90640 *params);
float MLX90640_GetTa(const uint16_t *frameData, const paramsMLX90640 *params, float vdd);
void MLX90640_GetImage(const uint16_t *frameData, const paramsMLX90640 *params, float *result);
void MLX90640_CalculateTo(const uint16_t *frameData, const paramsMLX90640 *params, float emissivity, float vdd, float ta, float tr, float *result);
void MLX90640_CalculateToShort(const uint16_t *frameData, const paramsMLX90640 *params, float emissivity, float vdd, float ta, float tr, short *result);
    
#endif
