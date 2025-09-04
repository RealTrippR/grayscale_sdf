#include "../grayscale_sdf.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define MAX_TASK_HANDLE_COUNT 32u

typedef	uint8_t u8;
typedef int8_t i8;
typedef uint16_t u16;
typedef int16_t i16;
typedef uint32_t u32;
typedef int32_t i32;
typedef uint64_t u64;
typedef int64_t i64;
typedef float f32;
typedef double f64;

typedef struct {
    sdf_thresholdType type;
    sdf_thresholdChannel channel;
    void* pNext;
} sdf_thresholdGeneric;

static u8 sizeofFmt(sdf_format fmt) 
{
    switch (fmt) {
    case SDF_FORMAT_R8:
        return 1;
    case SDF_FORMAT_R8G8:
        return 2;
    case SDF_FORMAT_R8G8B8:
        return 3;
    case SDF_FORMAT_R8G8B8A8:
        return 4;
    case SDF_FORMAT_R16:
        return 2;
    case SDF_FORMAT_R16G16:
        return 4;
    case SDF_FORMAT_R16G16B16:
        return 6;
    case SDF_FORMAT_R16G16B16A16:
        return 8;
    }
    return 0;
}

typedef struct {
    u8 r;
} colorR8;
typedef struct {
    u8 r;
    u8 g;
} colorR8G8;
typedef struct  {
    u8 r;
    u8 g;
    u8 b;
} colorR8G8B8 ;
typedef struct  {
    u8 r;
    u8 g;
    u8 b;
    u8 a;
} colorR8G8B8A8;
typedef struct {
    u16 r;
} colorR16;
typedef struct {
    u16 r;
    u16 g;
} colorR16G16;
typedef struct  {
    u16 r;
    u16 g;
    u16 b;
} colorR16G16B16;
typedef struct  {
    u16 r;
    u16 g;
    u16 b;
    u16 a;
} colorR16G16B16A16;

static f32 HSPLuminance(f32 r, f32 g, f32 b)
{
    // https://alienryderflex.com/hsp.html
    return sqrtf(.299f * r*r + .587f * g*g + .114f * b*b);
}


static f32 pxlHSPLuminance(const u8* pixel, sdf_format pixelFormat)
{
    if (pixelFormat == SDF_FORMAT_R8) 
    {
        const colorR8* c = (colorR8*)pixel;
        return c->r / 255.f;
    }
    else if (pixelFormat==SDF_FORMAT_R8G8)
    {
        const colorR8G8* c = (colorR8G8*)pixel;
        return HSPLuminance(c->r / 255.f, c->g / 255.f, 0);
    }
    else if (pixelFormat == SDF_FORMAT_R8G8B8)
    {
        const colorR8G8B8* c = (colorR8G8B8*)pixel;
        return HSPLuminance(c->r / 255.f, c->g / 255.f, c->b / 255.f);
    }
    else if (pixelFormat == SDF_FORMAT_R8G8B8A8)
    {
        const colorR8G8B8A8* c = (colorR8G8B8A8*)pixel;
        return HSPLuminance(c->r / 255.f, c->g / 255.f, c->b / 255.f) * (c->a / 255.f);
    }
    else if (pixelFormat == SDF_FORMAT_R16)
    {
        const colorR16* c = (colorR16*)pixel;
        return c->r / 65535.f;
    }
    else if (pixelFormat == SDF_FORMAT_R16G16)
    {
        const colorR16G16* c = (colorR16G16*)pixel;
        return HSPLuminance(c->r / 65535.f, c->g / 65535.f, 0);
    }
    else if (pixelFormat == SDF_FORMAT_R16G16B16)
    {
        const colorR16G16B16* c = (colorR16G16B16*)pixel;
        return HSPLuminance(c->r / 65535.f, c->g / 65535.f, c->b / 65535.f);
    }
    else if (pixelFormat == SDF_FORMAT_R16G16B16A16)
    {
        const colorR16G16B16A16* c = (colorR16G16B16A16*)pixel;
        return HSPLuminance(c->r / 65535.f, c->g / 65535.f, c->b / 65535.f) * (c->a / 65535.f);
    }
}

static bool evaluateThreshold(const u8* pixel, sdf_format pxlFormat, const sdf_threshold* __t__)
{
    const sdf_thresholdGeneric* t = (sdf_thresholdGeneric*)__t__;
    f32 v;

    if (pxlFormat < SDF_FORMAT_R16) {
        u8 r;
        u8 g;
        u8 b;
        u8 a;
        switch (t->channel)
        {
        case SDF_CHANNEL_R:
            v = *(u8*)(pixel) / 255.f;
            break;
        case SDF_CHANNEL_G:
            v = *(u8*)(pixel + 1) / 255.f;
            break;
        case SDF_CHANNEL_B:
            v = *(u8*)(pixel + 2) / 255.f;
            break;
        case SDF_CHANNEL_A:
            v = (u8)(*(u8*)(pixel + 3)) / 255.f;
            break;
        case SDF_CHANNEL_HSP:
            r = *(u8*)(pixel);
            g = *(u8*)(pixel + 1);
            b = *(u8*)(pixel + 2);
            a = *(u8*)(pixel + 3);
            v = HSPLuminance(r / 255.f, g / 255.f, b / 255.f) * (a / 255.f);
            break;
        default:
            break;
        }
    }
    else {
        u16 r;
        u16 g;
        u16 b;
        u16 a;
        switch (t->channel)
        {
        case SDF_CHANNEL_R:
            v = *(u16*)pixel / 65535.f;
            break;
        case SDF_CHANNEL_G:
            v = *(u16*)pixel + 1 / 65535.f;
            break;
        case SDF_CHANNEL_B:
            v = *(u16*)pixel + 2 / 65535.f;
            break;
        case SDF_CHANNEL_A:
            v = *(u16*)pixel + 3 / 65535.f;
            break;
        case SDF_CHANNEL_HSP:
            r = *(u16*)pixel;
            g = *(u16*)pixel + 1;
            b = *(u16*)pixel + 2;
            a = *(u16*)pixel + 3;
            v = HSPLuminance(r / 65535.f, g / 65535.f, b / 65535.f) * (a / 65535.f);
            break;
        default:
            break;
        }
    }


    if (t->type == SDF_RANGE) {
        const sdf_thresholdRange* thres = (sdf_thresholdRange*)t;
        if (v > thres->upperBound || v < thres->lowerBound)
            return false;
    } 
    else if (t->type == SDF_LESS_THAN)
    {
        const sdf_thresholdLessThan* thres = (sdf_thresholdLessThan*)t;
        if (v >= thres->a) {
            return false;
        }
    }
    else if (t->type == SDF_GREATER_THAN)
    {
        const sdf_thresholdGreaterThan* thres = (sdf_thresholdGreaterThan*)t;
        if (v <= thres->a) {
            return false;
        }
    }

    return true;
}


static bool evaluateThresholds(const u8* pixel, sdf_format pxlFormat, const sdf_threshold* t, uint16_t thresholdCount)
{
    bool b = true;
    u16 i;
    for (i = 0; i < thresholdCount; ++i)
    {
        if (evaluateThreshold(pixel, pxlFormat, t+i) == false) {
            b = false;
        }
    }
    return b;
}

static u32 getSegmentSize(u32 count, u32 divisions, u32 divisionIdx)
{
    i32 i = divisionIdx;
    i32 j = (count + 1) / divisions;
    i32 k = count - (i + 1) * j;
    if (i == (i32)divisions - 1)
        j += k;
    return j;
}

// 1D EDT - Felzenszwalb & Huttenlocher - https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf
static void edt_1d(const sdf_instance* instance, const float* f, float* d, u32* posList, int n) {
    u32* v = instance->malloc(sizeof(v[0]) * n);
    double* z = instance->malloc(sizeof(z[0]) * (n + 1));
    int k = 0;
    v[0] = 0;
    z[0] = -INFINITY;
    z[1] = INFINITY;

    for (int q = 1; q < n; ++q) {
        double s;
        while (k >= 0) {
            int p = v[k];
            s = ((double)f[q] + (double)(q * q) - ((double)f[p] + (double)(p * p))) / (2.0 * (q - p));
            if (s > z[k]) break;
            --k;
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = INFINITY;
    }

    k = 0;
    for (int i = 0; i < n; ++i) {
        while (z[k + 1] < i) ++k;
        u32 p = v[k];
        d[i] = (i - p) * (i - p) + f[p];
        if (posList) {
            posList[i] = p;
        }
    }
    instance->free(v);
    instance->free(z);
}

typedef struct {
    bool invert;
    const sdf_instance* instance;
    const sdf_imageInfo* imageInfo;
    u16 maxScanDist;
    const sdf_threshold* thresholds;
    u16 thresholdCount;
    sdf_format distanceFieldFormat;
    i8* distanceFieldOut;
} EDT_pass_Args;

static void* EDT_pass(void*  args___)
{
    EDT_pass_Args* args = args___;
    bool invert = args->invert;
    const sdf_instance* instance = args->instance;
    const sdf_imageInfo* imageInfo = args->imageInfo;
    u16 maxScanDist = args->maxScanDist;
    const sdf_threshold* thresholds = args->thresholds;
    u16 thresholdCount = args->thresholdCount;
    sdf_format distanceFieldFormat = args->distanceFieldFormat;
    i8* distanceFieldOut = args->distanceFieldOut;
    const u16 width = imageInfo->width;
    const u16 height = imageInfo->height;

    f32* thresholdMap = instance->malloc(sizeof(thresholdMap[0]) * width * height);
    memset(thresholdMap, 0, sizeof(thresholdMap[0])* width* height);
    u32 stride = sizeofFmt(imageInfo->format);
    for (u32 y = 0; y < height; ++y)
    {
        for (u32 x = 0; x < width; ++x)
        {
            const u8* pixel = imageInfo->pixels + (x + (width * y)) * stride;
            if (evaluateThresholds(pixel, imageInfo->format, thresholds, thresholdCount)) {
                if (!invert) {
                    thresholdMap[x + (width * y)] = 0;
                }
                else {
                    thresholdMap[x + (width * y)] = INFINITY;
                }
            }
            else {
                if (!invert) {
                    thresholdMap[x + (width * y)] = INFINITY;
                }
                else {
                    thresholdMap[x + (width * y)] = 0;
                }
            }
        }
    }

    // feature
    f32* f = instance->malloc(sizeof(f32) * (width > height ? width : height));
    // distance
    f32* d = instance->malloc(sizeof(f32) * (width > height ? width : height));
    u32* a = instance->malloc(sizeof(a[0]) * (width > height ? width : height));
    u32* posRows = NULL;
    if (distanceFieldFormat == SDF_FORMAT_R16G16 || distanceFieldFormat == SDF_FORMAT_R8G8) {
        posRows = instance->malloc(sizeof(posRows[0]) * width * height);
    }


    // Row-wise 1D EDT
    for (u16 y = 0; y < height; ++y) {
        for (u16 x = 0; x < width; ++x) {
            f[x] = thresholdMap[y * width + x];
        }
        if (posRows) {
            edt_1d(instance, f, d, posRows + width * y, width);
        }
        else {
            edt_1d(instance, f, d, NULL, width);
        }
        for (u16 x = 0; x < width; ++x) {
            thresholdMap[y * width + x] = d[x];
        }
    }

    // Column-wise 1D EDT (final distances)
    for (u16 x = 0; x < width; ++x)
    {
        for (u16 y = 0; y < height; ++y) 
        {
            f[y] = thresholdMap[y * width + x];
        }

        edt_1d(instance, f, d, a, height);

        for (u16 y = 0; y < height; ++y) {
            float dist = sqrtf(d[y]);

            if (dist == 0)
                continue;
            // Clamp to maxScanDist
            if (dist > maxScanDist) 
                dist = (float)maxScanDist;
            if (invert)
                dist = -dist;
            const f32 px = thresholdMap[x + y * width];
            size_t idx = y * width + x;

            f32 dir=0;
         
            if (posRows) {
                u32 fy = a[y];
                u32 fx = posRows[fy * width + x];
                f32 dx = (f32)x-fx;
                f32 dy = (f32)y-fy;
                dir = atan2f(dy, dx);
            }


          

            switch (distanceFieldFormat) {
            case SDF_FORMAT_R8:
                if (px) {
                    distanceFieldOut[idx] = (i8)(dist / maxScanDist * 127);
                }
                break;
            case SDF_FORMAT_R16:
                if (px) {
                    ((i16*)distanceFieldOut)[idx] = (i16)(dist / maxScanDist * 32767);
                }
                break;
            case SDF_FORMAT_R8G8:
                if (px) {
                    distanceFieldOut[idx * 2] = (i8)(dist / maxScanDist * 127);
                    distanceFieldOut[idx * 2 + 1] = (i8)((dir + 3.141592653589) / 6.28318530718) * 255;
                }
                break;
            case SDF_FORMAT_R16G16:
                if (px) {
                    ((i16*)distanceFieldOut)[idx * 2] = (i16)(dist / maxScanDist * 32767);
                    ((u16*)distanceFieldOut)[idx * 2 + 1] = (u16)((dir + 3.141592653589) / 6.28318530718) * 65535;
                }
                break;
            }
        }
    }
    instance->free(thresholdMap);
    if (posRows)
        instance->free(posRows);

    instance->free(f);
    instance->free(d);
    instance->free(a);

    return NULL;
}

SDF_API errno_t sdf_imageToSdf(const sdf_instance* instance, const sdf_imageInfo* imageInfo, u16 maxScanDist, const sdf_threshold* thresholds, u16 thresholdCount, u32* distanceFieldSizeOut, sdf_format distanceFieldFormat, i8* distanceFieldOut, bool _unsigned)
{
    if (instance->threadCount > MAX_TASK_HANDLE_COUNT)
        return -4;

    if (!(distanceFieldFormat == SDF_FORMAT_R8 || distanceFieldFormat == SDF_FORMAT_R16 || distanceFieldFormat == SDF_FORMAT_R8G8 || distanceFieldFormat == SDF_FORMAT_R16G16)) {
        return -3;
    }

    u8 fieldStride = sizeofFmt(distanceFieldFormat);
    if (distanceFieldSizeOut) {
        *distanceFieldSizeOut = imageInfo->width * imageInfo->height * fieldStride;
        if (!distanceFieldOut) {
            return 0;
        }
    }

    EDT_pass_Args args = { false,instance, imageInfo, maxScanDist, thresholds, thresholdCount, distanceFieldFormat, distanceFieldOut };
    EDT_pass_Args args2 = { true,instance, imageInfo, maxScanDist, thresholds, thresholdCount, distanceFieldFormat, distanceFieldOut };
    sdf_task task;
    task.args = &args;
    task.func = EDT_pass;
    sdf_task task2;
    task2.args = &args2;
    task2.func = EDT_pass;
    
    errno_t retcode=0u;
    void* hdls = instance->malloc(instance->taskHandleSize * 2);
    if (instance->launchTask(task, (u8*)hdls)) {
        retcode = -1;
        goto bail;
    }
    if (instance->launchTask(task2, (u8*)hdls + instance->taskHandleSize)) {
        retcode = -1;
        goto bail;
    }
    if (instance->joinTask((u8*)hdls)) {
        retcode = -1;
        goto bail;
    }
    if (instance->joinTask((u8*)hdls + instance->taskHandleSize)) {
        retcode = -1;
        goto bail;
    }

bail:
    if (hdls) {
        instance->free(hdls);
    }
    return retcode;
}





typedef struct {
    bool invert;
    const sdf_instance* instance;
    const sdf_imageInfo* imageInfo;
    u16 maxScanDist;
    const sdf_threshold* thresholds;
    u16 thresholdCount;
    sdf_format distanceFieldFormat;
    u8* distanceFieldOut;
} EDT_passUnsigned_Args;

static void* EDT_passUnsigned(void* args___)
{
    EDT_passUnsigned_Args* args = args___;
    bool invert = args->invert;
    const sdf_instance* instance = args->instance;
    const sdf_imageInfo* imageInfo = args->imageInfo;
    u16 maxScanDist = args->maxScanDist;
    const sdf_threshold* thresholds = args->thresholds;
    u16 thresholdCount = args->thresholdCount;
    sdf_format distanceFieldFormat = args->distanceFieldFormat;
    u8* distanceFieldOut = args->distanceFieldOut;
    const u16 width = imageInfo->width;
    const u16 height = imageInfo->height;

    f32* thresholdMap = instance->malloc(sizeof(thresholdMap[0]) * width * height);
    memset(thresholdMap, 0, sizeof(thresholdMap[0]) * width * height);
    u32 stride = sizeofFmt(imageInfo->format);
    for (u32 y = 0; y < height; ++y)
    {
        for (u32 x = 0; x < width; ++x)
        {
            const u8* pixel = imageInfo->pixels + (x + (width * y)) * stride;
            if (evaluateThresholds(pixel, imageInfo->format, thresholds, thresholdCount)) {
                if (!invert) {
                    thresholdMap[x + (width * y)] = 0;
                }
                else {
                    thresholdMap[x + (width * y)] = INFINITY;
                }
            }
            else {
                if (!invert) {
                    thresholdMap[x + (width * y)] = INFINITY;
                }
                else {
                    thresholdMap[x + (width * y)] = 0;
                }
            }
        }
    }

    // feature
    f32* f = instance->malloc(sizeof(f32) * (width > height ? width : height));
    // distance
    f32* d = instance->malloc(sizeof(f32) * (width > height ? width : height));
    u32* a = instance->malloc(sizeof(a[0]) * (width > height ? width : height));
    u32* posRows = NULL;
    if (distanceFieldFormat == SDF_FORMAT_R16G16 || distanceFieldFormat == SDF_FORMAT_R8G8) {
        posRows = instance->malloc(sizeof(posRows[0]) * width * height);
    }


    // Row-wise 1D EDT
    for (u16 y = 0; y < height; ++y) {
        for (u16 x = 0; x < width; ++x) {
            f[x] = thresholdMap[y * width + x];
        }
        if (posRows) {
            edt_1d(instance, f, d, posRows + width * y, width);
        }
        else {
            edt_1d(instance, f, d, NULL, width);
        }
        for (u16 x = 0; x < width; ++x) {
            thresholdMap[y * width + x] = d[x];
        }
    }

    // Column-wise 1D EDT (final distances)
    for (u16 x = 0; x < width; ++x)
    {
        for (u16 y = 0; y < height; ++y)
        {
            f[y] = thresholdMap[y * width + x];
        }

        edt_1d(instance, f, d, a, height);

        for (u16 y = 0; y < height; ++y) {
            float dist = sqrtf(d[y]);

            if (dist == 0)
                continue;
            // Clamp to maxScanDist
            if (dist > maxScanDist)
                dist = (float)maxScanDist;

            const f32 px = thresholdMap[x + y * width];
            size_t idx = y * width + x;

            f32 dir = 0;

            if (posRows) {
                u32 fy = a[y];
                u32 fx = posRows[fy * width + x];
                f32 dx = (f32)x - fx;
                f32 dy = (f32)y - fy;
                dir = atan2f(dy, dx);
            }




            switch (distanceFieldFormat) {
            case SDF_FORMAT_R8:
                if (px) {
                    distanceFieldOut[idx] = (u8)(dist / maxScanDist * 255);
                }
                break;
            case SDF_FORMAT_R16:
                if (px) {
                    ((i16*)distanceFieldOut)[idx] = (u16)(dist / maxScanDist * 65535);
                }
                break;
            case SDF_FORMAT_R8G8:
                if (px) {
                    distanceFieldOut[idx * 2] = (u8)(dist / maxScanDist * 255);
                    distanceFieldOut[idx * 2 + 1] = (i8)((dir + 3.141592653589) / 6.28318530718) * 255;
                }
                break;
            case SDF_FORMAT_R16G16:
                if (px) {
                    ((i16*)distanceFieldOut)[idx * 2] = (u16)(dist / maxScanDist * 65535);
                    ((i16*)distanceFieldOut)[idx * 2 + 1] = (u16)((dir + 3.141592653589) / 6.28318530718) * 65535;
                }
                break;
            }
        }
    }
    instance->free(thresholdMap);
    if (posRows)
        instance->free(posRows);

    instance->free(f);
    instance->free(d);
    instance->free(a);

    return NULL;
}

SDF_API errno_t sdf_imageToUdf(const sdf_instance* instance, const sdf_imageInfo* imageInfo, uint16_t maxScanDist, const sdf_threshold* thresholds, uint16_t thresholdCount, uint32_t* distanceFieldSizeOut, sdf_format distanceFieldFormat, uint8_t* distanceFieldOut) 
{
    if (instance->threadCount > MAX_TASK_HANDLE_COUNT)
        return -4;

    if (!(distanceFieldFormat == SDF_FORMAT_R8 || distanceFieldFormat == SDF_FORMAT_R16 || distanceFieldFormat == SDF_FORMAT_R8G8 || distanceFieldFormat == SDF_FORMAT_R16G16)) {
        return -3;
    }

    u8 fieldStride = sizeofFmt(distanceFieldFormat);
    if (!distanceFieldOut) {
        *distanceFieldSizeOut = imageInfo->width * imageInfo->height * fieldStride;
        return 0;
    }

    EDT_passUnsigned_Args args = { false, instance, imageInfo, maxScanDist, thresholds, thresholdCount, distanceFieldFormat, distanceFieldOut };
    EDT_passUnsigned_Args args2 = { true, instance, imageInfo, maxScanDist, thresholds, thresholdCount, distanceFieldFormat, distanceFieldOut };
    sdf_task task;
    task.args = &args;
    task.func = EDT_passUnsigned;
    sdf_task task2;
    task2.args = &args2;
    task2.func = EDT_passUnsigned;

    errno_t retcode = 0u;
    void* hdls = instance->malloc(instance->taskHandleSize * 2);
    if (instance->launchTask(task, (u8*)hdls)) {
        retcode = -1;
        goto bail;
    }
    if (instance->launchTask(task2, (u8*)hdls + instance->taskHandleSize)) {
        retcode = -1;
        goto bail;
    }
    if (instance->joinTask((u8*)hdls)) {
        retcode = -1;
        goto bail;
    }
    if (instance->joinTask((u8*)hdls + instance->taskHandleSize)) {
        retcode = -1;
        goto bail;
    }

bail:
    if (hdls) {
        instance->free(hdls);
    }
    return retcode;
}