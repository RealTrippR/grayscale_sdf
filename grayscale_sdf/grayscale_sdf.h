#ifndef GRAYSCALE_SDF_H
#define GRAYSCALE_SDF_H

#include <errno.h>
#include <stdint.h>

#ifdef __cplusplus
#define SDF_API extern "C"
#else
#define SDF_API
#endif // _cplusplus

typedef enum {
    SDF_FORMAT_R8,
    SDF_FORMAT_R8G8,
    SDF_FORMAT_R8G8B8,
    SDF_FORMAT_R8G8B8A8,

    SDF_FORMAT_R16,
    SDF_FORMAT_R16G16,
    SDF_FORMAT_R16G16B16,
    SDF_FORMAT_R16G16B16A16
} sdf_format;

typedef struct {
    const uint8_t* pixels;
    uint16_t width;
    uint16_t height;
    sdf_format format;
} sdf_imageInfo;

typedef enum {
    SDF_RANGE,
    SDF_GREATER_THAN,
    SDF_LESS_THAN
} sdf_thresholdType;

typedef enum {
    SDF_CHANNEL_R,
    SDF_CHANNEL_G,
    SDF_CHANNEL_B,
    SDF_CHANNEL_A,
    SDF_CHANNEL_HSP
} sdf_thresholdChannel;

typedef struct {
    sdf_thresholdType type;
    sdf_thresholdChannel channel;
    float lowerBound;
    float upperBound;
} sdf_thresholdRange;
typedef struct {
    sdf_thresholdType type;
    sdf_thresholdChannel channel;
    float a;
} sdf_thresholdGreaterThan;
typedef struct {
    sdf_thresholdType type;
    sdf_thresholdChannel channel;
    float a;
} sdf_thresholdLessThan;
typedef union {
    sdf_thresholdRange          range;
    sdf_thresholdGreaterThan    greaterThan;
    sdf_thresholdLessThan       lessThan;
} sdf_threshold;

typedef void* (*sdf_malloc_fptr)(uint64_t);
typedef void (*sdf_free_fptr)(void*);


typedef struct
{
    void* args;
    void* (*func)(void*);
} sdf_task;

typedef void* sdf_taskHandle;

typedef errno_t(*sdf_launchTask_fptr)(sdf_task, sdf_taskHandle);
typedef errno_t(*sdf_joinTask_fptr)(sdf_taskHandle);

typedef struct {
    sdf_malloc_fptr malloc;
    sdf_free_fptr free;
    sdf_launchTask_fptr launchTask;
    sdf_joinTask_fptr joinTask;
    uint16_t taskHandleSize;
    uint16_t threadCount;
} sdf_instance;



/* returns 0 upon success, non-zero upon failure */
SDF_API errno_t sdf_imageToSdf(const sdf_instance* instance, const sdf_imageInfo* imageInfo, uint16_t maxScanDist, const sdf_threshold* thresholds, uint16_t thresholdCount, uint32_t* distanceFieldSizeOut, sdf_format distanceFieldFormat, int8_t* distanceFieldOut);

#endif