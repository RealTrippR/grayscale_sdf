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


/*
@param sdf_instance* instance - sdf instance.
@param sdf_imageInfo imageInfo - image information.
@param uint16_t maxScanDist - the maximum distance in pixels of the field. Each value in the distance field will be a fraction of distance over this value.
@param sdf_threshold* thresholds - thresholds to determine what is a feature and what is not a feature.
@param uint16_t thresholdCount - number of thresholds.
@param uint32_t distanceFieldSizeOut - the size (in bytes) of the distance field that will be generated.
@param sdf_format distanceFieldFormat - the requested output format for the distance field. Only 4 valid options exist: SDF_FORMAT_R8, SDF_FORMAT_R16, SDF_FORMAT_R8G8, SDF_FORMAT_R16G16. If the format given has more than 1 channel, it will store direction (from 0-1, where 0 is right) in the second channel.
@param int8_t* distanceFieldOut - the distance field buffer to write to.
@return errno_t - returns 0 upon success, non-zero in the event of a failure.
returns 0 upon success, non-zero upon failure */
SDF_API errno_t sdf_imageToSdf(const sdf_instance* instance, const sdf_imageInfo* imageInfo, uint16_t maxScanDist, const sdf_threshold* thresholds, uint16_t thresholdCount, uint32_t* distanceFieldSizeOut, sdf_format distanceFieldFormat, int8_t* distanceFieldOut);

#endif