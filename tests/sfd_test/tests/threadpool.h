/*
Copyright (C) 2025 Tripp Robins

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#ifdef __cplusplus
#define THREAD_POOL_API extern "C"
#else
#define THREAD_POOL_API
#endif
#include <stdint.h>
#include <errno.h>

typedef struct
{
    uint16_t id;
    uint16_t threadCount;
} ThreadPoolHandle;
typedef struct
{
    uint8_t __;
} ThreadPoolTaskHandle;

typedef struct
{
    void* args;
    void* (*func)(void*);
    ThreadPoolTaskHandle* hdl;
} ThreadPoolTask;


/*
Creates a new thread pool.
@param ThreadPoolHandle* - a handle to the thread pool.
@param uint32_t timeoutMS* - the maximum time of inactivity in MS before an worker thread will close. Preferably this should be a decent bit of time to avoid the overhead caused by restarting an inactive thread.
@return errno_t - 0 upon success, non-zero value upon failure.*/
THREAD_POOL_API errno_t ThreadPool_New(ThreadPoolHandle*, uint32_t timeoutMS);

/*
Destroys a thread pool.
@param ThreadPoolHandle* - a handle to the thread pool.
@return errno_t - 0 upon success, non-zero value upon failure.*/
THREAD_POOL_API errno_t ThreadPool_Destroy(ThreadPoolHandle*);

/*
Launches a task.
@param ThreadPoolHandle* - a handle to the thread pool.
@param ThreadPoolTask task - the task to launch.
@param ThreadPoolTaskHandle* taskHdl - a handle to the task.
@return errno_t - 0 upon success, non-zero value upon failure.*/
THREAD_POOL_API errno_t ThreadPool_LaunchTask(ThreadPoolHandle tpHdl, ThreadPoolTask task, ThreadPoolTaskHandle* taskHdl);

/*
Waits for a task to complete
@param ThreadPoolTaskHandle* - the handle to wait for.*/
THREAD_POOL_API void ThreadPool_JoinTask(ThreadPoolTaskHandle* taskHdl);


#endif