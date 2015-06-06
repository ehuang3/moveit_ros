/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Georgia Tech Research Corporation
 *  All rights reserved.
 *
 *  Author(s): Eric Huang <ehuang@gatech.edu>
 *  Georgia Tech Socially Intelligent Machines Lab
 *  Under Direction of Prof. Andrea Thomaz <athomaz@cc.gatech.edu>
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_EXCEPTION_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_EXCEPTION_
#include <stdexcept>
#include <cstdio>
#include <string.h>
#include <signal.h>
#include <execinfo.h>
#include <stdlib.h>
#include <apc_msgs/PrimitivePlan.h>

#define __APC_FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

enum APC_ERROR
{
    NO_ERROR,
    COLLISION_IN_TRAJECTORY,
    NO_PLAN,
};

namespace apc_exception
{
    std::string GetResolvedStackTrace();
    std::string show_backtrace();

    struct Exception : public std::runtime_error
    {
        Exception(const std::string& what)
            : std::runtime_error(what)
        {}
        Exception(const std::string& what, const apc_msgs::PrimitivePlan& plan)
            : std::runtime_error(what),
              what_plan(plan)
        {}
        virtual ~Exception() throw(){}
        apc_msgs::PrimitivePlan what_plan;
    };
}

#define APC_ASSERT_PLAN_VECTOR(x, plans, ...)                           \
    if (!(x))                                                           \
    {                                                                   \
        std::string bt = apc_exception::GetResolvedStackTrace();        \
        char buf[4096];                                                 \
        snprintf(buf, 4096, __VA_ARGS__);                               \
        char buf2[4096];                                                \
        snprintf(buf2, 4096, "in %s of %s line %d\n",                   \
                 __FUNCTION__, __APC_FILENAME__, __LINE__);             \
        std::string error = std::string(buf2) +                         \
            std::string(buf) + std::string("\n") + bt;                  \
        static int fi = 0;                                              \
        throw apc_exception::Exception(error,                           \
                                       plans[(fi++) % plans.size()]);   \
    }                                                                   \

#define APC_ASSERT_PLAN(x, plan, ...)                                   \
    if (!(x))                                                           \
    {                                                                   \
        std::string bt = apc_exception::GetResolvedStackTrace();        \
        char buf[4096];                                                 \
        snprintf(buf, 4096, __VA_ARGS__);                               \
        char buf2[4096];                                                \
        snprintf(buf2, 4096, "in %s of %s line %d\n",                   \
                 __FUNCTION__, __APC_FILENAME__, __LINE__);             \
        std::string error = std::string(buf2) +                         \
            std::string(buf) + std::string("\n") + bt;                  \
        throw apc_exception::Exception(error, plan);                    \
    }                                                                   \

#define APC_ASSERT(x, ...)                                              \
    if (!(x))                                                           \
    {                                                                   \
        std::string bt = apc_exception::GetResolvedStackTrace();        \
        char buf[4096];                                                 \
        snprintf(buf, 4096, __VA_ARGS__);                               \
        char buf2[4096];                                                \
        snprintf(buf2, 4096, "in %s of %s line %d\n",                   \
                 __FUNCTION__, __APC_FILENAME__, __LINE__);             \
        std::string error = std::string(buf2) +                         \
            std::string(buf) + std::string("\n") + bt;                  \
        throw apc_exception::Exception(error);                          \
    }                                                                   \

#endif
