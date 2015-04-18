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

#define __APC_FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define APC_ASSERT(x, ...)                                  \
    if (!(x))                                               \
    {                                                       \
        char buf[4096];                                     \
        snprintf(buf, 4096, __VA_ARGS__);                   \
        char buf2[4096];                                    \
        snprintf(buf2, 4096, "in %s of %s line %d\n",       \
                 __FUNCTION__, __APC_FILENAME__, __LINE__); \
        std::string error = std::string(buf2) +             \
            std::string(buf);                               \
        throw std::logic_error(error);                      \
    }                                                       \

#endif
