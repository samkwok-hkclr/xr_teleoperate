// Copyright 2022 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DUAL_ARM__VISIBILITY_CONTROL_H_
#define DUAL_ARM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DUAL_ARM_EXPORT __attribute__ ((dllexport))
    #define DUAL_ARM_IMPORT __attribute__ ((dllimport))
  #else
    #define DUAL_ARM_EXPORT __declspec(dllexport)
    #define DUAL_ARM_IMPORT __declspec(dllimport)
  #endif
  #ifdef DUAL_ARM_BUILDING_LIBRARY
    #define DUAL_ARM_PUBLIC DUAL_ARM_EXPORT
  #else
    #define DUAL_ARM_PUBLIC DUAL_ARM_IMPORT
  #endif
  #define DUAL_ARM_PUBLIC_TYPE DUAL_ARM_PUBLIC
  #define DUAL_ARM_LOCAL
#else
  #define DUAL_ARM_EXPORT __attribute__ ((visibility("default")))
  #define DUAL_ARM_IMPORT
  #if __GNUC__ >= 4
    #define DUAL_ARM_PUBLIC __attribute__ ((visibility("default")))
    #define DUAL_ARM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DUAL_ARM_PUBLIC
    #define DUAL_ARM_LOCAL
  #endif
  #define DUAL_ARM_PUBLIC_TYPE
#endif

#endif  // DUAL_ARM__VISIBILITY_CONTROL_H_
