/**
 * \file processor_logging.h
 *
 *  Created on: Oct 5, 2017
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_PROCESSOR_LOGGING_H_
#define _WOLF_PROCESSOR_LOGGING_H_

/// @brief un-comment for IDE highlights.
//#include "logging.h"

#define __INTERNAL_WOLF_ASSERT_PROCESSOR \
  static_assert(std::is_base_of<ProcessorBase, \
   typename std::remove_pointer<decltype(this)>::type>::value, \
    "This macro can be used only within the body of a " \
    "non-static " \
    "ProcessorBase (and derived) function !");

#define WOLF_PROCESSOR_INFO(...)  __INTERNAL_WOLF_ASSERT_PROCESSOR WOLF_INFO_NAMED(getType(),  __VA_ARGS__);
#define WOLF_PROCESSOR_WARN(...)  __INTERNAL_WOLF_ASSERT_PROCESSOR WOLF_WARN_NAMED(getType(),  __VA_ARGS__);
#define WOLF_PROCESSOR_ERROR(...) __INTERNAL_WOLF_ASSERT_PROCESSOR WOLF_ERROR_NAMED(getType(), __VA_ARGS__);
#define WOLF_PROCESSOR_DEBUG(...) __INTERNAL_WOLF_ASSERT_PROCESSOR WOLF_DEBUG_NAMED(getType(), __VA_ARGS__);

#define WOLF_PROCESSOR_INFO_COND(cond, ...)  __INTERNAL_WOLF_ASSERT_PROCESSOR WOLF_INFO_NAMED_COND(getType(),  cond, __VA_ARGS__);
#define WOLF_PROCESSOR_WARN_COND(cond, ...)  __INTERNAL_WOLF_ASSERT_PROCESSOR WOLF_WARN_NAMED_COND(getType(),  cond, __VA_ARGS__);
#define WOLF_PROCESSOR_ERROR_COND(cond, ...) __INTERNAL_WOLF_ASSERT_PROCESSOR WOLF_ERROR_NAMED_COND(getType(), cond, __VA_ARGS__);
#define WOLF_PROCESSOR_DEBUG_COND(cond, ...) __INTERNAL_WOLF_ASSERT_PROCESSOR WOLF_DEBUG_NAMED_COND(getType(), cond, __VA_ARGS__);

#endif /* _WOLF_PROCESSOR_LOGGING_H_ */
