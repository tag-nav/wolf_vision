/**
 * \file test_wolf_logging.cpp
 *
 * Created on: Oct 28, 2016
 * \author: Jeremie Deray
 */

#include "wolf.h"
#include "logging.h"

int main(int, char*[])
{
  WOLF_INFO("test info ", 5, " ", 0.123);

  WOLF_WARN("test warn ", 5, " ", 0.123);

  WOLF_ERROR("test error ", 5, " ", 0.123);

  WOLF_TRACE("test trace ", 5, " ", 0.123);

  WOLF_DEBUG("test debug ", 5, " ", 0.123);

  // manually enable debug logging
  WOLF_ENABLE_DEBUG_LOG();

  WOLF_DEBUG("test enable debug");

  // manually disable debug logging
  WOLF_DISABLE_DEBUG_LOG();

  WOLF_DEBUG("test disable debug");

  return 0;
}
