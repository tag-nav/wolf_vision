/**
 * \file make_unique.h
 *
 *  Created on: Oct 11, 2017
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_MAKE_UNIQUE_H_
#define _WOLF_MAKE_UNIQUE_H_

#include <memory>

namespace wolf {

/// @note this appears only in cpp14
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}

#endif /* _WOLF_MAKE_UNIQUE_H_ */
