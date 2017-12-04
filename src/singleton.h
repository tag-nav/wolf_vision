/**
 * \file singleton.h
 *
 *  Created on: Aug 31, 2016
 *  \author: Jeremie Deray
 */

#ifndef WOLF_SINGLETON_H_
#define WOLF_SINGLETON_H_

#include <memory>

namespace wolf {
namespace internal {

/**
 * \brief A thread-safer (?) Singleton implementation with
 * argument forwarding.
 **/
template <class T>
class Singleton
{
  using SingletonOPtr = std::unique_ptr<T>;

public:

  template <typename... Args>
  static T& get(Args&&... args)
  {
    static SingletonOPtr instance_(new T(std::forward<Args>(args)...));

    return *instance_;
  }

  constexpr Singleton(const Singleton&)  = delete;
  constexpr Singleton(const Singleton&&) = delete;

  constexpr Singleton& operator=(const Singleton&) const = delete;
  constexpr Singleton& operator=(const Singleton&&) const = delete;

protected:

  constexpr Singleton() = default;
  virtual ~Singleton()  = default;
};

} // namespace internal
} // namespace wolf

#endif /* WOLF_SINGLETON_H_ */
