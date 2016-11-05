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
 * \brief A thread-safer Singleton implementation with
 * argument forwarding.
 **/
template <class T>
class Singleton
{
  /**
   * \brief Custom deleter to by-pass private destructor issue.
   **/
  struct Deleter;

  using SingletonOPtr = std::unique_ptr<T, Deleter>;

public:

  template <typename... Args>
  static T& get(Args&&... args)
  {
    static SingletonOPtr instance_(new T(args...));

    return *instance_;
  }

  constexpr Singleton(const Singleton&)       = delete;
  //constexpr Singleton(const Singleton&&)      = delete;

  constexpr Singleton& operator=(const Singleton&)  = delete;
  //constexpr Singleton& operator=(const Singleton&&) = delete;

protected:

  Singleton() = default;

  virtual ~Singleton() = default;
};

template <class T>
struct Singleton<T>::Deleter
{
  void operator()( const T* const p )
  {
    delete p;
  }
};

} // namespace internal
} // namespace wolf

#endif /* WOLF_SINGLETON_H_ */
