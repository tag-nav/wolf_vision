/**
 * \file logging.h
 *
 *  Created on: Oct 27, 2016
 *  \author: Jeremie Deray
 */

#ifndef WOLF_LOGGING_H_
#define WOLF_LOGGING_H_

// third_party include
// spdlog include
#include "spdlog/spdlog.h"
// enable the use of ostream operator<<
#include "spdlog/fmt/bundled/ostream.h"

// Wolf includes
#include "singleton.h"

namespace wolf {
namespace internal {

class Logger : public Singleton<Logger>
{
  friend class Singleton<Logger>;

protected:

  Logger();
  ~Logger();

// It is counter intuitive to have those functions
// public but that for the sake of the macros below
public:

  Logger(Logger&)         = delete;
  void operator=(Logger&) = delete;

  template<typename... Args>
  void info(const Args&... args);

  template<typename... Args>
  void warn(const Args&... args);

  template<typename... Args>
  void error(const Args&... args);

  template<typename... Args>
  void debug(const Args&... args);

  template<typename... Args>
  void trace(const Args&... args);

  bool set_async_queue(const std::size_t q_size);

protected:

  const std::string log_name_ = "wolf_main_console";

  std::shared_ptr<spdlog::logger> console_;

  std::string repeat_string(const std::string &str, std::size_t n)
  {
    if (n == 0) return {};

    if (n == 1 || str.empty()) return str;

    const auto n_char = str.size();

    if (n_char == 1) return std::string(n, str[0]);

    std::string res(str);
    res.reserve(n_char * n);

    std::size_t m = 2;
    for (; m <= n; m *= 2) res += res;

    n -= m*.5;

    res.append(res.c_str(), n * n_char);

    return res;
  }
};

inline Logger::Logger()
{
  // Create main logger
  console_ = spdlog::stdout_color_mt(log_name_);

#ifdef _WOLF_TRACE
  console_->set_level(spdlog::level::trace);
#endif

  // Enable asynchronous logging
  // Queue size must be a power of 2
  spdlog::set_async_mode(4096);

  // Logging pattern is :
  // [thread num][hour:minutes:seconds.nanoseconds][log type] #log-content
  console_->set_pattern("[%t][%H:%M:%S.%F][%l] %v");
}

inline Logger::~Logger()
{
  spdlog::drop(log_name_);
}

template<typename... Args>
void Logger::info(const Args&... args)
{
  console_->info(repeat_string("{}", sizeof...(args)).c_str(), args...);
}

template<typename... Args>
void Logger::warn(const Args&... args)
{
  console_->warn(repeat_string("{}", sizeof...(args)).c_str(), args...);
}

template<typename... Args>
void Logger::error(const Args&... args)
{
  console_->error(repeat_string("{}", sizeof...(args)).c_str(), args...);
}

template<typename... Args>
void Logger::debug(const Args&... args)
{
  console_->debug(repeat_string("{}", sizeof...(args)).c_str(), args...);
}

template<typename... Args>
void Logger::trace(const Args&... args)
{
  console_->trace(repeat_string("{}", sizeof...(args)).c_str(), args...);
}

inline bool Logger::set_async_queue(const std::size_t q_size)
{
  bool p2 = q_size%2 == 0;

  if (p2) spdlog::set_async_mode(q_size);

  return q_size;
}

#define WOLF_STRINGIFY(x) #x
#define WOLF_STR_HELPER(x) WOLF_STRINGIFY(x)

#define WOLF_INFO(...) \
  wolf::internal::Logger::get().info(__VA_ARGS__);

#define WOLF_WARN(...) \
  wolf::internal::Logger::get().warn(__VA_ARGS__);

#define WOLF_ERROR(...) \
  wolf::internal::Logger::get().error(__VA_ARGS__);

#ifdef _WOLF_DEBUG
  #define WOLF_DEBUG(...) \
    wolf::internal::Logger::get().debug(__VA_ARGS__);
#else
  #define WOLF_DEBUG(...)
#endif

#define WOLF_ASYNC_QUEUE_LOG(x) \
  wolf::internal::Logger::get().set_async_queue(x);

#ifdef _WOLF_TRACE
  #define WOLF_TRACE(...) \
    {char this_file[] = __FILE__;\
    wolf::internal::Logger::get().trace("[", basename(this_file), " l#", __LINE__, \
              " : ", __FUNCTION__, "] ", __VA_ARGS__);}
#else
  #define WOLF_TRACE(...)
#endif

} // namespace internal
} // namespace wolf

#endif /* WOLF_LOGGING_H_ */
