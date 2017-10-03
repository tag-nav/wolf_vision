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

static const auto repeated_brace = std::make_tuple("{}",
                                                   "{}{}",
                                                   "{}{}{}",
                                                   "{}{}{}{}",
                                                   "{}{}{}{}{}",
                                                   "{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}"); // up to 25 args.

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
  void info(Args&&... args);

  template<typename... Args>
  void warn(Args&&... args);

  template<typename... Args>
  void error(Args&&... args);

  template<typename... Args>
  void debug(Args&&... args);

  template<typename... Args>
  void trace(Args&&... args);

  bool set_async_queue(const std::size_t q_size);

protected:

  const std::string log_name_ = "wolf_main_console";

  std::shared_ptr<spdlog::logger> console_;
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
void Logger::info(Args&&... args)
{
  console_->info(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::warn(Args&&... args)
{
  console_->warn(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::error(Args&&... args)
{
  console_->error(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::debug(Args&&... args)
{
  console_->debug(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::trace(Args&&... args)
{
  console_->trace(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
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
    wolf::internal::Logger::get().trace("[", basename(this_file), " L", __LINE__, \
              " : ", __FUNCTION__, "] ", __VA_ARGS__);}
#else
  #define WOLF_TRACE(...)
#endif

} // namespace internal
} // namespace wolf

#endif /* WOLF_LOGGING_H_ */
