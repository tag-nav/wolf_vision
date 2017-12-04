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
namespace do_not_enter_where_the_wolf_lives {

#define __INTERNAL_WOLF_MAIN_LOGGER_NAME_ "wolf_main_console"

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
                                                   "{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}{}"); // up to 25 args. Should be fine
class Logger
{
public:

  Logger(const std::string& name);

  Logger(std::string&& name);

  ~Logger();

  // Not copyable/movable
  Logger(Logger&)          = delete;
  void operator=(Logger&)  = delete;
  Logger(Logger&&)         = delete;
  void operator=(Logger&&) = delete;

  template<typename... Args>
  void info(Args&&... args) const;

  template<typename... Args>
  void warn(Args&&... args) const;

  template<typename... Args>
  void error(Args&&... args) const;

  template<typename... Args>
  void debug(Args&&... args) const;

  template<typename... Args>
  void trace(Args&&... args) const;

  bool set_async_queue(const std::size_t q_size);

  void set_pattern(const std::string& p);

protected:

  const std::string log_name_;

  std::shared_ptr<spdlog::logger> console_;
};

inline Logger::Logger(const std::string& name) :
  log_name_(name)
{
  // Create main logger
  console_ = spdlog::stdout_color_mt(log_name_);

#ifdef _WOLF_TRACE
  console_->set_level(spdlog::level::trace);
#endif

  // Enable asynchronous logging
  // Queue size must be a power of 2
  spdlog::set_async_mode(4096);

  if (log_name_ == __INTERNAL_WOLF_MAIN_LOGGER_NAME_)
    // Logging pattern is :
    // [thread num][hour:minutes:seconds.nanoseconds][log type] #log-content
    //set_pattern("[%t][%H:%M:%S.%F][%l] %v");
    // [log type][MM/DD/YY - hour:minutes:seconds.nanoseconds] #log-content
//    set_pattern("[%l][%x - %H:%M:%S.%F] %v");
      set_pattern("[%l][%H:%M:%S] %v");
  else
    // Logging pattern is :
    // [logger name][thread num][hour:minutes:seconds.nanoseconds][log type] #log-content
    //set_pattern("[" + log_name_ + "]" +"[%t][%H:%M:%S.%F][%l] %v");
    // [log type][MM/DD/YY - hour:minutes:seconds.nanoseconds][logger name] #log-content
    set_pattern("[%l][%x - %H:%M:%S.%F][" + log_name_ + "] %v");
}

inline Logger::Logger(std::string&& name) :
  log_name_(std::forward<std::string>(name))
{
  // Create main logger
  console_ = spdlog::stdout_color_mt(log_name_);

#ifdef _WOLF_TRACE
  console_->set_level(spdlog::level::trace);
#endif

  // Enable asynchronous logging
  // Queue size must be a power of 2
  spdlog::set_async_mode(4096);

  if (log_name_ == __INTERNAL_WOLF_MAIN_LOGGER_NAME_)
    // Logging pattern is :
    // [thread num][hour:minutes:seconds.nanoseconds][log type] #log-content
    //set_pattern("[%t][%H:%M:%S.%F][%l] %v");
    // [log type][MM/DD/YY - hour:minutes:seconds.nanoseconds] #log-content
//    set_pattern("[%l][%x - %H:%M:%S.%F] %v");
      set_pattern("[%l][%H:%M:%S] %v");
  else
    // Logging pattern is :
    // [logger name][thread num][hour:minutes:seconds.nanoseconds][log type] #log-content
    //set_pattern("[" + log_name_ + "]" +"[%t][%H:%M:%S.%F][%l] %v");
    // [log type][MM/DD/YY - hour:minutes:seconds.nanoseconds][logger name] #log-content
    set_pattern("[%l][%x - %H:%M:%S.%F][" + log_name_ + "] %v");
}

inline Logger::~Logger()
{
  spdlog::drop(log_name_);
}

template<typename... Args>
void Logger::info(Args&&... args) const
{
  console_->info(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::warn(Args&&... args) const
{
  console_->warn(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::error(Args&&... args) const
{
  console_->error(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::debug(Args&&... args) const
{
  console_->debug(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::trace(Args&&... args) const
{
  console_->trace(std::get<sizeof...(args)-1>(repeated_brace), std::forward<Args>(args)...);
}

inline bool Logger::set_async_queue(const std::size_t q_size)
{
  bool p2 = q_size%2 == 0;

  if (p2) spdlog::set_async_mode(q_size);

  return q_size;
}

inline void Logger::set_pattern(const std::string& p)
{
  console_->set_pattern(p);
}

using LoggerPtr = std::unique_ptr<Logger>;

/// dummy namespace to avoid colision with c++14
/// @todo use std version once we move to cxx14
namespace not_std {
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
} /* namespace not_std */

class LoggerManager
{
public:

  LoggerManager()  = default;
  ~LoggerManager() = default;

  bool exists(const std::string& name) const
  {
    std::lock_guard<std::mutex> lock(mut_);
    return existsImpl(name);
  }

  const Logger& getLogger(const std::string& name) /*const*/
  {
    std::lock_guard<std::mutex> lock(mut_);

    if (!existsImpl(name)) addLogger(name);

    return *(logger_map_.at(name));
  }

protected:

  mutable std::mutex mut_;

  std::map<const std::string, const LoggerPtr> logger_map_;

  bool addLogger(const std::string& name)
  {
    /// @note would be easier with cpp17 map.try_emplace...
    const bool created = existsImpl(name) ?
                          false :
                          logger_map_.emplace(name, not_std::make_unique<Logger>(name)).second;
    return created;
  }

  bool existsImpl(const std::string& name) const
  {
    return (logger_map_.find(name) != logger_map_.end());
    //return (spdlog::get(name) != nullptr);
  }
};

} /* namespace do_not_enter_where_the_wolf_lives */

using WolfLogger = Singleton<do_not_enter_where_the_wolf_lives::Logger>;
using WolfLoggerManager = Singleton<do_not_enter_where_the_wolf_lives::LoggerManager>;

} /* namespace internal */
} /* namespace wolf */

#define WOLF_STRINGIFY(x) #x
#define WOLF_STR_HELPER(x) WOLF_STRINGIFY(x)

/// @brief NAMED LOGGING

#define WOLF_ASYNC_QUEUE_LOG_NAMED(name, ...) wolf::internal::WolfLoggerManager::get().getLogger(name).set_async_queue(x);

#define WOLF_INFO_NAMED(name, ...) wolf::internal::WolfLoggerManager::get().getLogger(name).info(__VA_ARGS__);
#define WOLF_INFO_NAMED_COND(name, cond, ...) if (cond) WOLF_INFO_NAMED(name, __VA_ARGS__);

#define WOLF_WARN_NAMED(name, ...) wolf::internal::WolfLoggerManager::get().getLogger(name).warn(__VA_ARGS__);
#define WOLF_WARN_NAMED_COND(name, cond, ...) if (cond) WOLF_WARN_NAMED(name, __VA_ARGS__);

#define WOLF_ERROR_NAMED(name, ...) wolf::internal::WolfLoggerManager::get().getLogger(name).error(__VA_ARGS__);
#define WOLF_ERROR_NAMED_COND(name, cond, ...) if (cond) WOLF_ERROR_NAMED(name, __VA_ARGS__);

#ifdef _WOLF_DEBUG
  #define WOLF_DEBUG_NAMED(name, ...) wolf::internal::WolfLoggerManager::get().getLogger(name).debug(__VA_ARGS__);
  #define WOLF_DEBUG_NAMED_COND(name, cond, ...) if (cond) WOLF_DEBUG_NAMED(name, __VA_ARGS__);
#else
  #define WOLF_DEBUG_NAMED(name, ...)
  #define WOLF_DEBUG_NAMED_COND(cond, name, ...)
#endif

#ifdef _WOLF_TRACE
  #define WOLF_TRACE_NAMED(name, ...) \
    {char this_file[] = __FILE__;\
    wolf::internal::WolfLoggerManager::get().getLogger(name).trace("[", basename(this_file), " L", __LINE__, \
              " : ", __FUNCTION__, "] ", __VA_ARGS__);}
  #define WOLF_TRACE_NAMED_COND(name, cond, ...) if (cond) WOLF_TRACE_NAMED_COND(name, __VA_ARGS__);
#else
  #define WOLF_TRACE_NAMED(...)
  #define WOLF_TRACE_NAMED_cond(name, cond, ...)
#endif

/// @brief MAIN LOGGING

#define WOLF_ASYNC_QUEUE_LOG(x) wolf::internal::WolfLogger::get(__INTERNAL_WOLF_MAIN_LOGGER_NAME_).set_async_queue(x);

#define WOLF_INFO(...) wolf::internal::WolfLogger::get(__INTERNAL_WOLF_MAIN_LOGGER_NAME_).info(__VA_ARGS__);
#define WOLF_INFO_COND(cond, ...) if (cond) WOLF_INFO(__VA_ARGS__);

#define WOLF_WARN(...) wolf::internal::WolfLogger::get(__INTERNAL_WOLF_MAIN_LOGGER_NAME_).warn(__VA_ARGS__);
#define WOLF_WARN_COND(cond, ...) if (cond) WOLF_WARN(__VA_ARGS__);

#define WOLF_ERROR(...) wolf::internal::WolfLogger::get(__INTERNAL_WOLF_MAIN_LOGGER_NAME_).error(__VA_ARGS__);
#define WOLF_ERROR_COND(cond, ...) if (cond) WOLF_ERROR(__VA_ARGS__);

#ifdef _WOLF_DEBUG
  #define WOLF_DEBUG(...) wolf::internal::WolfLogger::get(__INTERNAL_WOLF_MAIN_LOGGER_NAME_).debug(__VA_ARGS__);
  #define WOLF_DEBUG_COND(cond, ...) if (cond) WOLF_DEBUG(__VA_ARGS__);
#else
  #define WOLF_DEBUG(...)
  #define WOLF_DEBUG_COND(cond, ...)
#endif

#ifdef _WOLF_TRACE
  #define WOLF_TRACE(...) \
    {char this_file[] = __FILE__;\
    wolf::internal::WolfLogger::get(__INTERNAL_WOLF_MAIN_LOGGER_NAME_).trace("[", basename(this_file), " L", __LINE__, \
              " : ", __FUNCTION__, "] ", __VA_ARGS__);}
  #define WOLF_TRACE_COND(cond, ...) if (cond) WOLF_TRACE(__VA_ARGS__);
#else
  #define WOLF_TRACE(...)
  #define WOLF_TRACE_COND(cond, ...)
#endif

#endif /* WOLF_LOGGING_H_ */
