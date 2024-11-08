#include "logging_levels.h"

#ifndef LIBRARY_LOG_NAME
  #define LIBRARY_LOG_NAME    "APP"
#endif
#ifndef LIBRARY_LOG_LEVEL
  #define LIBRARY_LOG_LEVEL LOG_DEBUG
#endif

#if !defined( LOG_METADATA_FORMAT ) && !defined( LOG_METADATA_ARGS )
  #define LOG_METADATA_FORMAT "[%s:%d]"
  #define LOG_METADATA_ARGS __FILE__, __LINE__
#endif

#include "logging_stack.h"
