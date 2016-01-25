/*****************************************************************************
 * @Brief     Log configuration for the dji2mav system. Only macros here
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/12/16
 * @Modified  2015/12/16
 *****************************************************************************/

#ifndef _DJI2MAV_LOG_H_
#define _DJI2MAV_LOG_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define _NO_LOG_FUNC(fmt, ...) \
        do{ } while(0)
#define _FATAL_FUNC(fmt, ...) \
        do{ fprintf(stderr, "[dji2mav] [FATAL] " \
        "In file: %s, function: %s, line: %d. Errno: %s. " fmt "\n", \
        __FILE__, __func__, __LINE__, strerror(errno), ## __VA_ARGS__); \
        } while(0)
#define _ERROR_FUNC(fmt, ...) \
        do{ fprintf(stderr, "[dji2mav] [ERROR] " \
        "In file: %s, function: %s, line: %d. Errno: %s. " fmt "\n", \
        __FILE__, __func__, __LINE__, strerror(errno), ## __VA_ARGS__); \
        } while(0)
#define _WARN_FUNC(fmt, ...) \
        do{ fprintf(stdout, "[dji2mav] [WARN] " \
        "In file: %s, function: %s, line: %d. " fmt "\n", \
        __FILE__, __func__, __LINE__, ## __VA_ARGS__); } while(0)
#define _INFO_FUNC(fmt, ...) \
        do{ fprintf(stdout, "[dji2mav] [INFO] " \
        fmt "\n", ## __VA_ARGS__); } while(0)
#define _DEBUG_FUNC(fmt, ...) \
        do{ fprintf(stdout, "[dji2mav] [DEBUG] " \
        "%s, %s, %d: " fmt "\n", \
        __FILE__, __func__, __LINE__, ## __VA_ARGS__); } while(0)
#define _TRACE_FUNC(fmt, ...) \
        do{ fprintf(stdout, "[dji2mav] [TRACE] " \
        "%s, %s, %d: " fmt "\n", \
        __FILE__, __func__, __LINE__, ## __VA_ARGS__); } while(0)

#ifndef DJI2MAV_LOG_OFF
#  define DJI2MAV_FATAL _FATAL_FUNC
#  ifndef DJI2MAV_LOG_FATAL
#    define DJI2MAV_ERROR _ERROR_FUNC
#    ifndef DJI2MAV_LOG_ERROR
#      define DJI2MAV_WARN _WARN_FUNC
#      ifndef DJI2MAV_LOG_WARN
#        define DJI2MAV_INFO _INFO_FUNC
#        ifndef DJI2MAV_LOG_INFO
#          define DJI2MAV_DEBUG _DEBUG_FUNC
#          ifndef DJI2MAV_LOG_DEBUG
#            define DJI2MAV_TRACE _TRACE_FUNC
#          else
#            define DJI2MAV_TRACE _NO_LOG_FUNC
#          endif
#        else
#          define DJI2MAV_DEBUG _NO_LOG_FUNC
#          define DJI2MAV_TRACE _NO_LOG_FUNC
#        endif
#      else
#        define DJI2MAV_INFO _NO_LOG_FUNC
#        define DJI2MAV_DEBUG _NO_LOG_FUNC
#        define DJI2MAV_TRACE _NO_LOG_FUNC
#      endif
#    else
#      define DJI2MAV_WARN _NO_LOG_FUNC
#      define DJI2MAV_INFO _NO_LOG_FUNC
#      define DJI2MAV_DEBUG _NO_LOG_FUNC
#      define DJI2MAV_TRACE _NO_LOG_FUNC
#    endif
#  else
#    define DJI2MAV_ERROR _NO_LOG_FUNC
#    define DJI2MAV_WARN _NO_LOG_FUNC
#    define DJI2MAV_INFO _NO_LOG_FUNC
#    define DJI2MAV_DEBUG _NO_LOG_FUNC
#    define DJI2MAV_TRACE _NO_LOG_FUNC
#  endif
#else
#  define DJI2MAV_FATAL _NO_LOG_FUNC
#  define DJI2MAV_ERROR _NO_LOG_FUNC
#  define DJI2MAV_WARN _NO_LOG_FUNC
#  define DJI2MAV_INFO _NO_LOG_FUNC
#  define DJI2MAV_DEBUG _NO_LOG_FUNC
#  define DJI2MAV_TRACE _NO_LOG_FUNC
#endif


#endif
