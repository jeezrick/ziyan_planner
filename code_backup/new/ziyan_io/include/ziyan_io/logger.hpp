#ifndef ZIYAN_IO__ZIYAN_PLANNER_LOGGER_HPP_
#define ZIYAN_IO__ZIYAN_PLANNER_LOGGER_HPP_

#include <iostream>

#define ZIYAN_DEBUG
// #undef ZIYAN_DEBUG

#ifdef ZIYAN_DEBUG
#define ZIYAN_ASSERT(exp)                                             \
        do {                                                                  \
          if (!(exp)) {                                                       \
            fprintf(stderr, "[ASSERT!][%s:%d]: %s\n",                     \
              __FUNCTION__, __LINE__);                  \
            abort();                                                          \
          }                                                                   \
        } while (0)

#define ZIYAN_INFO(...)                                               \
        {                                                                     \
          fprintf(stdout, "[INFO][%s:%d] ",                               \
            __FUNCTION__, __LINE__);                    \
          fprintf(stdout, __VA_ARGS__);                                       \
          fprintf(stdout, "\n");                                              \
        }

#define ZIYAN_ERROR(...)                                              \
        {                                                                     \
          fprintf(stdout, "[ERROR!][%s:%d] ",                             \
            __FUNCTION__, __LINE__);                    \
          fprintf(stdout, __VA_ARGS__);                                       \
          fprintf(stdout, "\n");                                              \
          abort();                                                            \
        }                                               
#else
#define ZIYAN_ASSERT( exp)
#define ZIYAN_INFO(...)
#define ZIYAN_ERROR(...)
#endif // ifdef

#endif // ZIYAN_IO__ZIYAN_PLANNER_LOGGER_HPP_