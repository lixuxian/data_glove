//
// Timer functions.
//
#ifndef _TIMERS_H
#define _TIMERS_H

#include <stdint.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>

// ------------------------------------------------------------------

typedef int64_t nsecs_t;       // nano-seconds

static inline nsecs_t seconds_to_nanoseconds(nsecs_t secs)
{
    return secs*1000000000;
}

static inline nsecs_t milliseconds_to_nanoseconds(nsecs_t secs)
{
    return secs*1000000;
}

static inline nsecs_t microseconds_to_nanoseconds(nsecs_t secs)
{
    return secs*1000;
}

static inline nsecs_t nanoseconds_to_seconds(nsecs_t secs)
{
    return secs/1000000000;
}

static inline nsecs_t nanoseconds_to_milliseconds(nsecs_t secs)
{
    return secs/1000000;
}

static inline nsecs_t nanoseconds_to_microseconds(nsecs_t secs)
{
    return secs/1000;
}

inline nsecs_t s2ns(nsecs_t v)  {return seconds_to_nanoseconds(v);}
inline nsecs_t ms2ns(nsecs_t v) {return milliseconds_to_nanoseconds(v);}
inline nsecs_t us2ns(nsecs_t v) {return microseconds_to_nanoseconds(v);}
inline nsecs_t ns2s(nsecs_t v)  {return nanoseconds_to_seconds(v);}
inline nsecs_t ns2ms(nsecs_t v) {return nanoseconds_to_milliseconds(v);}
inline nsecs_t ns2us(nsecs_t v) {return nanoseconds_to_microseconds(v);}

inline nsecs_t seconds(nsecs_t v)      { return s2ns(v); }
inline nsecs_t milliseconds(nsecs_t v) { return ms2ns(v); }
inline nsecs_t microseconds(nsecs_t v) { return us2ns(v); }

inline nsecs_t systemTime()
{

    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_REALTIME, &t);
    return nsecs_t(t.tv_sec)*1000000000LL + t.tv_nsec;
}



#endif // _TIMERS_H
