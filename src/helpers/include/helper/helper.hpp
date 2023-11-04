#include <string>
#include <sys/time.h>

std::string GetEnv(const std::string &key)
{
    char *val = getenv(key.c_str());
    if (val == NULL)
    {
        return "";
    }
    else
    {
        return std::string(val);
    }
}

uint64_t GetTimeNowMilliSec()
{
    timeval tim;
    gettimeofday(&tim, NULL);
    return 1.0e3 * tim.tv_sec + tim.tv_usec * 1.0e-3;
}

uint64_t GetTimeNowMikroSec()
{
    timeval tim;
    gettimeofday(&tim, NULL);
    return 1.0e6 * tim.tv_sec + tim.tv_usec;
}

uint64_t GetTimeNowSec()
{
    timeval tim;
    gettimeofday(&tim, NULL);
    return tim.tv_sec;
}