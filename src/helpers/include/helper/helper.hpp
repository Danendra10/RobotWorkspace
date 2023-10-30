#include <string>

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