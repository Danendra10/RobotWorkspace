#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <string>
#include <fstream>
#include <iostream>
#include <cstdarg>
#include <ctime>

#if defined(__has_include) && __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
#include <cstring>
namespace fs = std::experimental::filesystem;
#elif defined(__has_include) && __has_include(<filesystem>)
#include <filesystem>
#include <cstring>
namespace fs = std::__fs::filesystem;
#else
#include <filesystem>
namespace fs = std::__fs::filesystem;
#endif

namespace logger
{

    const std::string RED = "\033[0;31m";
    const std::string GREEN = "\033[0;32m";
    const std::string YELLOW = "\033[0;33m";
    const std::string BLUE = "\033[0;34m";
    const std::string MAGENTA = "\033[0;35m";
    const std::string CYAN = "\033[0;36m";
    const std::string WHITE = "\033[0;37m";
    const std::string RESET = "\033[0m";

    class Logger
    {
    private:
        std::string log_dir;

        void SaveLog(const char *msg, va_list args)
        {
            char buffer[200];
            std::time_t now = std::time(0);
            std::tm *ltm = std::localtime(&now);

            vsnprintf(buffer, sizeof(buffer), msg, args);
            strcat(buffer, "\n");

            std::ofstream file(log_dir, std::ios_base::app);
            if (file.is_open())
            {
                file << buffer;
            }
            else
            {
                std::cerr << "Failed to open log file." << std::endl;
            }
        }

    public:
        Logger()
        {
            fs::path log_directory = fs::current_path() / "../log";
            fs::path log_path = log_directory / "logger.log";

            if (!fs::exists(log_directory))
            {
                fs::create_directories(log_directory);
            }

            log_dir = log_path.string();
        }

        void Log(const std::string color, const char *msg, ...)
        {
            char timestampBuffer[100];
            std::time_t now = std::time(0);
            std::tm *ltm = std::localtime(&now);
            snprintf(timestampBuffer, sizeof(timestampBuffer), "[%d-%d-%d %d:%d:%d] => ",
                     1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday,
                     ltm->tm_hour, ltm->tm_min, ltm->tm_sec);

            char msgBuffer[200];
            va_list args;
            va_start(args, msg);
            vsnprintf(msgBuffer, sizeof(msgBuffer), msg, args);
            va_end(args);

            SaveLog(msgBuffer, args);

            std::cout << color;
            std::vprintf(msgBuffer, args);
            std::cout << RESET << std::endl;
        }
    };

} // namespace logger

#endif // LOGGER_HPP_
