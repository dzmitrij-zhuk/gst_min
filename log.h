#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <mutex>
#include <string>
#include <thread>
#include <queue>
#include <condition_variable>

#include "utils.h"

enum class LogLevel {
    INFO,
    WARNING,
    ERROR
};

// Thread-safe queue for log messages
template <typename T>
class ThreadSafeQueue {
public:
    void push(const T& value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(value);
        }
        condition_.notify_one();
    }

    bool pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() { return !queue_.empty() || shutdown_; });
        if (shutdown_ && queue_.empty()) 
            return false;
        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
        }
        condition_.notify_all();
    }

private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool shutdown_ = false;
};

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void log(LogLevel level, const std::string& function, int line, const std::string& message) {
        std::stringstream ss;
        ss << getTimestamp() << " [" << logLevelToString(level) << "] (" << function << ":" << line << "): " 
           << message;
        logQueue_.push(ss.str());
    }

    void setLogFile(const std::string& filename) {
        std::lock_guard<std::mutex> lock(streamMutex_);
        if (fileStream_.is_open()) {
            fileStream_.close();
        }
        fileStream_.open(filename, std::ios::app);
    }

private:
    Logger(){
        loggingThread_ = std::thread([this]() { this->processLogs(); });
    }
    ~Logger() {
        if (fileStream_.is_open()) {
            fileStream_.close();
        }
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void processLogs() {
        std::string logMessage;
        while (logQueue_.pop(logMessage)) {
            std::lock_guard<std::mutex> lock(streamMutex_);
            std::cout << logMessage << std::endl;
            if (fileStream_.is_open()) {
                fileStream_ << logMessage << std::endl;
            }
        }
    }

    std::string logLevelToString(LogLevel level) {
        switch (level) {
            case LogLevel::INFO: return "INFO";
            case LogLevel::WARNING: return "WARNING";
            case LogLevel::ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }

    std::ofstream fileStream_;
    std::mutex streamMutex_;
    std::thread loggingThread_;
    ThreadSafeQueue<std::string> logQueue_;
};

// Macros to simplify logging usage
#define LOG_INFO(msg) Logger::getInstance().log(LogLevel::INFO, __FUNCTION__, __LINE__, msg)
#define LOG_WARNING(msg) Logger::getInstance().log(LogLevel::WARNING, __FUNCTION__, __LINE__, msg)
#define LOG_ERROR(msg) Logger::getInstance().log(LogLevel::ERROR, __FUNCTION__, __LINE__, msg)
