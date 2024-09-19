#include <ester_common/logging/file_logger.hpp>
#include <sstream>
#include <fstream>
#include <iostream>

#include <ctime>
#include <chrono>

namespace ester_common::logging
{

FileLoggerBuilder FileLoggerBuilder::begin() {
    FileLoggerBuilder flb;
    flb.filename_ = "";
    flb.header_ = "";
    flb.initial_timestamp_ = 0;
    return flb;
}

FileLoggerBuilder &FileLoggerBuilder::filename(const std::string &s) {
    std::string filetype = ".csv";
    const auto now = std::chrono::system_clock::now();
    time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    tm *now_tm = localtime(&now_time_t);
    char time_str[25];
    strftime(time_str, 25, "_%Y-%m-%d-%H-%M-%OS", now_tm);
    std::string dir = std::string(getenv("HOME")) + "/mpc_comparisons/";
    filename_ = dir + s + time_str + filetype;
    return *this;
}

FileLoggerBuilder &FileLoggerBuilder::header(const std::string &s) {
    header_ = s;
    return *this;
}

FileLoggerBuilder &FileLoggerBuilder::new_field(const std::string &name) {
    fields_.push_back(name);
    return *this;
}

FileLoggerBuilder &FileLoggerBuilder::starting_timestamp(double t) {
    initial_timestamp_ = t;
    return *this;
}

std::unique_ptr<FileLogger> FileLoggerBuilder::build() {
    std::unique_ptr<FileLogger> logger;
    logger.reset(new FileLogger(filename_, header_, initial_timestamp_));
    for (const auto &field : fields_) {
        if (!logger->register_field(field)) {
            throw std::runtime_error("Field already registered " + field);
        }
    }
    logger->write_field_headings();
    return logger;
}


FileLogger::FileLogger(
    const std::string &filename, const std::string &header, double timestamp)
    :file_name_(filename), current_timestamp_(timestamp)
{
    std::cerr << "Logging to " << file_name_ << std::endl;
    std::ofstream of(file_name_, std::ofstream::out | std::ofstream::ate);
    if (!of) {
        std::cerr << "Failed to open " << file_name_ << std::endl;
        throw std::runtime_error("Failed to open " + file_name_);
    }
    write_to_file(header);
}

FileLogger::~FileLogger() {
    write_current_data();
}

bool FileLogger::register_field(const std::string &name) {
    if (field_values_.count(name))
    {
        return false;
    }
    field_values_[name] = 0;
    return true;
}

void FileLogger::write_field_headings() {
    std::stringstream to_write;
    to_write << "timestamp,";
    for (const auto &pair : field_values_) {
        to_write << pair.first << ",";
    }
    to_write << std::endl;
    write_to_file(to_write.str());
}

bool FileLogger::write_field(const std::string &name, double value, double timestamp)
{
    if (!field_values_.count(name)) {
        return false;
    }
    if (timestamp != current_timestamp_) {
        write_current_data();
        current_timestamp_ = timestamp;
    }
    field_values_[name] = value;
    return true;
}

void FileLogger::write_current_data() {
    std::stringstream to_write;

    to_write << current_timestamp_ << ",";
    for (const auto &pair : field_values_) {
        to_write << pair.second << ",";
    }
    to_write << std::endl;
    write_to_file(to_write.str());
}

void FileLogger::write_to_file(const std::string &s) {
    std::ofstream of(file_name_, std::ofstream::out | std::ofstream::app);
    of << s;
}

} // namespace ester_common::logging