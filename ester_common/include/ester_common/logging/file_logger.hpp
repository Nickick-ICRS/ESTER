#ifndef __FILE_LOGGER_HPP__
#define __FILE_LOGGER_HPP__

#include <string>
#include <memory>
#include <vector>
#include <map>

namespace ester_common::logging
{

class FileLogger;

class FileLoggerBuilder {
public:
    static FileLoggerBuilder begin();

    FileLoggerBuilder &filename(const std::string &s);
    FileLoggerBuilder &header(const std::string &s);
    FileLoggerBuilder &new_field(const std::string &name);
    FileLoggerBuilder &starting_timestamp(double t);

    std::unique_ptr<FileLogger> build();

private:
    std::string filename_;
    std::string header_;
    std::vector<std::string> fields_;
    double initial_timestamp_;
};

class FileLogger {
public:
    ~FileLogger();

    bool write_field(const std::string &name, double value, double timestamp);

private:
    FileLogger(
        const std::string &filename, const std::string &header,
        double timestamp);
    
    bool register_field(const std::string &name);

    void write_field_headings();

    void write_to_file(const std::string &data);

    void write_current_data();

    std::string file_name_;

    std::map<std::string, double> field_values_;

    double current_timestamp_;

    friend class FileLoggerBuilder;
};

} // namespace ester_common::logging

#endif // __FILE_LOGGER_HPP__