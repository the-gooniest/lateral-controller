#ifndef LATERALROSNODEEXCEPTION_H
#define LATERALROSNODEEXCEPTION_H

#include <exception>
#include <string>

class LateralNodeException : public std::exception {
    std::string error_message;
public:
    explicit LateralNodeException(const std::string& message) : error_message(message) {}
    virtual ~LateralNodeException() throw() {}
    virtual const char* what() const throw() {
        return error_message.c_str();
    }
};

#endif

