#ifndef IPOLECART_CONTROLLER_HPP
#define IPOLECART_CONTROLLER_HPP

#include <string>

class IPolecart_Controller
{
    public:
        virtual void init(std::string input_topic, std::string output_topic) = 0;
        virtual void start() = 0;
};

#endif