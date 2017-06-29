#ifndef PARSER_HPP
#define PARSER_HPP
#include "Parcour.hpp"
#include "Robot.hpp"
#include <string>

class Parser
{

public:
    Parser();
    
    Parcour *getParcour(std::string &path);
    
    std::vector<StepInfo> getStepInfo(std::string &path);
};

#endif // PARSER_HPP
