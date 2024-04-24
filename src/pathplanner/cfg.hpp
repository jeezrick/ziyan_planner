#ifndef ZIYAN_PLANNER__CFG_HPP_
#define ZIYAN_PLANNER__CFG_HPP_

#include <string>
#include <unordered_map>

#include "pathplanner/planner_io.hpp"


namespace ziyan_planner
{

std::unordered_map<std::string, std::string> parseConfigFile(const std::string& filename);

void readConfigFileToInfo(
    std::unordered_map<std::string, std::string> & configMap, 
    std::shared_ptr<ziyan_planner::Info> & node
);

}

#endif