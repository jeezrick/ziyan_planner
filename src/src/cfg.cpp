#include <iostream>
#include <fstream>
#include <sstream>

#include "pathplanner/cfg.hpp"

namespace ziyan_planner
{

std::unordered_map<std::string, std::string> parseConfigFile(const std::string& filename) {
    std::unordered_map<std::string, std::string> configMap;
    std::ifstream file(filename);
    std::string line;
    std::string currentSection;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') // Skip empty lines and comments
            continue;

        if (line[0] == '[') {
            currentSection = line.substr(1, line.find(']') - 1);
            continue;
        }

        std::istringstream lineStream(line);
        std::string key, value;
        std::getline(lineStream, key, '=');
        std::getline(lineStream, value);

        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);

        configMap[currentSection + "." + key] = value;
    }

    for (const auto& pair : configMap) {
        std::cout << "Parameter: " << pair.first << ", Value: " << pair.second << std::endl;
    }

    return configMap;
}


void readConfigFileToInfo(
    std::unordered_map<std::string, std::string> & configMap, 
    std::shared_ptr<ziyan_planner::Info> & node) 
{
    try {
        // Smoother parameters
        node->smoother.tolerance = std::stod(configMap["smoother.tolerance"]);
        node->smoother.max_iterations = std::stoi(configMap["smoother.max_iterations"]);
        node->smoother.w_data = std::stod(configMap["smoother.w_data"]);
        node->smoother.w_smooth = std::stod(configMap["smoother.w_smooth"]);
        node->smoother.do_refinement = configMap["smoother.do_refinement"] == "true";
        node->smoother.refinement_num = std::stoi(configMap["smoother.refinement_num"]);

        // Planner2D parameters
        node->planner2d_params.tolerance = std::stof(configMap["planner2d_params.tolerance"]);
        node->planner2d_params.downsample_costmap = configMap["planner2d_params.downsample_costmap"] == "true";
        node->planner2d_params.downsampling_factor = std::stoi(configMap["planner2d_params.downsampling_factor"]);
        node->planner2d_params.cost_travel_multiplier = std::stof(configMap["planner2d_params.cost_travel_multiplier"]);
        node->planner2d_params.allow_unknown = configMap["planner2d_params.allow_unknown"] == "true";
        node->planner2d_params.max_iterations = std::stoi(configMap["planner2d_params.max_iterations"]);
        node->planner2d_params.max_on_approach_iterations = std::stoi(configMap["planner2d_params.max_on_approach_iterations"]);
        node->planner2d_params.terminal_checking_interval = std::stoi(configMap["planner2d_params.terminal_checking_interval"]);
        node->planner2d_params.use_final_approach_orientation = configMap["planner2d_params.use_final_approach_orientation"] == "true";
        node->planner2d_params.max_planning_time = std::stod(configMap["planner2d_params.max_planning_time"]);

        // PlannerHybrid parameters
        node->plannerhybrid_params.angle_quantization_bins = std::stoi(configMap["plannerhybrid_params.angle_quantization_bins"]);
        node->plannerhybrid_params.smooth_path = configMap["plannerhybrid_params.smooth_path"] == "true";
        node->plannerhybrid_params.minimum_turning_radius = std::stod(configMap["plannerhybrid_params.minimum_turning_radius"]);
        node->plannerhybrid_params.lookup_table_size = std::stod(configMap["plannerhybrid_params.lookup_table_size"]);
        node->plannerhybrid_params._debug_visualizations = configMap["plannerhybrid_params._debug_visualizations"] == "true";
        node->plannerhybrid_params.motion_model_for_search = configMap["plannerhybrid_params.motion_model_for_search"];
        node->plannerhybrid_params.allow_primitive_interpolation = configMap["plannerhybrid_params.allow_primitive_interpolation"] == "true";
        node->plannerhybrid_params.cache_obstacle_heuristic = configMap["plannerhybrid_params.cache_obstacle_heuristic"] == "true";
        node->plannerhybrid_params.reverse_penalty = std::stof(configMap["plannerhybrid_params.reverse_penalty"]);
        node->plannerhybrid_params.change_penalty = std::stof(configMap["plannerhybrid_params.change_penalty"]);
        node->plannerhybrid_params.non_straight_penalty = std::stof(configMap["plannerhybrid_params.non_straight_penalty"]);
        node->plannerhybrid_params.cost_penalty = std::stof(configMap["plannerhybrid_params.cost_penalty"]);
        node->plannerhybrid_params.retrospective_penalty = std::stof(configMap["plannerhybrid_params.retrospective_penalty"]);
        node->plannerhybrid_params.analytic_expansion_ratio = std::stof(configMap["plannerhybrid_params.analytic_expansion_ratio"]);
        node->plannerhybrid_params.analytic_expansion_max_cost = std::stof(configMap["plannerhybrid_params.analytic_expansion_max_cost"]);
        node->plannerhybrid_params.analytic_expansion_max_cost_override = configMap["plannerhybrid_params.analytic_expansion_max_cost_override"] == "true";
        node->plannerhybrid_params.use_quadratic_cost_penalty = configMap["plannerhybrid_params.use_quadratic_cost_penalty"] == "true";
        node->plannerhybrid_params.downsample_obstacle_heuristic = configMap["plannerhybrid_params.downsample_obstacle_heuristic"] == "true";
        node->plannerhybrid_params.analytic_expansion_max_length = std::stof(configMap["plannerhybrid_params.analytic_expansion_max_length"]);
        node->plannerhybrid_params.tolerance = std::stof(configMap["plannerhybrid_params.tolerance"]);
        node->plannerhybrid_params.downsample_costmap = configMap["plannerhybrid_params.downsample_costmap"] == "true";
        node->plannerhybrid_params.downsampling_factor = std::stoi(configMap["plannerhybrid_params.downsampling_factor"]);
        node->plannerhybrid_params.cost_travel_multiplier = std::stof(configMap["plannerhybrid_params.cost_travel_multiplier"]);
        node->plannerhybrid_params.allow_unknown = configMap["plannerhybrid_params.allow_unknown"] == "true";
        node->plannerhybrid_params.max_iterations = std::stoi(configMap["plannerhybrid_params.max_iterations"]);
        node->plannerhybrid_params.max_on_approach_iterations = std::stoi(configMap["plannerhybrid_params.max_on_approach_iterations"]);
        node->plannerhybrid_params.terminal_checking_interval = std::stoi(configMap["plannerhybrid_params.terminal_checking_interval"]);
        node->plannerhybrid_params.max_planning_time = std::stod(configMap["plannerhybrid_params.max_planning_time"]);

        // Inflation parameters
        node->inflation_params.enabled = configMap["inflation_params.enabled"] == "true";
        node->inflation_params.inflation_radius = std::stod(configMap["inflation_params.inflation_radius"]);
        node->inflation_params.cost_scaling_factor = std::stod(configMap["inflation_params.cost_scaling_factor"]);
        node->inflation_params.inflate_unknown = configMap["inflation_params.inflate_unknown"] == "true";
        node->inflation_params.inflate_around_unknown = configMap["inflation_params.inflate_around_unknown"] == "true";
        node->inflation_params.radius = std::stod(configMap["inflation_params.radius"]);
        node->inflation_params.footprint_padding = std::stof(configMap["inflation_params.footprint_padding"]);
        node->inflation_params.save_inflated_map = configMap["inflation_params.save_inflated_map"] == "true";
        node->inflation_params.save_path = configMap["inflation_params.save_path"];

        // OccupancyMap parameters
        node->occupancymap_params.resolution = std::stod(configMap["occupancymap_params.resolution"]);
        node->occupancymap_params.width = std::stoi(configMap["occupancymap_params.width"]);
        node->occupancymap_params.height = std::stoi(configMap["occupancymap_params.height"]);
        node->occupancymap_params.origin_x = std::stod(configMap["occupancymap_params.origin_x"]);
        node->occupancymap_params.origin_y = std::stod(configMap["occupancymap_params.origin_y"]);
        node->occupancymap_params.start_x = std::stoi(configMap["occupancymap_params.start_x"]);
        node->occupancymap_params.start_y = std::stoi(configMap["occupancymap_params.start_y"]);
        node->occupancymap_params.end_x = std::stoi(configMap["occupancymap_params.end_x"]);
        node->occupancymap_params.end_y = std::stoi(configMap["occupancymap_params.end_y"]);

    } catch(const std::invalid_argument& e) {
        std::cerr << "Invalid argument: " << e.what() << std::endl;
    } catch(const std::out_of_range& e) {
        std::cerr << "Out of range: " << e.what() << std::endl;
    }
}

}