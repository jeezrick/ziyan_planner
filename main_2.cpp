#include <pathplanner/planner_io.hpp>
#include "pathplanner/logger.hpp"
#include "pathplanner/costmap_manager.hpp"
#include <pathplanner/astar_planner_2d.hpp>
#include <pathplanner/astar_planner_hybrid.hpp>
#include "pathplanner/cfg.hpp"

#include "main_aux.hpp"
#include <assert.h>

using namespace ziyan_planner;

int main() {

    std::string cfg_path = "./data/config.cfg";

    // parse config
    Info::SharedPtr node = std::make_shared<Info>();
    auto configMap = parseConfigFile(cfg_path);
    readConfigFileToInfo(configMap, node);
    std::string data_path = configMap["other.data_path"];

    int x = node->occupancymap_params.width, y = node->occupancymap_params.height;
    int start_x = node->occupancymap_params.start_x, start_y = node->occupancymap_params.start_y;
    int end_x = node->occupancymap_params.end_x, end_y = node->occupancymap_params.end_y;

    std::streamsize buffer_size = x * y * sizeof(uint8_t);
    uint8_t* map_u = new uint8_t[x * y];
    readArray(data_path + configMap["other.map_data_path"], map_u, buffer_size);

    std::shared_ptr<AstarPlanner> planner;
    if (configMap["other.use_hybrid"] == "true") 
    {
        ZIYAN_INFO("makeing palnner hybrid.")
        planner = std::make_shared<AstarPlannerHybrid>(cfg_path);
    } else {
        ZIYAN_INFO("makeing palnner 2d.")
        planner = std::make_shared<AstarPlanner2D>(cfg_path);
    }

    planner->setMap(
        node->occupancymap_params.width, 
        node->occupancymap_params.height, 
        node->occupancymap_params.resolution,
        node->occupancymap_params.origin_x, 
        node->occupancymap_params.origin_y, 
        map_u
    );

    delete[] map_u;

    PoseStamped start, end;
    start.pose.position.x = start_x;
    start.pose.position.y = start_y;
    end.pose.position.x = end_x;
    end.pose.position.y = end_y;
   
    std::stringstream ss;
    ss << "_s(" << start_x << "_" << start_y << ")"
       << "_e(" << end_x << "_" << end_y << ")"
       << "_r(" <<node->plannerhybrid_params.minimum_turning_radius << ")";
    std::string path_suffix = ss.str();
    std::cout << path_suffix << std::endl;

    auto dummy_cancel_checker = []() {
    return false;
    };

    Path path;
    path = planner->createPlan(start, end, dummy_cancel_checker);

    planner->cleanup();

    planner.reset();

    {
        ZIYAN_INFO("Path size: %d", path.poses.size());
        unsigned int* out = new unsigned int[path.poses.size() * 2];
        int iidx = 0;
        for (const XYT& coord : path.xyt_vec) {
            out[iidx++] = coord.x;
            out[iidx++] = coord.y;
        }

        std::string file_name = data_path + "/out_path" + (configMap["other.use_hybrid"] == "true" ? "_hybrid" : "_2d") + path_suffix + ".bin";
        saveArray(out, path.xyt_vec.size() * 2, file_name);
        delete[] out;
    }

    return 0;
}