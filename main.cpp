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
    // parse config
    Info::SharedPtr node = std::make_shared<Info>();
    auto configMap = parseConfigFile("./data/config.cfg");
    readConfigFileToInfo(configMap, node);
    std::string data_path = configMap["other.data_path"];

    int x = node->occupancymap_params.width, y = node->occupancymap_params.height;
    int start_x = node->occupancymap_params.start_x, start_y = node->occupancymap_params.start_y;
    int end_x = node->occupancymap_params.end_x, end_y = node->occupancymap_params.end_y;

    std::streamsize buffer_size = x * y * sizeof(uint8_t);
    uint8_t* map_u = new uint8_t[x * y];
    try
    {
        readArray(data_path + configMap["other.map_data_path"], map_u, buffer_size);
        // for (size_t i = 0; i < x*y; ++i) {
        //     uint8_t value = map_u[i]; 
        //     if (value == 1) {
        //         value = 254; // 将值为 1 的元素修改为 100
        //     }
        //     map_u[i] = value; // 将修改后的值存回数组中
        // }
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
    }

    std::shared_ptr<CostmapManager> costmap_ziyan = std::make_shared<CostmapManager>(
        node,    
        node->occupancymap_params.width, 
        node->occupancymap_params.height, 
        node->occupancymap_params.resolution,
        node->occupancymap_params.origin_x, 
        node->occupancymap_params.origin_y, 
        map_u
    );
    delete[] map_u;

    Costmap2D* costmap_ptr = costmap_ziyan->getCostmapPtr();
    costmap_ptr -> saveMap(data_path + configMap["other.map_after_inflation_path"]);

    PoseStamped start, end;
    start.pose.position.x = start_x;
    start.pose.position.y = start_y;
    end.pose.position.x = end_x;
    end.pose.position.y = end_y;

    // costmap_ptr->mapToWorld(start_x, start_y, start.pose.position.x, start.pose.position.y);
    // costmap_ptr->mapToWorld(end_x, end_y, end.pose.position.x, end.pose.position.y);

    unsigned int ref_x_s, ref_y_s, ref_x_e, ref_y_e;
    costmap_ptr->worldToMap(start.pose.position.x, start.pose.position.y, ref_x_s, ref_y_s);
    costmap_ptr->worldToMap(end.pose.position.x, end.pose.position.y, ref_x_e, ref_y_e);
    std::cout << "ref_x_s: " << ref_x_s << ", ref_y_s: " << ref_y_s << std::endl;   
    std::cout << "ref_x_e: " << ref_x_e << ", ref_y_e: " << ref_y_e << std::endl;
    
    std::stringstream ss;
    ss << "_s(" << start_x << "_" << start_y << ")"
       << "_e(" << end_x << "_" << end_y << ")"
       << "_r(" <<node->plannerhybrid_params.minimum_turning_radius << ")";
    std::string path_suffix = ss.str();
    std::cout << path_suffix << std::endl;
    
    // plan astar 2d
    {
        auto planner_2d = std::make_unique<AstarPlanner2D>(node);
        planner_2d->setMap(costmap_ziyan);

        auto dummy_cancel_checker = []() {
        return false;
        };

        Path path;
        path = planner_2d->createPlan(start, end, dummy_cancel_checker);

        planner_2d->cleanup();
        planner_2d.reset();

        ZIYAN_INFO("Path size: %d", path.poses.size());
        unsigned int* out = new unsigned int[path.poses.size() * 2];
        int iidx = 0;
        for (const XYT& coord : path.xyt_vec) {
            out[iidx++] = coord.x;
            out[iidx++] = coord.y;
        }

        std::string file_name = data_path + "/out_path_2d" + path_suffix + ".bin";
        saveArray(out, path.xyt_vec.size() * 2, file_name);
        delete[] out;
    }

    // plan astar hybrid
    {
        auto planner = std::make_unique<AstarPlannerHybrid>(node);
        planner->setMap(costmap_ziyan);

        auto dummy_cancel_checker = []() {return false;};

        Path path;
        path = planner->createPlan(start, end, dummy_cancel_checker);

        planner->cleanup();

        planner.reset();

        {
            ZIYAN_INFO("Path size: %d", path.xyt_vec.size());
            unsigned int* out = new unsigned int[path.xyt_vec.size() * 2];
            int iidx = 0;
            for (const XYT& coord : path.xyt_vec) {
                // std::cout << "x: " << coord.x << ", y: " << coord.y << std::endl;
                out[iidx++] = coord.x;
                out[iidx++] = coord.y;
            }

            std::string file_name = data_path + "/out_path_hybrid" + path_suffix + ".bin";
            saveArray(out, path.xyt_vec.size() * 2, file_name);
            delete[] out;
        }

        {
            double* poseout = new double[path.poses.size() * 2];
            int iidx = 0;
            for (const PoseStamped& pose : path.poses) {
                // std::cout << "x: " << pose.pose.position.x << ", y: " << pose.pose.position.y << std::endl;
                poseout[iidx++] = pose.pose.position.x;
                poseout[iidx++] = pose.pose.position.y;
            }

            std::string file_name = data_path + "/out_path_hybrid_world" + path_suffix + ".bin";
            saveArray(poseout, path.poses.size() * 2, file_name);
            delete[] poseout;
        }
    }

    return 0;
}