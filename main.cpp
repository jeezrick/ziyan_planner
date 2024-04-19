#include <ziyan_io/ziyan_io.hpp>
#include "ziyan_io/logger.hpp"
#include <map/costmap_2d.hpp>
#include "map/inflation_layer.hpp"
#include "map/costmap_2d_ziyan.hpp"
#include <planner/smac_planner_hybrid.hpp>
#include <planner/smac_planner_2d.hpp>

#include "main_aux.hpp"
#include <assert.h>

using namespace ZiYan_IO;

int main() {
    // parse config
    Info::SharedPtr node = std::make_shared<Info>();
    auto configMap = parseConfigFile("./data/config.cfg");
    readConfigFileToInfo(configMap, node);

    int x = node->occupancymap_params.width, y = node->occupancymap_params.height;
    int start_x = node->occupancymap_params.start_x, start_y = node->occupancymap_params.start_y;
    int end_x = node->occupancymap_params.end_x, end_y = node->occupancymap_params.end_y;

    std::streamsize buffer_size = x * y * sizeof(uint8_t);
    uint8_t* map_u = new uint8_t[x * y];
    int8_t* map_i = new int8_t[x * y];

    try
    {
        readArray(configMap["other.map_data_path"], map_u, buffer_size);
        convertArray(map_i, map_u, x*y);
        delete[] map_u;
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
    }

    OccupancyGrid ins_map(
        node->occupancymap_params.width, 
        node->occupancymap_params.height, 
        node->occupancymap_params.resolution,
        node->occupancymap_params.origin_x, 
        node->occupancymap_params.origin_y, 
        map_i
    );
    std::shared_ptr<Costmap2DZiYan> costmap_ziyan = std::make_shared<Costmap2DZiYan>(
        node, ins_map    
    );
    nav2_costmap_2d::Costmap2D* costmap_ptr = costmap_ziyan->getCostmap();
    costmap_ptr -> saveMap(configMap["other.map_after_inflation_path"]);

    PoseStamped start, end;
    costmap_ptr->mapToWorld(start_x, start_y, start.pose.position.x, start.pose.position.y);
    costmap_ptr->mapToWorld(end_x, end_y, end.pose.position.x, end.pose.position.y);

    unsigned int ref_x_s, ref_y_s, ref_x_e, ref_y_e;
    costmap_ptr->worldToMap(start.pose.position.x, start.pose.position.y, ref_x_s, ref_y_s);
    costmap_ptr->worldToMap(end.pose.position.x, end.pose.position.y, ref_x_e, ref_y_e);
    
    assert(ref_x_s == start_x && ref_y_s == start_y); 
    assert(ref_x_e == end_x && ref_y_e == end_y);

    std::stringstream ss;
    ss << "_s(" << start_x << "_" << start_y << ")"
       << "_e(" << end_x << "_" << end_y << ")"
       << "_r(" <<node->plannerhybrid_params.minimum_turning_radius << ")";
    std::string path_suffix = ss.str();
    std::cout << path_suffix << std::endl;
    // plan astar 2d
    {
        auto planner_2d = std::make_unique<nav2_smac_planner::SmacPlanner2D>();
        planner_2d->configure(node, costmap_ziyan);
        planner_2d->activate();

        auto dummy_cancel_checker = []() {
        return false;
        };

        Path path;
        path = planner_2d->createPlan(start, end, dummy_cancel_checker);

        planner_2d->deactivate();
        planner_2d->cleanup();

        planner_2d.reset();

        ZIYAN_INFO("Path size: %d", path.path.size());
        unsigned int* out = new unsigned int[path.path.size() * 2];
        int iidx = 0;
        for (const Entry& coord : path.path) {
            out[iidx++] = coord.x;
            out[iidx++] = coord.y;
        }

        std::string file_name = "./data/out_path_2d" + path_suffix + ".bin";
        saveArray(out, path.path.size() * 2, file_name);
        delete[] out;
    }

    // plan astar hybrid
    {
        auto planner = std::make_unique<nav2_smac_planner::SmacPlannerHybrid>();
        planner->configure(node, costmap_ziyan);
        planner->activate();

        auto dummy_cancel_checker = []() {return false;};

        Path path;
        path = planner->createPlan(start, end, dummy_cancel_checker);

        planner->deactivate();
        planner->cleanup();

        planner.reset();

        {
            ZIYAN_INFO("Path size: %d", path.path.size());
            unsigned int* out = new unsigned int[path.path.size() * 2];
            int iidx = 0;
            for (const Entry& coord : path.path) {
                // std::cout << "x: " << coord.x << ", y: " << coord.y << std::endl;
                out[iidx++] = coord.x;
                out[iidx++] = coord.y;
            }

            std::string file_name = "./data/out_path_hybrid" + path_suffix + ".bin";
            saveArray(out, path.path.size() * 2, file_name);
            delete[] out;
        }

        {
            double* poseout = new double[path.path.size() * 2];
            int iidx = 0;
            for (const PoseStamped& pose : path.poses) {
                // std::cout << "x: " << pose.pose.position.x << ", y: " << pose.pose.position.y << std::endl;
                poseout[iidx++] = pose.pose.position.x;
                poseout[iidx++] = pose.pose.position.y;
            }

            std::string file_name = "./data/out_path_hybrid_world" + path_suffix + ".bin";
            saveArray(poseout, path.path.size() * 2, file_name);
            delete[] poseout;
        }
    }

    return 0;
}