#include <autoware/costmap_generator/costmap_core.hpp>
#include <algorithm>
#include <cmath>
namespace autoware::costmap_generator {
grid_map::GridMap CostmapCore::initialize(const std::string & frame,double lx,double ly,double res,double x,double y){grid_map::GridMap map;map.setFrameId(frame);map.setGeometry({lx,ly},res,{x,y});return map;}
grid_map::Matrix CostmapCore::combine(const grid_map::GridMap & map,const std::string & points,const std::string & objects,const std::string & primitives,const std::string & combined,double minimum){auto out=map.exists(combined)?map[combined]:grid_map::Matrix::Constant(map.getSize().x(),map.getSize().y(),minimum);out.setConstant(minimum);for(const auto * name:{&points,&objects,&primitives})if(map.exists(*name))out=out.cwiseMax(map[*name]);return out;}
void CostmapCore::snapCenterToPose(grid_map::GridMap & map,double x,double y){const auto p=map.getPosition();const double r=map.getResolution();map.setPosition(p+grid_map::Position(std::round((x-p.x())/r)*r,std::round((y-p.y())/r)*r));}
}
