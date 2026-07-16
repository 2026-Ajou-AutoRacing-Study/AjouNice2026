from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, List, Optional


@dataclass
class PlannerData:
    lanelet_map: Optional[Any] = None
    odometry: Optional[Any] = None
    dynamic_objects: Optional[Any] = None
    no_ground_pointcloud: Optional[Any] = None
    compare_map_filtered_pointcloud: Optional[Any] = None
    traffic_signals: Optional[Any] = None
    acceleration: Optional[Any] = None
    occupancy_grid: Optional[Any] = None
    virtual_traffic_light_states: Optional[Any] = None
    current_time: float = 0.0


@dataclass
class SceneModuleResult:
    module_name: str
    modified: bool = False
    stop_reasons: List[Any] = field(default_factory=list)
    planning_factors: List[Any] = field(default_factory=list)
    markers: List[Any] = field(default_factory=list)
    traffic_signal: Optional[Any] = None
    infrastructure_commands: List[Any] = field(default_factory=list)
    status: str = ""


class SceneModuleManagerInterface(ABC):
    """ROS 1 equivalent of an Autoware BVP scene-module manager plugin."""

    module_name = "unnamed"

    @abstractmethod
    def modify_path_velocity(self, path_with_lane_id, output_path, planner_data):
        """Modify output_path in place and return a SceneModuleResult."""

    def on_map_updated(self, lanelet_map):
        del lanelet_map

    def reset(self):
        pass
