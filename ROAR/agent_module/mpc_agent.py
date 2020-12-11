from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.configurations.configuration import Configuration as AgentConfig
from pathlib import Path
from ROAR.control_module.mpc_controller import VehicleMPCController
from ROAR.planning_module.mission_planner.waypoint_following_mission_planner import WaypointFollowingMissionPlanner
from ROAR.planning_module.behavior_planner.behavior_planner import BehaviorPlanner
from ROAR.planning_module.local_planner.simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR.utilities_module.data_structures_models import SensorsData


class MPCAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings)
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        self.mpc_controller = VehicleMPCController(agent=self,
                                                   route_file_path=Path(self.agent_settings.waypoint_file_path))
        self.mission_planner = WaypointFollowingMissionPlanner(agent=self)
        self.behavior_planner = BehaviorPlanner(agent=self)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            agent=self,
            controller=self.mpc_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=3)

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(MPCAgent, self).run_step(vehicle=vehicle,
                                       sensors_data=sensors_data)
        vehicle_control = self.local_planner.run_in_series()
        return vehicle_control
