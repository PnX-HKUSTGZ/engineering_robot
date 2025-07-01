from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("exchange_station", package_name="exchange_station_moveit").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
