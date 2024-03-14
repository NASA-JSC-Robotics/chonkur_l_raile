import os
import yaml

import rclpy
from rclpy.node import Node


from ament_index_python.packages import get_package_share_directory

from controller_manager_msgs.srv import ListControllers

class ControlStatusClient(Node):
    def __init__(self, sim):
        super().__init__('ctrl_status_client')
        if sim:
            self.get_logger().info('SIM SELECTED')
            self.ctrlr_cfg_path = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'sim_controllers.yaml')
        else:
            self.get_logger().info('HARDWARE SELECTED')
            self.ctrlr_cfg_path = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'hardware_controllers.yaml')
        self.list_ctrlrs = self.create_client(ListControllers, '/controller_manager/list_controllers')
        while not self.list_ctrlrs.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.list_ctrlrs.srv_name} is not available, waiting again...')
        self.list_ctrlrs_req = ListControllers.Request()

    def get_spawned_ctrlrs(self):
        self.future = self.list_ctrlrs.call_async(self.list_ctrlrs_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.spawned_ctrlrs = self.future.result().controller
        return self.spawned_ctrlrs
    
    def get_all_ctrlrs(self):
        # TODO: If ctrlr_cfg_path doesn't exist, fail return exit
        if not os.path.exists(self.ctrlr_cfg_path):
            self.get_logger().error(f"Failed to find configuration file at {self.ctrlr_cfg_path}, file does not exist")
            self.all_ctrlrs = None
        else:
            with open(self.ctrlr_cfg_path, 'r') as file:
                ctrlr_cfg = yaml.safe_load(file)
            self.all_ctrlrs = list(ctrlr_cfg['controller_manager']['ros__parameters'].keys())
        return self.all_ctrlrs
    
    def compare_ctrlrs(self):
        self.get_spawned_ctrlrs()
        self.get_all_ctrlrs()
        if self.all_ctrlrs == None:
            ctrlr_status = None
        else:
            spawned_ctrlr_names = [ctrlr.name for ctrlr in self.spawned_ctrlrs]
            spawned_ctrlr_states = [ctrlr.state for ctrlr in self.spawned_ctrlrs]
            ctrlrs_spawned = {'name': spawned_ctrlr_names, 'state':spawned_ctrlr_states}
            ctrlrs_not_spawned = {'name': list((set(self.all_ctrlrs) - set(spawned_ctrlr_names)))}
            ctrlr_status = {'spawned': ctrlrs_spawned, 'not_spawned': ctrlrs_not_spawned}
        return ctrlr_status
