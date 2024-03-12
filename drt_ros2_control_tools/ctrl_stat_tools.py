#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node


from ament_index_python.packages import get_package_share_directory

from controller_manager_msgs.srv import ListControllers

class ControlStatusClient(Node):
    def __init__(self):
        super().__init__('ctrl_status_client')

        # IDK how to do either of these
        # TODO: Specify hardware/sim_controllers.yaml via hardware/sim operation
        # TODO: Specify cfg path by running robot
        self.ctrlr_cfg_path = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'sim_controllers.yaml')

        self.list_ctrlrs = self.create_client(ListControllers, '/controller_manager/list_controllers')
        while not self.list_ctrlrs.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.list_ctrlrs.srv_name} is not available, waiting again...')
        self.list_ctrlrs_req = ListControllers.Request()

    def get_spawned_ctrlr_names(self):
        self.future = self.list_ctrlrs.call_async(self.list_ctrlrs_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.spawned_ctrlr_names = [item.name for item in self.future.result().controller]
        return self.spawned_ctrlr_names
    
    def get_all_ctrlr_names(self):
        # TODO: If ctrlr_cfg_path doesn't exist, fail return exit
        with open(self.ctrlr_cfg_path, 'r') as file:
            ctrlr_cfg = yaml.safe_load(file)
        self.all_ctrlr_names = list(ctrlr_cfg['controller_manager']['ros__parameters'].keys())
        return self.all_ctrlr_names
    
    def compare_ctrlrs(self):
        self.get_spawned_ctrlr_names()
        self.get_all_ctrlr_names()
        ctrlrs_spawned = set(self.all_ctrlr_names) & set(self.spawned_ctrlr_names)
        ctrlrs_not_spawned = set(self.all_ctrlr_names) - set(self.spawned_ctrlr_names)
        return ctrlrs_spawned, ctrlrs_not_spawned 


# TODO: put main in a different file
def main():
    rclpy.init()

    ctrl_stat_client = ControlStatusClient()
    spawned, not_spawned = ctrl_stat_client.compare_ctrlrs()
    print(not_spawned)

if __name__ == '__main__':
    main()








# Call list_controllers for controller status
# Get all controllers from compiled configuration yaml
# Compare all controllers vs spawned controllers
# Return compare