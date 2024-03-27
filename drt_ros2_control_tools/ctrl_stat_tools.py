import curses
import os
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.callback_groups import ReentrantCallbackGroup
from controller_manager_msgs.srv import ListControllers

class ControlStatusClient(Node):
    def __init__(self, pkg, config):
        super().__init__('ctrl_status_client')
        self.ctrlr_cfg_path = config
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

    
class ControlStatusClientCurses(ControlStatusClient):
    def __init__(self, pkg, config, highlight=None):

        super().__init__(pkg, config)

        self.stdscr = curses.initscr()
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_WHITE)
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_RED)

        self.highlight = highlight
        if self.highlight:
            with open(highlight, 'r') as file:
                self.highlighted = yaml.safe_load(file)
        
    def show_compare(self):
        self.stdscr.clear()
        line = 0
        ctrlr_status = self.compare_ctrlrs()
        if ctrlr_status:
            self.stdscr.addstr(line, 0, 'Listed and spawned:')
            line += 1
            for i in range(len(ctrlr_status['spawned']['name'])):
                name = ctrlr_status['spawned']['name'][i]
                state = ctrlr_status['spawned']['state'][i]
                if self.highlight and name in self.highlighted:
                    if name=='joint_state_broadcaster' and state=='inactive':
                        color = 4
                    else:
                        color = 2
                else:
                    color = 1
                self.stdscr.addstr(line, 0,
                                   '\t - ' + name + ' [' + state + ']',
                                   curses.color_pair(color))
                line += 1
            self.stdscr.addstr(line, 0, 'Listed and not spawned')
            line += 1
            for i in range(len(ctrlr_status['not_spawned']['name'])):
                name = ctrlr_status['not_spawned']['name'][i]
                if self.highlight and name in self.highlighted:
                    color = 4
                else:
                    color = 3
                self.stdscr.addstr(line, 0,
                                   '\t - ' + name,
                                   curses.color_pair(color))
                line += 1
            self.stdscr.refresh()
            return
        else:
            self.get_logger().info('Comparason failed')
            return
    
    def exit(self):
        curses.echo()
        curses.endwin()
        return