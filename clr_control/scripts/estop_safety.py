#!/usr/bin/env python3
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from clr_control.clr_estop_safety_class import CLREStopSafety


def main(args=None):

    rclpy.init(args=args)

    clr_estop_node = CLREStopSafety()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(clr_estop_node)

    try:
        print("spinning")
        executor.spin()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        clr_estop_node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
