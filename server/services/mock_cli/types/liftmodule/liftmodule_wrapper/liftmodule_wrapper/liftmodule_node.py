# System
import json

# Related to ROS
import rclpy
from rclpy.node import Node

import user_wrapper

# FARobot msg
from far_fleet_msgs.msg import Artifact
# Artifact msg type
# string id
# string conf_info
# string service
# string fleet_name
from far_wms_msgs.msg import StorageCell
# string area_id
# string cell_id
# string map
# geometry_msgs/Pose pose
# string type
# string direction
# string status
# float32 size_x
# float32 size_y


class ArtifactWrapper(Node):

    def __init__(self):
        super().__init__('artifact_liftmodule_wrapper', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # TODO: The artifact ID is from RMT agent
        self.liftmodule_id = "liftmodule_01"

        # Publish artifact status
        self.state_pub_ = self.create_publisher(Artifact, 'artifact_state', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Publish storage cell
        self.wms_pub_ = self.create_publisher(StorageCell, '/storage_manager/agent_storage_cell_update', 10)

        # Subscribe the artifact request from AMR
        self.req_sub_ = self.create_subscription(
            Artifact,
            '/artifact_request',
            self.request_callback,
            10)
        self.req_sub_  # prevent unused variable warning

    def timer_callback(self):
        # TODO: state_publisher should show in json file.
        msg = Artifact()
        msg.id = self.liftmodule_id
        msg.service = user_wrapper.state_publisher()
        self.state_pub_.publish(msg)

    def request_callback(self, msg):
        # Only process msg to myself
        self.artifact_id = msg.id
        if self.artifact_id != self.liftmodule_id:
            return
        service_req_json = json.loads(msg.service)
        for service_req in service_req_json:
            service_params = service_req_json[service_req]
            if service_req == "lift":
                user_wrapper.lift(service_params)


def main():
    rclpy.init(args=args)

    artifact_wrapper = ArtifactWrapper()

    rclpy.spin(artifact_wrapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    artifact_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
