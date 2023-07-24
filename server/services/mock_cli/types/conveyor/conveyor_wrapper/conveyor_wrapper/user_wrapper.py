import rclpy
from rclpy.node import Node

# state_publisher:start
def state_publisher():
    return ""
# state_publisher:stop

# convey_backward:start
def convey_backward(service_params={"state":False}):
    # TODO: Put your convey_backward code here
    return None
# convey_backward:stop

# convey_forward:start
def convey_forward(service_params={"state":True}):
    # TODO: Put your convey_forward code here
    return None
# convey_forward:stop

# sensor_back:start
def sensor_back(service_params):
    # TODO: Put your sensor_back code here
    return None
# sensor_back:stop

# sensor_front:start
def sensor_front(service_params):
    # TODO: Put your sensor_front code here
    return None
# sensor_front:stop


# The main function is only used when we want to test specific function
def main():
    import sys
    if len(sys.argv) == 1:
        sys.exit(1)
    if sys.argv[1] == "convey_backward":
        convey_backward()
    elif sys.argv[1] == "convey_forward":
        convey_forward()
    elif sys.argv[1] == "sensor_back":
        sensor_back()
    elif sys.argv[1] == "sensor_front":
        sensor_front()
    elif sys.argv[1] == "state_publisher":
        state_publisher()


if __name__ == '__main__':
    main()
