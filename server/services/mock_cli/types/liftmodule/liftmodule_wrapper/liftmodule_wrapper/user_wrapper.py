import rclpy
from rclpy.node import Node

# state_publisher:start
def state_publisher():
    return ""
# state_publisher:stop

# lift:start
def lift(service_params={"position":0.0}):
    # TODO: Put your convey_backward code here
    return None
# lift:stop


# The main function is only used when we want to test specific function
def main():
    import sys
    if len(sys.argv) == 1:
        sys.exit(1)
    if sys.argv[1] == "lift":
        lift()


if __name__ == '__main__':
    main()
