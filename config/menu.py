#!/usr/bin/env python3
from pick import pick
from subprocess import run

TITLE = "🚌 ROS2 DEMO 🚌\n" + "═" * 50
OPTIONS = [
    ("Exit", None),
    ("Tools: rviz + plotjuggler", "ros2 launch /config/tools_launch.py > /dev/null"),
    ("Create BB1 vehicle", "ros2 launch /config/vehicle_launch.py -- -- vehicle_name:=bb1"),
    ("Create BB2 vehicle", "ros2 launch /config/vehicle_launch.py -- -- vehicle_name:=bb2 y:=+5.0"),
    ("Node list", "ros2 node list"),
    ("Topic list", "ros2 topic list"),
    ("Service list", "ros2 service list"),
    ("Show BB1 velocity", "ros2 topic echo /vehicle/bb1/velocity"),
    ("Show BB1 door state", "ros2 topic echo /vehicle/bb1/door"),
    ("Declare BB1 velocity to  0m/s", "ros2 topic pub --once /vehicle/bb1/declare_velocity std_msgs/msg/Float32 '{data: 0}'"),
    ("Declare BB1 velocity to  5m/s", "ros2 topic pub --once /vehicle/bb1/declare_velocity std_msgs/msg/Float32 '{data: 5}'"),
    ("Declare BB1 velocity to 10m/s", "ros2 topic pub --once /vehicle/bb1/declare_velocity std_msgs/msg/Float32 '{data: 10}'"),
    ("Declare BB2 velocity to  0m/s", "ros2 topic pub --once /vehicle/bb2/declare_velocity std_msgs/msg/Float32 '{data: 0}'"),
    ("Declare BB2 velocity to  5m/s", "ros2 topic pub --once /vehicle/bb2/declare_velocity std_msgs/msg/Float32 '{data: 5}'"),
    ("Declare BB2 velocity to 10m/s", "ros2 topic pub --once /vehicle/bb2/declare_velocity std_msgs/msg/Float32 '{data: 10}'"),
    ("Open BB1 Door", "ros2 service call /vehicle/bb1/open_door std_srvs/SetBool '{data: true}'"),
    ("Close BB1 Door", "ros2 service call /vehicle/bb1/open_door std_srvs/SetBool '{data: false}'"),
    ("Set obstacles count to 5", "ros2 param set /obstacles count_obstacles 5"),
    ("Set obstacles count to 100", "ros2 param set /obstacles count_obstacles 100"),
    ("Run The (Pseudo) Autonomy Controller for BB1", "ros2 run code_prelection autonomy_controller --ros-args -p vehicle_name:=bb1"),
]


while True:
    option, index = pick(
        title=TITLE,
        options=[o for o, _ in OPTIONS],
        indicator="🚌",
    )
    if option.lower() == "exit":
        break

    command = OPTIONS[index][1]
    try:
        run("clear")
        print("Selected:\x1b[1m", option, "\x1b[0m")
        print("Command to execute:\x1b[92m")
        print(command, "\x1b[0m")
        print()
        input("Press enter dude.")
        print("═" * 50)
        print()
    except KeyboardInterrupt:
        run("clear")
        continue

    try:
        run(command, shell=True)
    except KeyboardInterrupt:
        pass

    try:
        print()
        print("═" * 50, "\x1b[93m")
        input("Command finished. Press enter dude.\x1b[0m")
    except KeyboardInterrupt:
        pass
    finally:
        run("clear")

