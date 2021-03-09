import sys
import rospy
from drone_wrapper import DroneWrapper

DRONE = None

def do_land(*args):
    global DRONE
    if DRONE:
        DRONE.land()

def do_takeoff(*args):
    data = args[0]
    if not data:
        h = 3
    else:
        h = int(data)

    global DRONE
    if DRONE:
        DRONE.takeoff()

def do_connect(*args):
    global DRONE
    DRONE = DroneWrapper()

def do_help(*args):
    data = args[0]
    if not data:
        print("Command Line Drone Wrapper, version 0.1.0")
        print("Commands are defined internally. Type help' to see this list.")
        print("Type 'help cmd' to know more about the command 'cmd'.")
        print("Type 'help clw' to know more about Command Ground Control in general.")
        print("")
        print("connect")
        print("help [cmd]")
        print("takeoff [height]")
        print("land")
    elif data[0] == "clw":
        print("Command Line Drone Wrapper, version 0.1.0")
        print("Author: Pedro Arias Perez")
        print("")
        print("Description: WORK IN PROGRESS")
    elif data[0] == "connect":
        print("connect: connect")
        print("WORK IN PROGRESS")
    elif data[0] == "help":
        print("help: help [cmd]")
        print("WORK IN PROGRESS")
    elif data[0] == "takeoff":
        print("takeoff: takeoff [height]")
        print("WORK IN PROGRESS")
    elif data[0] == "land":
        print("land: land")
        print("WORK IN PROGRESS")
    else:
        print("[Error] Invalid command. Please type 'help' to see available commands.")


def process_line(s):
    tokens = s.split(" ")
    if tokens[0] == "help":
        do_help(tokens[1:])
    elif tokens[0] == "connect":
        do_connect(tokens[1:])
    elif tokens[0] == "takeoff":
        do_takeoff(tokens[1:])
    elif tokens[0] == "land":
        do_land(tokens[1:])
    elif tokens[0] == "":
        pass
    else:
        print("[Error] Invalid command. Please type 'help' to see available commands.")


def main():
    while True:
        try:
            s = raw_input("> ")
        except EOFError:
            break
        process_line(s)


if __name__ == '__main__':
    main()
