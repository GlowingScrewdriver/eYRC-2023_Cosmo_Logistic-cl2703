#!/bin/sh

### A wrapper for some common tasks in the Hardware Arena ###
### Usage: utils.sh <task> <args> 

ebot_move () { ## Control motion of the ebot
    case "$1" in
    stop) ## Stop moving
        vel='{}'
        ;;
    spin) ## Spin
        vel='{angular: {z: 0.4}}'
        ;;
    fwd)  ## Move foward, i.e. in the direction of laser scanner
        vel='{linear: {x: 0.2}}'
        ;;
    bwd)  ## Move backward, i.e. in the direction of electromagnet
        vel='{linear: {x: -0.2}}'
        ;;
    *)
        echo "Please specify a valid argument to ebot_move"
        usage
    esac

    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "$vel"
}

magnet_off () { ## Turn off selected magnets
    case "$1" in
    arm)  ## Arm box gripper magnet
        service=/io_and_status_controller/set_io
        type=ur_msgs/srv/SetIO
        msg="{fun: 1, pin: 16, state: 0.0}"
        ;;
    ebot) ## eBot rack gripper magnet
        service=/usb_relay_sw
        type=usb_relay/srv/RelaySw
        msg="{relaychannel: 1, relaystate: true}"
        ;;
    esac

    ros2 service call "$service" "$type" "$msg"
}

usage () {
    echo "Usage: utils.sh <subcommand> [<subcommand args>]"
    echo "Available subcommands and arguments:"
    sed -n -e 's/ () { *## /   # /p' -e 's/) *##/   #/p' $0
    exit 1
}

if [ "$1" = "" ]; then
    usage
fi

$@
