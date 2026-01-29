#!/bin/bash

set -e -o pipefail

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml

SHELL_FOLDER=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo $SHELL_FOLDER
pushd "$SHELL_FOLDER"/../../ || exit
    source setup.bash
    export LD_LIBRARY_PATH=./deploy_assets/thirdparty/onnxruntime-linux-x64-1.19.2/lib:$LD_LIBRARY_PATH
    export PLUGIN_INSTALL_DIR="$PWD/legged_system/bin"
    export AIMRT_CFG_PATH="$PWD/legged_system/bin/iceoryx_chn_cfg.yaml"
    ros2 launch rl_controllers rl_control_real.launch.py "$@"
popd || exit
