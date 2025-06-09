#/bin/bash

export AUTOWARE_HOME=/home/panosim/autoware
export Script_HOME=$AUTOWARE_HOME/PanoSimBridge


source $AUTOWARE_HOME/install/setup.bash

cd $Script_HOME
LogPath=$Script_HOME/logs

if [ ! -d "$LogPath" ]; then
    echo "Directory $LogPath does not exist. Creating it..."
    mkdir -p "$LogPath"
fi

startArray=("RecvIMU" "RecvImage" "RecvGNSS" "RecvEgoStatus" "RecvLidar" "tf_node" "SendControl")

for value in ${startArray[@]};do
    echo "Starting $value"
    /usr/bin/python3 "${value}.py" > "${LogPath}/${value}.log" 2>&1 &
    sleep 0.5
done;


#pkill -f your_launch_file.launch.py
#sleep 2
#ros2 launch autoware_launch panosim_simulator.launch.xml map_path:=$HOME/autoware_map/PanoTown01 vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit  > autoware_launch.log 2>1

echo "All processes started."

