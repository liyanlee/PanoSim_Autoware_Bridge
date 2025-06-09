#/bin/bash

export AUTOWARE_HOME=/home/panosim/autoware
export Script_HOME=$AUTOWARE_HOME/PanoSimBridge


source $AUTOWARE_HOME/install/setup.bash

cd $Script_HOME

startArray=("RecvIMU" "RecvImage" "RecvGNSS" "RecvEgoStatus" "RecvLidar" "tf_node" "SendControl")

for value in ${startArray[@]};do
    echo "Stop $value"
    pid=$(ps -ef | grep $value.py | grep -v grep | awk '{print $2}')
    if [ ! -z "$pid" ]; then
        kill -9 $pid
    else
        echo "No process found for $value"
    fi
    sleep 0.5
done

echo "All processes stopped."

