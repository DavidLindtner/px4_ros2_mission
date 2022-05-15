#!/bin/bash
# run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# It assumes px4 is already built, with 'make px4_sitl_default gazebo'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this:
#./Tools/gazebo_sitl_multiple_run.sh -n 10 -m iris

function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

function spawn_model() {
	MODEL=$1
	N=$2 #Instance Number
	X=$3
	Y=$4
	Z=$5
	X=${X:=0.0}
	Y=${Y:=$((3*${N}))}
	Z=${Z:=0.83}
	
	SUPPORTED_MODELS=("iris" "plane" "standard_vtol" "rover" "r1_rover" "typhoon_h480")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL"* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi

	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $N in $(pwd)"
	../bin/px4 -i $N -d "$build_path/etc" -w sitl_${MODEL}_${N} -s etc/init.d-posix/rcS >out.log 2>err.log &
	python3 ${src_path}/Tools/sitl_gazebo/scripts/jinja_gen.py ${src_path}/Tools/sitl_gazebo/models/${MODEL}/${MODEL}.sdf.jinja ${src_path}/Tools/sitl_gazebo --mavlink_tcp_port $((4560+${N})) --mavlink_udp_port $((14560+${N})) --mavlink_id $((1+${N})) --gst_udp_port $((5600+${N})) --video_uri $((5600+${N})) --mavlink_cam_udp_port $((14530+${N})) --output-file /tmp/${MODEL}_${N}.sdf

	echo "Spawning ${MODEL}_${N} at ${X} ${Y}"

	gz model --spawn-file=/tmp/${MODEL}_${N}.sdf --model-name=${MODEL}_${N} -x ${X} -y ${Y} -z ${Z}

	popd &>/dev/null

}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>] [-w <world>] [-s <script>]"
	echo "-s flag is used to script spawning vehicles e.g. $0 -s iris:3,plane:2"
	exit 1
fi

while getopts n:m:w:s:t:l: option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
		s) SCRIPT=${OPTARG};;
		t) TARGET=${OPTARG};;
		l) LABEL=_${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=3}
world=${WORLD:=empty}
target=${TARGET:=px4_sitl_default}
vehicle_model=${VEHICLE_MODEL:="iris"}
export PX4_SIM_MODEL=${vehicle_model}

echo ${SCRIPT}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."

build_path=${src_path}/build/${target}
mavlink_udp_port=14560
mavlink_tcp_port=4560

echo "killing running instances"
pkill -x px4 || true

sleep 1

source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/${target}

# To use gazebo_ros ROS2 plugins
if [[ -n "$ROS_VERSION" ]] && [ "$ROS_VERSION" == "2" ]; then
	ros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
else
	ros_args=""
fi

echo "Starting gazebo"
gzserver ${src_path}/Tools/sitl_gazebo/worlds/${world}.world --verbose $ros_args &
sleep 5


spawn_model ${vehicle_model} 0 0 0 1
spawn_model ${vehicle_model} 1 -38 30 8
spawn_model ${vehicle_model} 2 -30 50 6


trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo client"
gzclient
