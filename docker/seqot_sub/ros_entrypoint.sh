set -e
echo $NAVEDU_ROBOT
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}]: {message}"
source "/opt/ros/foxy/setup.bash"
python3 subscriber.py --cfg ./config/config.yaml