set -e
source "/opt/ros/foxy/setup.bash"
echo $NAVEDU_ROBOT
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}]: {message}"
python3 publisher.py