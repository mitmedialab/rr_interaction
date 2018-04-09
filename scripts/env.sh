
# source ros setup
. /opt/ros/indigo/setup.sh

# source ros catkin setup
. /home/robots/projects/ros_catkin_ws/devel/setup.sh

# Add java home (needs to be set for catkin_make to run).
export JAVA_HOME=/usr/lib/jvm/java-7-openjdk-i386/

# Export home directory.
export HOME=/home/robots/

# Add bin to path. Also add directory where python installs pip-env.
export PATH=/home/robots/.local/bin:/home/robots/bin:$PATH

# Credentials for Google ASR.
export GOOGLE_APPLICATION_CREDENTIALS=/home/robots/asr-projects-d6df21a97936.json

# Affdex path and setup
export AFFDEX_DATA_DIR=$HOME/projects/affdex-sdk/data
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/projects/affdex-sdk/lib

# Setup ROS IP
export ROS_IP=$(avahi-resolve-host-name -4 $(hostname).local | cut -f2)
export ROS_MASTER_URI=http://localhost:11311
