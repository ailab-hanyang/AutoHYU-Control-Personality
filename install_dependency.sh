# install dependency

# Since kvaser CANlib already exists, it is commented out to prevent it from being overwritten.

## Install Kvaser CANlib
# sudo apt-add-repository -y ppa:astuff/kvaser-linux
# sudo apt install kvaser-canlib-dev kvaser-drivers-dkms -y

## Install ROS packages
sudo apt install ros-noetic-jsk-rviz-plugins ros-noetic-ros-numpy ros-noetic-nmea-msgs ros-noetic-gps-common ros-noetic-can-msgs -y
sudo apt install net-tools libpugixml-dev libgeographic-dev rospack-tools libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev libpcap-dev coinor-libipopt-dev gfortran liblapack-dev pkg-config swig cmake -y
sudo apt-get install ros-noetic-rosbridge-server ros-noetic-derived-object-msgs -y