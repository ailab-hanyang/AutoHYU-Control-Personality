#!/bin/bash
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
# File name: autohyu_cm.sh
# Author: Yuseung Na, Junhee Lee, Seounghoon Park
# Date: 2024.09.10
# Description: This is a temporary shellscript for convenience
#              TODO : need to combine with autohyu_control.sh
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #

# ==============================================================================
# -- Set up environment --------------------------------------------------------
# ==============================================================================
PRJ_DIR=$HOME/git/AutoHYU-Control-Personality
ROSPKG_LIST=($(bash -ic 'rospack list&'))
MAP="kcity"        # konkuk, grandpark, speedway, kcity, hanyang ...
CAR="hmg"          # hmg, ailab
MODE="carmaker"    # real, carmaker, carla, morai
PROJ="local_cartesian"

USAGE_STRING=$(
  cat <<-END

  Arguments    
    --help                       print usage
    --map [MAP]                  load map at [MAP] (konkuk, grandpark, speedway, kcity)
    --car [CAR]                  target vehicle [CAR] (hmg, ailab)
    --mode [MODE]                opearation mode [MODE] (real, carmaker, carla, morai)

END
)


# ==============================================================================
# -- Parse arguments -----------------------------------------------------------
# ==============================================================================

# Set argument names
options=$(getopt -o cmg -l help,map:,car: -- "$@")
if [ $? != 0 ]; then
  echo "ERROR: print usage"
  exit 1
fi

eval set -- "$options"

# Parse arguments
while true; do
  case $1 in
  -h | --help)
    echo "$USAGE_STRING"
    exit
    ;;
  --map)
    MAP=$2
    shift 2
    ;;
  --car | -c)
    CAR=$2
    shift 2
    ;;
  --mode | -m)
    MODE=$2
    shift 2
    ;;
  --proj | -p)
    PROJ=$2
    shift 2
    ;;
  --)
    shift
    break
    ;;
  esac
done

# Print a message in orange color to indicate the selected map location.
echo -e "\e[37mMap location: $MAP\e[0m"
echo -e "\e[37mCar: $CAR\e[0m"
echo -e "\e[37mMode: $MODE\e[0m"
echo -e "\e[37mProject Mode: $PROJ\e[0m"

# ==============================================================================
# -- Run autonmous driving platform and environment ----------------------------
# ==============================================================================
# launch_list+=("roscore")
# number_of_launch_files=1

# hmi
# launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen launch/hmi.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)")
# number_of_launch_files=$((${number_of_launch_files} + 1))

# bsw
# launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen launch/bsw.launch car:=${CAR} location:=${MAP} mode:=${MODE}")
# number_of_launch_files=$((${number_of_launch_files} + 1))
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen launch/simulator.launch location:=${MAP} mode:=${MODE} proj_mode:=${PROJ}")
number_of_launch_files=$((${number_of_launch_files} + 1))
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen src/bsw/driver/virtual_object_generator/launch/virtual_object_generator.launch")
number_of_launch_files=$((${number_of_launch_files} + 1))

# localization

# planning
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen src/app/planning/waypoint_planning/launch/waypoint_planning.launch location:=${MAP}")
number_of_launch_files=$((${number_of_launch_files} + 1))
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen src/app/perception/motion_prediction/launch/motion_prediction.launch location:=${MAP}")
number_of_launch_files=$((${number_of_launch_files} + 1))

# rte
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen launch/rte.launch")
number_of_launch_files=$((${number_of_launch_files} + 1))

# TODO : make this to hmi
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; rosrun rviz rviz")
number_of_launch_files=$((${number_of_launch_files} + 1))

# control
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen src/app/control/acados_scc_control/launch/acados_scc_control.launch")
number_of_launch_files=$((${number_of_launch_files} + 1))
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen src/app/control/lateral_control/launch/lateral_control.launch")
number_of_launch_files=$((${number_of_launch_files} + 1))
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen src/app/control/longitudinal_control/launch/longitudinal_control.launch")
number_of_launch_files=$((${number_of_launch_files} + 1))
launch_list+=("sleep ${number_of_launch_files}; source devel/setup.bash; roslaunch --screen src/app/control/grip_manager/launch/grip_manager.launch")
number_of_launch_files=$((${number_of_launch_files} + 1))

# ==============================================================================
# -- Terminator Layout ---------------------------------------------------------
# ==============================================================================

# echo "  Initialize to construct Terminator Layout..."
file=$(dirname "$0")/.config
upper_line_number=$(grep -n AutoHYU-Layout ~/.config/terminator/config | cut -d: -f1)
lower_line_number=$(grep -n plugins ~/.config/terminator/config | cut -d: -f1)

# Remove content in terminator config file
if [[ -n ${upper_line_number} ]]; then
  sed -i "${upper_line_number},$((${lower_line_number}-1))d" ~/.config/terminator/config
  # echo "  Terminator Layout \"AutoHYU-Layout\" exists, remove and re-construct it..."
fi

number_of_tabs=$(( (${number_of_launch_files} + 8 - 1) / 8 ))
number_of_windows=$(( ${number_of_tabs} * 6 ))
number_of_terminals=$(( ${number_of_tabs} * 8 ))

# Make layout config file named 'AutoHYU'
cat <<EOM >${file}
\  [[AutoHYU-Layout]]
    [[[root]]]
      fullscreen = False
      last_active_window = True
      maximised = True
      order = 0
      parent = ""
      title = AutoHYU
      type = Window
EOM

# Divide tab
if [[ ${number_of_tabs} > 1 ]]; then

cat <<EOT >> ${file}
    [[[tab]]]
      active_page = 0
      order = 0
      parent = root
      type = Notebook
EOT

fi

# Divide tabs
for ((tab_id = 0; tab_id < ${number_of_tabs}; tab_id++)); do
  if [[ ${number_of_tabs} == 1 ]]; then
    cat <<EOT >> ${file}
    [[[tab${tab_id}]]]
      order = ${tab_id}
      parent = root
      ratio = 0.5
      type = HPaned
EOT
  else
    cat <<EOT >> ${file}
    [[[tab${tab_id}]]]
      order = ${tab_id}
      parent = tab
      ratio = 0.5
      type = HPaned
EOT
  fi
  # Divide windows
  for ((win_id = 0; win_id < 6; win_id++)); do
    if [[ $(( ${win_id} % 6 )) < 2 ]]; then
      parent_window=tab${tab_id}
      paned_type="HPaned"
    else
      parent_window=window$(( 6*${tab_id} + ${win_id} / 2 - 1 ))
      paned_type="VPaned"
    fi        
    cat <<EOT >> ${file}
    [[[window$((6*${tab_id}+${win_id}))]]]
      order = $(( ${win_id} % 2 ))
      parent = ${parent_window}
      ratio = 0.5
      type = ${paned_type}
EOT
  done  # for win_id
done  # for tab_id

# Divide terminals
if [[ ${number_of_launch_files} -gt ${number_of_terminals} ]]; then
  echo "ERROR: The number of packages (${number_of_launch_files}) is larger than the number of terminals (${number_of_terminals})."
  exit 1
fi

for ((term_id = 0; term_id < ${number_of_terminals}; term_id++)); do
  if [[ $(( ${term_id} % 8 )) < 2 ]]; then
    parent_terminal=window$(( ${term_id}/8 * 6 + 2 ))
  elif [[ $(( ${term_id} % 8 )) < 4 ]]; then
    parent_terminal=window$(( ${term_id}/8 * 6 + 3 ))
  elif [[ $(( ${term_id} % 8 )) < 6 ]]; then
    parent_terminal=window$(( ${term_id}/8 * 6 + 4 ))
  else
    parent_terminal=window$(( ${term_id}/8 * 6 + 5 ))
  fi  
  
  if [[ ${term_id} -lt ${#launch_list[@]} ]]; then
    command="${launch_list[$term_id]} && bash"
  else
    command=""
  fi
  
  cat <<EOT >> ${file}
    [[[terminal${term_id}]]]
      directory = ${PRJ_DIR}
      command = ${command}
      order = $(( ${term_id} % 2 ))
      parent = ${parent_terminal}
      title = ""
      type = Terminal
EOT
done

sed -i 's/$/\\/g' ${file}
sed -i '$s/\\//g' ${file}

# Add config file to terminator
config_file=$(cat ${file})
sed -i "/\[plugins\]/ i ${config_file}" ~/.config/terminator/config

# echo "  Complete to construct Terminator layout!"

# ==============================================================================
# -- Launch AutoHYU --------------------------------------------------------------
# ==============================================================================

# terminator -p '~/.config/terminator/config' -l AutoHYU-Layout&
bash -ic "terminator -p '~/.config/terminator/config' -l AutoHYU-Layout&"

# ==============================================================================
# -- ...and we are done --------------------------------------------------------
# ==============================================================================