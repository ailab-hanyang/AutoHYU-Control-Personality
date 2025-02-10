# AutoHYU-Control-Personality
This repository is for submitting HMG’s Personality assignment results.

## Getting Start
Please clone the project in your ${HOME}/git/ directory

'''bash
mkdir -p ~/git && cd ~/git
git clone https://github.com/ailab-hanyang/AutoHYU-Control-Personality.git
cd ~/git/AutoHyu-Control-Personality
catkin_make -DCMAKE_BUILD_TYPE=Release
'''

## Configuring Novatel Offset
To set the Novatel offset, you can call the respective ROS services:

- **Offset to Center of Gravity (CG):**
  ```bash
  rosservice call /novatel/oem7/receivers/main/Oem7Cmd "SETINSTRANSLATION USER 1.5 0.0 0.0 0.1 0.0 0.0 IMUBODY"
  ```
  
- **Offset to Rear Axle (INS Mount):**
  ```bash
  rosservice call /novatel/oem7/receivers/main/Oem7Cmd "SETINSTRANSLATION USER 0.0 0.0 0.0 0.0 0.0 0.0 IMUBODY"
  ```

#### Additional Resources
For more information on runtime operations and specific command usage, refer to the following documentation:

- [Novatel OEM7 Driver Runtime Operation](https://wiki.ros.org/novatel_oem7_driver/runtime_operation)
- [Novatel SETINSTRANSLATION Command](https://docs.novatel.com/OEM7/Content/SPAN_Commands/SETINSTRANSLATION.htm?tocpath=Commands%20%2526%20Logs%7CCommands%7CSPAN%20Commands%7C_____23)

## Excluding Specific Packages from the Build
To exclude specific packages from the build process, use the following command:
'''bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES='lateral_control;longitudinal_control;'
'''

## ACADOS import issue (when it's installed but cannot find 'acados_template')
'''bash
cd src/lib/acados/
rm -rf c_generated_code/ include/ build/ lib/
cd ..
./install_acados
source ~/.bashrc
'''
