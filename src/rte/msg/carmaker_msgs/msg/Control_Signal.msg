std_msgs/Header header
float32 steerangle #rad -270 ~ 270
float32 brake      #0: no brake, 1: full brake 
float32 gas        #0: no accel pedal, 1: full accel pedal
float32 accel      #m/s^2 : if ACC mode is on, this value is used
int32   gear       #1: drive, 0: neutral, -1: rear, -9: parking
