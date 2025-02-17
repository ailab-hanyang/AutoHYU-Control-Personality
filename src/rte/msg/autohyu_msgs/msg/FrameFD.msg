std_msgs/Header header
  
uint32 id
bool is_rtr
bool is_extended
bool is_error
bool is_canfd
uint8 dlc # Byte --> DLC [1-8:1-8] [12:9] [16:10] [20:11] [24:12] [48:13] [64:14]
uint8[] data