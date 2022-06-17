#load the c++ lib

#set the defualt IPAddr
#ifconfig eth0 10.60.2.222

#Set CAN0
#bring down the device
#set the bitrate 500K == 1000K
#bring up the device

cansend can0 000#01.00.00.00.00.00.00.00
cansend can0 205#06.00.00.00.00.00.00.00
cansend can0 205#0f.00.00.00.00.00.00.00

#cansend can0 205#06.00.00.00.00.00.00.00
#cansend can0 000#01.00.00.00.00.00.00.00
#Set CAN1
#ip link set can1 type can bitrate 1000000
#ip link set can1 up

# Excute the program
#./build/src/ccr_split/ccr_core



