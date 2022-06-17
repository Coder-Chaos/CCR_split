#load the c++ lib
#sudo ip link set down can0 
#sudo ifconfig can0 txqueuelen 1000
#sudo ip link set can0 type can bitrate 1000000
#sudo ip link set up can0
sudo ifconfig can0 down
#sudo ifconfig can0 up
#cansend can0 000#01.00.00.00.00.00.00.00
#cansend can0 205#06.00.00.00.00.00.00.00
#cansend can0 205#0f.00.00.00.00.00.00.00
