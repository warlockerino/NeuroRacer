# NeuroRacer
## Systemtools
These scripts can only be used as root, otherwise the files wont be readable.
For camera usage, use python2 for caopatibility reasons with opencv.
Make sure, that you have persmission to acces  videopipe(tk and tx1 boards) or the videofeed(/dev/video0/ on tx2)

## Arduino
needed:

* IDE (Platformio, Arduino, etc.) 
* ROS Serial Package for the IDE
* Arduino (leonardo or Micro work out of the box)
* Cable (seems obvious, but some wont work)

steps:
1. identify usbdevice `ls /dev/ | grep tty*` (should be listed as tty ACM* or ttyUSBx)
2. run `sudo chmod 666 /dev/<your_tty>` to make the port available
3. `optional:` But worth the effort. Uninstall the Ubuntu modemmanager 
(`sudo apt-get purge modemmanager`) since it keeps locking up usb connections
4. now flash the sketch to the Arduino. **Important: write down the choosen baudrate (9600 - 115200) its needed later**
5. After the flash ended sucessfully, its time to connect the Arduino with ROS. For that purpose, three terminals are needed. 
The following commands need to be execeuted in this order, to run the Node:
  1.`roscore` to boot ros and initialize the system
  2. `rosrun rosserial_python serial_node.py _port:=<your_tty> _baud:=<your_baudrate>` to setup the connected arduino as a node
  3. `rosrun <your_packet>  <your_node>` or `roslaunch <your_packet>  <your_launchfile>` to setup the node the Arduino should talk to
  4. `optional:` `rostopic echo /rosout` or `rostopic echo <your_topic>` to monitor the Setups
