# GelSlim

## System Requirements
- Ubuntu 18.04 Bionic Beaver
- Python 2.7
- ROS Melodic         
  * *http://wiki.ros.org/melodic/Installation/Ubuntu*
- Python OpenCV 3.4.4 
  * *pip install opencv-python==3.4.4*
- Arp Scan            
  * *sudo apt-get install -y arp-scan*
- NMap
  * *sudo apt-get install nmap*
- Matplotlib          
  * *pip install matplotlib*
- Scipy               
  * *pip install scipy*

## Raspberry Pi Settings
 - Turn on SSH in Ubuntu Mate
   * *sudo systemctl enable ssh*
 
 - Set Raspi IP Manually
   * *IPV4 IP Address: 192.168.1.XXX*
   * *Net Mask: 255.255.255.0*
   * *DNS 8.8.4.4.,8.8.8.8*

## Installation

- Download the GelSlim folder and unzip

- cd to ./GelSlim in terminal

- run python gui.py

## IP Address

### To change the IP Address used for ROS Master and Raspberry Pi use the included arguments for gui.py
#### Note the default IP Addresses are currently set to be "NONE" valued so the script will automatically locate them for you.

  - python gui.py -a 1      *Finds IP Addresses automatically using **arp** terminal command*

  - python gui.py -a 2      *Set IP Addresses manually*
  
  - change the default IP Address in line 21-22 of gui.py
  
  - **NOTE:** the local name and password for the raspberry pi must be "raspi" for the automatic login of ssh to work
              if this is not the case you must change the local name and password within "raspberry_new.launch"
    
## Error Log

https://docs.google.com/document/d/1r3EBdHhFpfCTQFS_eKzmCBGyteXqJAIMatUEwNnOWO8/edit
