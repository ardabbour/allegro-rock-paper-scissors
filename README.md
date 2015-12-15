Allegro Hand / Kinect Rock Paper Scissors
=========================================
This project works only for PEAK System CAN interface (chardev) for USB: PCAN-USB and has only been tested on Kinect v1 through Processing v2.2.1 running on Ubuntu 14.04.

Download
--------
Git Repo
https://github.com/thedabbour/allegro_kinect_rpc/

Dependencies
------------
1. PCAN-USB driver from http://www.peak-system.com/fileadmin/media/linux/index.htm
2. PCAN-Basic API from http://www.peak-system.com/PCAN-USB.199.0.html
4. Grasping Library from http://simlab.dyndns.org:9000/AllegroHandWiki/index.php/Grasping_Library_for_Linux
5. Processing from https://processing.org/download/?processing (at this moment only v2.2.1 supports the SimpleOpenNI library)
6. SimpleOpenNI from https://code.google.com/p/simple-openni/downloads/list
7. FingerTracker from http://makematics.com/code/FingerTracker/

Instructions
------------
1. Install all the dependencies mentioned above.
2. Modify the 'main.cpp' function to include the correct motor offset values of you Allegro Hand model and the file location of gesture recognition result.
3. Play!

-----------------------------------------------------------------------------------------------------------------------------------
The Allegro Hand project was developed by Dr. Kyong-Sok "K.C." Chang; this repository is a fork of the original that can be found over at https://github.com/simlabrobotics/allegro_hand_linux
