Allegro Hand / Kinect Rock Paper Scissors
=========================================
This project works only for PEAK System CAN interface (chardev) for USB: PCAN-USB and has only been tested on Kinect v1 through Processing v2.2.1 running on Ubuntu 14.04.

Download
========
Git Repo
https://github.com/thedabbour/allegro_kinect_rpc/

Build and Run: "grasp"
======================

1. Download, build, and install PCAN-USB driver for Linux: libpcan

2. Download, build, and install PCAN-Basic API for Linux: libpcanbasic

3. Download, build, and install Grasping Library for Linux, "libBHand": Grasping_Library_for_Linux

4. Build Allegro Hand Project using cmake "out of source build" style.

mkdir build
cd build
cmake ..
make
make install

Note: You will need to replace the encoder offsets and directions and the motor directions in the array at the top of the main.cpp file. These offsets can be found on the offsets and directions table on your Allegro Hand Wiki page (front page - scroll down): Allegro_Hand_DML_Info

Note: Using cmake "out of source build" style, the entire build tree is created under "build" directory so that you can delete "build" directory without worrying about the sources.

5. Connect PCAN-USB and Allegro Hand

6. Start the grasping program: "grasp"

7. Power on Allegro Hand.

================================================
The Allegro Hand project was developed by Dr. Kyong-Sok "K.C." Chang; this repository is a fork of the original that can be found over at https://github.com/simlabrobotics/allegro_hand_linux
