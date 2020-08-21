# Setup Guide
Zero step:
Before you will have to enable ssh inside RASPberry pi and other basic systems, so 
connect dirrectly to a monitos and keyboard and execute the script in this repo called:
***pan_and_tilt_setup/ubuntumate_raspberrypi_setup.sh***

First get inside your RaspBerry Pi through ssh or RDP.
```bash
ssh user_name@IP_DEVICE
```

Then you install git to be able to download this repo
```bash
sudo apt install git
```

Once done, download this repo:
```bash 
git clone https://bitbucket.org/theconstructcore/pan_and_tilt_morpheus_chair.git
```

Then execute the following commands to install ROS and enable SMBUS for the Pan and Tilt Hast for RAspberry PI
```bash
./roskinetic_install.sh
```
And follow the instructions in the script ***setup_ic2_for_smbus.sh***

Now you should be able to execute everything related to the Pan and Tilt. For the Camera setup keep on reading:

***PENDING***
