# Allegro Hand Interface
Adapted from https://github.com/simlabrobotics/allegro_hand_linux_v4

## TODO

-  have a class-based API


# Required hardware
1. [Allegro hand v4](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Allegro_Hand_v4.0)
2. [PCAN-USB interface](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1)

# Setup instructions
## Prerequisites
### 1. PCAN-USB driver
Download, build, and install PCAN-USB driver for Linux "libpcan"
```bash
tar -xzvf peak-linux-driver-x.x.tar.gz
cd peak-linux-driver-x.x
make NET=NO
sudo make install
```
#### Troubleshooting
* If encounter 
  ```bash
  src/pcan-settings.c:47:10: fatal error: popt.h: No such file or directory
     47 | #include <popt.h>
      |      	^~~~~~~~
  compilation terminated.
  ```
  Run
  ```bash
  sudo apt install libpopt-dev
  ```

### 2. `libpcanbasic`
Download, build, and install PCAN-Basic API for Linux: libpcanbasic
```bash
tar -xzvf PCAN_Basic_Linux-x.x.x.tar.gz
cd PCAN_Basic_Linux-x.x.x/pcanbasic
make
sudo make install
```

### 3. `libBHand` grasping library
Download, build, and install Grasping Library for Linux, "libBHand": Grasping_Library_for_Linux
```bash
unzip LibBHand_{32|64}.zip
cd libBHand_{32|64}
sudo make install
sudo ldconfig
```


## Installing this interface


```
pip install .
```

# Usage

See `example.py`