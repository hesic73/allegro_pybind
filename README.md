# allegro_pybind

A Python binding for controlling the Allegro Hand, heavily based on https://github.com/simlabrobotics/allegro_hand_linux_v4. The setup instructions are also borrowed from the original project.



**Tested on**:

- Ubuntu 20.04
- g++ 9.4.0
- cmake 3.16.3
- python 3.8.20



## Required hardware
1. [Allegro hand v4](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Allegro_Hand_v4.0)
2. [PCAN-USB interface](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1)

## Setup instructions
### Prerequisites
#### 1. PCAN-USB driver
Download, build, and install PCAN-USB driver for Linux "libpcan"
```bash
tar -xzvf peak-linux-driver-x.x.tar.gz
cd peak-linux-driver-x.x
make NET=NO
sudo make install
```
##### Troubleshooting
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

#### 2. `libpcanbasic`
Download, build, and install PCAN-Basic API for Linux: libpcanbasic
```bash
tar -xzvf PCAN_Basic_Linux-x.x.x.tar.gz
cd PCAN_Basic_Linux-x.x.x/pcanbasic
make
sudo make install
```

#### 3. `libBHand` grasping library
Download, build, and install Grasping Library for Linux, "libBHand": Grasping_Library_for_Linux
```bash
unzip LibBHand_{32|64}.zip
cd libBHand_{32|64}
sudo make install
sudo ldconfig
```


### Installation

Besides the dependencies mentioned above, ensure the following dependencies are installed before building the package:

- pybind11
- Eigen3


After installing the dependencies, you can build and install the package with:

```
pip install .
```

## Usage

See `example.py`

## TODO

- thread-safety