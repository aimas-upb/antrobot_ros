# Ant Robot SOCs Setup
The steps described in this section will guide through installing the os and tools for rpi4, 
jetson nano A02/B01, realsense, rplidar with support for ROS noetic and pytorch.
## I. Jetson Nano Setup.
### A. Installing Ubuntu 20.04 for Jetson Nano v3 (only for B01)
1. Get an SD card with 32GB (minimal)/64GB recommended.
2. Download the [ubuntu 20.4 image](https://drive.google.com/file/d/1GyHptqenyuyNOPx8tE4ria6-G9Eehpmm/view?usp=sharing)
3. Flash the image on the SD card with the Imager or balenaEtcher.
4. Insert the SD card into Jetson Nano.
5. User: `jetson`, Password: `jetson`, Hostname: `nano`.
6. Enjoy!

Notes: 
1. The indicated image comes with the following pre-installed frameworks:
   - OpenCV 4.5.3
   - TensorFLow 2.4.1
   - Pytorch 1.9.0
   - TorchVision 0.10.0
   - TeamViewer aarch64 15.24.5

2. If the SD card is bigger than 32GB you can inflate the image use `gparted` to inflate it.
If this doesn't work for simply use `parted` after booting the nano.  

3. Do not install Chromium as it will interfere with the Snap installation. Use the preinstalled Morzilla Firefox.

4. Do not install JTop! It disrupts your vulkan lavapipe which is always active during your Ubuntu sessions.

5. Many CUDA related software needs gcc version 8. We have installed gcc and g++ version 8 alongside the preinstalled version 9.
You can select your choice with `sudo update-alternatives --config gcc` and `sudo update-alternatives --config g++`

6. You may encounter issues when upgrading (`sudo apt-get upgrade`) this Ubuntu 20.04 version. 
It has to do with a conflicting /etc/systemd/sleep.conf file, which blocks the upgrade. 
Follow the instructions [here](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html#upgrade) to correct the  issue.
If they don't work for you do the following:
   - `sudo apt --fix-missing update`
   - `sudo apt update`
   - `sudo apt install -f`
   
### B. Installing Ubuntu 20.04 through distribution upgrade from 18.04
1. Install the official ubuntu 18.04 image (see [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-2gb-devkit))
2. Insert the sd card into the nano and boot
3. Update and upgrade:

`sudo apt update;sudo apt upgrade -y`
4. Install latest jetpack:

`sudo apt install nvidia-jetpack`
5. Remove Chromium browser, upgrade and get rid of unused packages:
```bash
sudo apt remove --purge chromium-browser chromium-browser-l10n
sudo apt update
sudo apt upgrade
sudo apt autoremove
```
If durring the upgrade you encounter an error about the nvidia-l4t-bootlader do the following:
   
   a. `sudo mv /var/lib/dpkg/info/ /var/lib/dpkg/backup/ #move /var/lib/info/`
   b. `sudo mkdir /var/lib/dpkg/info/ #create new /var/lib/dpkg/info`
   c. `sudo apt-get update #update the source list`
   d. `sudo apt-get -f install #force install to correct the problem` 
   e. `sudo mv /var/lib/dpkg/info/* /var/lib/dpkg/backup/ #Move the new structure dpkg/info to old info`
   f. `sudo rm -rf /var/lib/dpkg/info #remove the new dpkg structure folder`
   g. `sudo mv /var/lib/dpkg/backup/ /var/lib/dpkg/info/ #back the old`
   
6. Install nano editor:.

`sudo apt install nano`

7. Next, you need to enable distribution upgrades in the update manager by setting prompt=normal in the `/etc/update-manager/release-upgrades` file.
As usual, close with `<Ctrl> + <X>, <Y>` and `<Enter>`.

`sudo nano /etc/update-manager/release-upgrades`

8. Change `Prompt=never` to `Prompt=normal`, save and exit.
9. With the update manager set, we need to refresh the software database again. Once done, you can reboot.

```bash
sudo apt update
sudo apt dist-upgrade
sudo reboot
```
10. With all preparations made, it's time for the upgrade to Ubuntu 20.04. 
It will take several hours. Unfortunately, some input is required throughout the procedure as there are questions to be answered. 
Check your screen now and then. Answer all questions with the suggested default value.

`sudo do-release-upgrade`
11. When the process finishes, **do not reboot!**
12. First, check that `WaylandEnable=false` is uncommented in the `/etc/gdm3/custom.conf` file.
13. Uncomment `Driver "nividia"` in the `/etc/X11/xorg.conf` file.
14. Finally, reset the upgrade manager to never (`Prompt=never`) and now reboot.

```bash
sudo nano /etc/gdm3/custom.conf
sudo nano /etc/X11/xorg.conf
sudo nano /etc/update-manager/release-upgrades
sudo reboot
```
15. Update, upgrade and auto-remove:
```bash
sudo apt update
sudo apt upgrade
sudo apt autoremove
```
16. Next, remove an annoying circular symbolic link in `/usr/share/applications` that makes the same app appear 86 times in your software overview.
```bash
# remove circular symlink
sudo rm /usr/share/applications/vpi1_demos
# remove distorted nvidia logo in top bar
cd /usr/share/nvpmodel_indicator
sudo mv nv_logo.svg no_logo.svg
```
17. Re-enable the original NVIDIA repositories, which were disabled during the upgrade. 
In the folder `/etc/apt/sources.list.d/` you will find the five files that needed to be changed. 
Open each file with `sudo` nano and remove the hash in front of the line to activate the repository.

18. Again, Update, upgrade and auto-remove:
```bash
sudo apt update
sudo apt upgrade
sudo apt autoremove
```
19. Some software packages, especially the CUDA software, requires a gcc version 8. 
We shall install this version besides the already available version 9. 
With a simple command, you can now switch between the two versions. 
The gcc compiler is always accompanied by the corresponding g++ compiler. 
The latter will also be installed.
```bash
# install gcc and g++ version 8
sudo apt install gcc-8 g++-8
# setup the gcc selector
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
# setup the g++ selector
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
# if you want to make a selection use these commands
sudo update-alternatives --config gcc
sudo update-alternatives --config g++
```
20. You may run into problems upgrading Ubuntu 20.04 on your Jetson Nano after a while. 
The Software Updater cannot install all the packages listed. 
```bash
sudo apt --fix-broken
sudo apt intall -f
```
### C. Pytorch/Torchvision Build and Install
Unfortunately, to get pytorch to work with the jetson and particular version of python3 we have to build it from scratch.
For python3.6 there are a couple of *.whl that are already generated and you can follow the steps [here](https://qengineering.eu/install-pytorch-on-jetson-nano.html)
to install them. Whereas, the steps described here will generate our own wheel with support for torch 1.10 / cuda 10.2 versions. 
Note, the whole procedure takes about 8 - 10 hours on an overclocked Jetson Nano and about 6 hours when compling natively.
To generate the wheel natively see [Jetson Nano Pytorch Build Docker](https://github.com/dnovischi/jetson-nano-pythorch).
Most important, when **upgrading** modify the version number in the file `~/pytorch/version.txt` from 1.7.0 to 1.7.1, if you install version PyTorch 1.7.1;
or similarly for other versions. It seems the developers forget to adjust this number on GitHub. 
If you don't change the number, you end up with a torch-1.7.0a0-cp36-cp36m-linux_aarch64.whl wheel, suggesting you have the old version still on your machine.
This is not required on a fresh install.

1. Install pip:

`sudo apt install python3-pip`
2. First, make sure everything is updated: 
```bash
sudo apt update
sudo apt upgrade
```
3. Install package dependecies:
```bash
sudo apt install ninja-build git cmake
sudo apt install libjpeg-dev libopenmpi-dev libomp-dev ccache
sudo apt install libopenblas-dev libblas-dev libeigen3-dev
sudo pip3 install -U --user wheel mock pillow
sudo -H pip3 install testresources
   ```
4. Upgrade setup tools:
```bash
sudo -H pip3 install -U setuptools
sudo -H pip3 install scikit-build
python3 -m pip install setuptools==58.3.0   
```
5. Download pytorch source with all its libraries:
```bash
git clone -b v1.10.0 --depth=1 --recursive https://github.com/pytorch/pytorch.git
cd pytorch
```
6. Install required python dependecies:
```bash
sudo pip3 install -r requirements.txt
```
7. Again, make sure everything is updated:
```bash
sudo apt update
sudo apt upgrade
```
8. Install `dphys-swapfile`
```bash
sudo apt install dphys-swapfile
```
9. Enlarge swap boundary in `/sbin/dphys-swapfile` by setting `CONF_MAXSWAP=4096`.
10. Set the required swap memory size in `/etc/dphys-swapfile` by setting `CONF_SWAPSIZE=4096`
11. Reboot `sudo reboot`
12. Install clang and clang-8:
```bash
sudo apt install clang
sudo apt install clang-8
```
13. Update clang alternatives and configure clang-8:
```bash
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-8 100
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-8 100
sudo update-alternatives --config clang # select clang 8
```
14. Next, modify the PyTorch code you just downloaded from GitHub. 
All alterations limit the maximum of CUDA threads available during runtime. 
There are three places which need our attention.
    - Edit `~/pytorch/aten/src/ATen/cpu/vec/vec256/vec256_float_neon.h`, 
      add `#if defined(__clang__) ||(__GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ > 3))` and `#endif`
      around line 28 such that the contents of the file should look similar to the following:
      ```c
      //...
      // Right now contains only aarch64 implementation.
      // Due to follow two reasons aarch32 is not currently supported.
      // 1. Due to difference in ISA been aarch32 and aarch64, intrinsics
      //    that work for aarch64 dont work for aarch32.
      // 2. Android NDK r21 has problems with compiling aarch32.
      //    Clang seg faults.
      //    https://github.com/android/ndk/issues/1248
      //    https://bugs.llvm.org/show_bug.cgi?id=45824
      // Most likely we will do aarch32 support with inline asm.
      #if defined(__aarch64__)
      #if defined(__clang__) ||(__GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ > 3))
      #ifdef __BIG_ENDIAN__
      #error "Big endian is not supported."
      #endif
      #endif
      //...
      ```
    - Edit `~/pytorch/aten/src/ATen/cuda/CUDAContext.cpp` by adding the extra line `device_prop.maxThreadsPerBlock = device_prop.maxThreadsPerBlock / 2;` around line 26
      such that the file should look similar to the following:
      ```c
      void initDeviceProperty(DeviceIndex device_index) {
      cudaDeviceProp device_prop;
        AT_CUDA_CHECK(cudaGetDeviceProperties(&device_prop, device_index));
        device_prop.maxThreadsPerBlock = device_prop.maxThreadsPerBlock / 2; // <--- New line added
        device_properties[device_index] = device_prop;
      }
 
      ```
    - Edit `~/pytorch/aten/src/ATen/cuda/detail/KernelUtils.h` and change the `CUDA_NUM_THREADS` to equal `512` 
      such that it looks similar to the following:
      ```c
      // Use 1024 threads per block, which requires cuda sm_2x or above
      constexpr int CUDA_NUM_THREADS = 512; //1024;
      ```
15. Set the build options and do the build:
```bash
# set NINJA parameters
export BUILD_CAFFE2_OPS=OFF
export USE_FBGEMM=OFF
export USE_FAKELOWP=OFF
export BUILD_TEST=OFF
export USE_MKLDNN=OFF
export USE_NNPACK=OFF
export USE_XNNPACK=OFF
export USE_QNNPACK=OFF
export USE_PYTORCH_QNNPACK=OFF
export USE_CUDA=ON
export USE_CUDNN=ON
export TORCH_CUDA_ARCH_LIST="5.3;6.2;7.2"
export USE_NCCL=OFF
export USE_SYSTEM_NCCL=OFF
export USE_OPENCV=OFF
export MAX_JOBS=4
# set path to ccache
export PATH=/usr/lib/ccache:$PATH
# set clang compiler
export CC=clang
export CXX=clang++
# create symlink to cublas
sudo ln -s /usr/lib/aarch64-linux-gnu/libcublas.so /usr/local/cuda/lib64/libcublas.so
# start the build
python3 setup.py bdist_wheel
```

16. Install the wheel:
```bash
cd ./dist
sudo -H pip3 install torch-1.10.0a0+git36449ea-cp38-cp38-linux_aarch64.whl
```
17. Check the installation, e.g.:
```
$ python3

>>> import torch
>>> torch.__version__
'1.10.0a0+git36449ea'
>>> print(torch.rand(5,4))
tensor([[0.7016, 0.8775, 0.2243, 0.3520],
        [0.0253, 0.2327, 0.4430, 0.7621],
        [0.5847, 0.9329, 0.1710, 0.4259],
        [0.0877, 0.0088, 0.9197, 0.6599],
        [0.4864, 0.3551, 0.9600, 0.9442]])
>>> print(torch.hypot(torch.tensor([1.]),torch.tensor([1.0])))
tensor([1.4142])
```
18. Now we also need torchvision (other extension follow a similar process). 
First clone the repository for the compatible pytorch version.
```bash
git clone -b v0.11.1 https://github.com/pytorch/vision.git
cd vision    
```
19. Do the build and generate the distribution wheel:
```bash
python3 setup.py build
```
20. Install torchvision: 
```bash
cd dist/
sudo -H pip3 install torchvision-0.11.0a0+fa347eb-cp38-cp38-linux_aarch64.whl
```
21. Check that everything is ok:
```bash
$ python3

Python 3.8.10 (default, Nov 26 2021, 20:14:08) 
[GCC 9.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import torchvision
>>> torchvision.__version__
'0.11.0a0+fa347eb
```
22. Clean up after the fact:
```bash
# remove the dphys-swapfile now
$ sudo /etc/init.d/dphys-swapfile stop
$ sudo apt-get remove --purge dphys-swapfile
# just a tip to save some space
$ sudo rm -rf ~/pytorch
```
#### D. SSH Setup
1. Install your ssh key on the nano:

`ssh-copy-id -i jetson@nano.local`
2. Login:

`ssh jetson@nano.local`

### E. Wi-Fi Setup/Troubleshooting
TODO: Add Wi-Fi connect via nmcli and text for troubleshooting.

   ```bash
    sudo apt-get update
    sudo apt-get install git linux-headers-generic build-essential dkms
    git clone https://github.com/pvaret/rtl8192cu-fixes.git
    sudo dkms add ./rtl8192cu-fixes
    sudo dkms install 8192cu/1.11
    sudo depmod -a
    sudo cp ./rtl8192cu-fixes/blacklist-native-rtl8192.conf /etc/modprobe.d/
    sudo echo options rtl8xxxu ht40_2g=1 dma_aggregation=1 | sudo tee /etc/modprobe.d/rtl8xxxu.conf
    sudo iw dev wlan0 set power_save off
    sudo reboot now
    # Set AutomaticLoginEnable = true  AutomaticLogin = user // put your user name here e.g. jetson
    sudo nano /etc/gdm3/custom.conf 
   ``` 

### F. Realsense Setup

On the Jetson Nano, first clone the Realsense installation repo (forked from JetsonHacks):
```bash
git clone https://github.com/dnovischi/installLibrealsense.git
```
#### Install from available packages:
Simply, run the install script in the repository:
```
cd installLibrealsense/
sudo ./installLibrealsense.sh
```
**Note:** If this fails, it means that the packages aren't available for your distribution and need to be compiled from source.
#### Build and install from source:
```bash
cd installLibrealsense/
./buildLibrealsense.sh  
 ```
**Note:** ROS Realsense support packages can be installed with:

```sudo apt install ros-$ROS_DISTRO-realsense2-camera ```

### II. Raspberry PI4 Install Ubuntu 20.04 Server 
 
[//]: # (TODO: Add RPI4 setput guide and remove notes below. )


Notes for building torch, torchvision and torchaudio
```
# TORCHVER=v1.7.1
# VISIONVER=v0.8.2
# AUDIOVER=v0.7.2

# git clone -b ${TORCHVER} --recursive https://github.com/pytorch/pytorch
# cd /pytorch \
    && sed -i -e "/^if(DEFINED GLIBCXX_USE_CXX11_ABI)/i set(GLIBCXX_USE_CXX11_ABI 1)" \
                 CMakeLists.txt \
    && pip3 install -r requirements.txt \
    && python3 setup.py build \
    && python3 setup.py bdist_wheel \
    && cd ..

# pip3 install /pytorch/dist/*.whl \

# git clone -b ${VISIONVER} https://github.com/pytorch/vision.git
# cd /vision \
    && python3 setup.py build \
    && python3 setup.py bdist_wheel \
    && cd ..

# git clone -b ${AUDIOVER} https://github.com/pytorch/audio.git
# cd /audio \
    && apt-get install -y sox libsox-dev \
    && python3 setup.py build \
    && python3 setup.py bdist_wheel \
    && cd ..
```
