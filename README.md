# Ant Robot v2 ROS Package
## Licence
LGPLv2
TODO:

## Ant Robot SOCs Setup
The steps described in this section will guide through installing the os and tools for rpi4, 
jetson nano A02/B01, realsense, rplidar with support for ROS noetic and pythorch.
#### Installing ubuntu 20.04.
##### A. Using prebuilt image for Jetson Nano v3 (B01)
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
##### B. Distribution upgrade from 18.04
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
6. Install nano editor:.

  `sudo apt install nano`

7. Next, you need to enable distribution upgrades in the update manager by setting prompt=normal in the `/etc/update-manager/release-upgrades` file.
As usual, close with <Ctrl>+<X>, <Y> and <Enter>.

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
11. When the process finishes, do not reboot.
12. First, check that `WaylandEnable=false` is uncommented in the `/etc/gdm3/custom.conf` file.
13. Uncomment `Driver "nividia"` in the `/etc/X11/xorg.conf` file.
14. Finally, reset the upgrade manager to never (`Prompt=never`) and reboot.

   ```bash
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
20. The latter will also be installed.
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
21. You may run into problems upgrading Ubuntu 20.04 on your Jetson Nano after a while. 
The Software Updater cannot install all the packages listed. 
   ```bash
   sudo apt --fix-broken
   sudo apt intall -f
   ```
#### Pytorch Build and Install
Unfortunately, to get pytorch to work with the jetson and particular version of python3 we have to build it from scratch.
For python3.6 there are a couple of *.whl that are already generated an you can follow the steps [here](https://qengineering.eu/install-pytorch-on-jetson-nano.html)
to install them. Whereas, the steps described here will generate our own wheel with support for torch 1.10 / cuda 10.2 versions. 
Note, the whole procedure takes about 8 hours on an overclocked Jetson Nano.
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
13. Update clang alternatives and confgigure clang-8:
   ```bash
   sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-8 100
   sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-8 100
   sudo update-alternatives --config clang # select clang 8
   ```
14. Next, modify the PyTorch code you just downloaded from GitHub. 
All alterations limits the maximum of CUDA threads available during runtime. 
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

#### SSH Setup
1. Install your ssh key on the nano:

    ` ssh-copy-id -i jetson@nano.local`
2. Login:

    ` ssh jetson@nano.local`

#### Wi-Fi Setup/Troubleshooting

#### Realsense Setup

TODO: Remove below:

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

