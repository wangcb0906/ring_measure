# Cross-Compiling Point Cloud Library (PCL) for Windows using Arch Linux WSL

This repository documents the environment setup and cross-compilation workflow for building the Point Cloud Library (PCL) and its dependencies natively on Arch Linux (via WSL) and cross-compiling them for Windows (x86_64-w64-mingw32). Note it is impossible to cross-compile PCL with CUDA, GPU, etc.

---

## Prerequisites & Environment Setup

### 1. Install Arch Linux on WSL

``` shell
wsl --install archlinux
```

### 2. Initialize Pacman & Create User

``` shell
pacman-key --init
pacman-key --populate
pacman -Syu
```

#### 2.1 Set root password

``` shell
passwd
```

#### 2.2 Install nano and configure sudoers

``` shell
pacman -S nano
export EDITOR=nano
visudo
```

Uncomment: %wheel ALL=(ALL:ALL)

#### 2.3 Create a regular user

``` shell
useradd -m -s /bin/bash username
passwd username
usermod -aG wheel username
exit
```

### 3. Configure WSL Default User

Run from Windows PowerShell to set the default user:

``` shell
wsl -d archlinux -u root -e bash -c "echo -e '[user]\ndefault=username' >> /etc/wsl.conf"
wsl --shutdown
```

---

## Git & SSH Configuration

``` shell
sudo pacman -Syu openssh
ssh-keygen -t ed25519 -C "username@mail.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

## Display public key to add to GitHub

``` shell
cat ~/.ssh/id_ed25519.pub
```
See in the internet how to add public key to your Github account.

## Clone the project

Clone the project to your *$HOME* because we configurate the path of the toolchain based on $home. Otherwise, you have to reconfigure it when installing packages.
``` shell
git clone ...
```

---

## Installing Dependencies

Install baseline development tools, native libraries, and MinGW cross-compilers:

``` shell
wsl
sudo pacman -S --needed base-devel git gcc mingw-w64-gcc cmake
sudo pacman -S boost eigen flann qhull openmp which spdlog opencv imagemagick pkgconf
```

---

## Building Native Dependencies

Create a workspace directory:

``` shell
mkdir -p ~/software && cd ~/software
```

### 1. Argparse

``` shell
git clone https://github.com/p-ranav/argparse.git
cd argparse && mkdir build && cd build
cmake -DARGPARSE_BUILD_SAMPLES=on -DARGPARSE_BUILD_TESTS=on ..
make && sudo make install
cd ~/software/argparse && mkdir build-mingw && cd build-mingw
cmake .. \
  -DCMAKE_SYSTEM_NAME=Windows \
  -DCMAKE_C_COMPILER=x86_64-w64-mingw32-gcc \
  -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++ \
  -DCMAKE_INSTALL_PREFIX=/usr/x86_64-w64-mingw32
make -j && sudo make install
```

### 2. Inifile-cpp

cd ~/software
git clone https://github.com/Rookfighter/inifile-cpp.git
cd inifile-cpp && mkdir build && cd build
cmake .. && sudo make install
cd ~/software/inifile-cpp && mkdir build-mingw && cd build-mingw
cmake .. \
  -DCMAKE_SYSTEM_NAME=Windows \
  -DCMAKE_C_COMPILER=x86_64-w64-mingw32-gcc \
  -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++ \
  -DCMAKE_INSTALL_PREFIX=/usr/x86_64-w64-mingw32
make -j && sudo make install

### 3. Native PCL

``` shell
cd ~/software
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout 26cb0b909
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j10 && sudo make install -j20
```

---

## Building MinGW Dependencies

Skip This part if you don't intend to deploy on Windows

### 1. LZ4 (MinGW)

``` shell
cd ~/software
git clone https://github.com/lz4/lz4.git
cd flann
cmake -S build/cmake -B build-mingw \
    -DCMAKE_TOOLCHAIN_FILE="$HOME/ring_measure/cmake/mingw-w64-x86_64.cmake" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/x86_64-w64-mingw32 \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_STATIC_LIBS=OFF
    -DCMAKE_INSTALL_PREFIX=/usr/x86_64-w64-mingw32
cmake --build build-mingw -j10 && sudo cmake --install build-mingw -j20
```

### 2. FLANN (MinGW)

``` shell
cd ~/software
git clone -b 1.9.2 https://github.com/flann-lib/flann.git
cd flann
sed -i 's/VERSION 2.6/VERSION 3.5/g' CMakeLists.txt

export PKG_CONFIG_SYSROOT_DIR=/usr/x86_64-w64-mingw32
export PKG_CONFIG_LIBDIR=/usr/x86_64-w64-mingw32/lib/pkgconfig:/usr/x86_64-w64-mingw32/share/pkgconfig
cmake -B build-mingw \
    -DCMAKE_TOOLCHAIN_FILE="$HOME/ring_measure/cmake/mingw-w64-x86_64.cmake" \
    -DBUILD_MATLAB_BINDINGS=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/x86_64-w64-mingw32
cmake --build build-mingw -j10 && sudo cmake --install build-mingw -j20
```

### 3. Qhull (MinGW)

``` shell
cd ~/software
git clone https://github.com/qhull/qhull.git
cd qhull
git checkout 2020.2
sed -i 's/VERSION 2.6/VERSION 3.5/g' CMakeLists.txt

cmake -B build-mingw \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="$HOME/ring_measure/cmake/mingw-w64-x86_64.cmake" \
    -DBUILD_SHARED_LIBS=ON \
    -DUSE_EXTERNAL_LZ4=ON \
    -DUSE_HDF5=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/x86_64-w64-mingw32 \
cmake --build build-mingw --parallel 10 && sudo cmake --install build-mingw --parallel 20
```

---

## AUR & MinGW Package Helpers

``` shell
sudo nano /etc/locale.gen
Uncomment en_US.UTF-8 UTF-8
sudo locale-gen
```

Reboot wsl in *windows* terminal

``` shell
wsl --shutdown
wsl
```

Install `yay` to access AUR packages for MinGW-w64 dependencies:

``` shell
git clone https://aur.archlinux.org/yay-bin.git
cd yay-bin && makepkg -si


yay -S mingw-w64-spdlog mingw-w64-boost mingw-w64-eigen mingw-w64-qhull
yay -S mingw-w64-jsoncpp
# Installing opencv for mingw could take very long time and desire interactive confirms.
# Try again if it does not work.
export MAKEFLAGS="-j10"
yay -S mingw-w64-opencv
```

---

## Cross-Compiling PCL for Windows

``` shell
cd ~/software/pcl
cmake -B build-mingw \
    -G "Unix Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE="$HOME/ring_measure/cmake/mingw-w64-x86_64.cmake" \
    -DCMAKE_INSTALL_PREFIX=/usr/x86_64-w64-mingw32 \
    -DCMAKE_BUILD_TYPE=Release \
    -DPCL_SHARED_LIBS=OFF \
    -DPCL_NO_PRECOMPILE=ON \
    -DPCL_ONLY_CORE_POINT_TYPES=OFF \
    -DWITH_CUDA=OFF \
    -DWITH_VTK=OFF \
    -DWITH_QT=OFF \
    -DBUILD_apps=OFF \
    -DBUILD_examples=OFF \
    -DBUILD_global_tests=OFF \
    -DWITH_DOCS=OFF
cmake --build build-mingw --parallel 10 && sudo cmake --install build-mingw --parallel 20
```