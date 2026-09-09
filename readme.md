# Project Title

Ring Calibration

## Description

This projects calibrates the rings and generate the parameters, include rotational and translational corrections for further measurement.

## Getting Started

### Dependencies

* Archlinux
* g++/gcc
* Python for visualization only.
* [VSCode codelldb](https://github.com/vadimcn/codelldb.git) extension to enable debugging.
* [PCL(Beta Version)](https://github.com/pointcloudlibrary/pcl)
* [Opencv](https://github.com/opencv/opencv.git)
* [SPDLOG](https://github.com/gabime/spdlog.git)
* [argparse](https://github.com/p-ranav/argparse.git)
* [inicpp](https://github.com/Rookfighter/inifile-cpp.git)
* [mingw-gcc](https://packages.msys2.org/packages/mingw-w64-x86_64-gcc) for cross-compilation.
* doxygen

### Installing

#### On Archlinux

```
cmake -B build -DLOG_LEVEL=INFO -DCMAKE_BUILD_TYPE=Release
cmake --build build --target ring_cali -j
```
 
#### On Windows(Cross-compilation)

```
cmake -S . -B build-win -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/mingw-w64-x86_64.cmake"
cmake --build build-win --target ring_cali -j
cmake --build build-win --target deploy -j
```

#### To build the document
```
doxygen
cd latex
pdflatex refman.tex
makeindex refman.idx
pdflatex refman.tex
pdflatex refman.tex
```

### Executing program

* To show help function of each application, run
```
./ring_cali
./ring_cali_eval
```

## Help

* To show help function of each application, run
```
./ring_cali -h
./ring_cali_eval -h
```

* To evaluate calibrations, run
```
python visual/eval_bin.py ./cali/20260622/result_cali_eval/
```

* To enable debuggin mode. 
    * Compile the project with Debug mode.
    * Enable the break point with conditions.
    * Start the debugging mode you want in VScode.
    * Set *show* in the cpp code to 1 to enable a web app to generate an interactive visualization.

* The calibration is structured in the following way:

```
cali/20260622/
├── auto             # calibration with some parameters
├── auto_cali_eval   # folder to evaluate the calibration of the same name
├── auto.ini         # parameter to generate the calibration of the same name
├── config           # all the config file, one for each ring
├── input            # has all the scan images
├── manual           # another calibration
├── manual_cali_eval # another evaluation of calibration
├── manual.ini       # another parameter of calibration
└── system.ini       # the data from robot and camera and correction
```

* Examples of system.ini and ring.ini in *config*
``` 
[Robot] # system.ini
# Configuration for the scan from inside
angleI     = -50
corrAngleI = 0.0
# (855, 1500) -> (92.6623, 1629.76)
corrXI     = -762.3377
corrYI     = 129.76
# Configuration for the scan from outside
angleA     = -50
corrAngleA = 0.0
# (-55, 1500) -> (-816.385, 1625.19)
corrXA     = -761.385
corrYA     = 125.19
# Configuration for the scan from unterneath
angleU     = 20.5  
corrAngleU = 0.0
# (1130, 1320)-> (1326.63, 615.659)
corrXU     = 196.63
corrYU     = -704.341
# Configuration for the robot arm and sensor geometry
armX = 534.0
armY = 743.5
# distance from rotation point to reset point
dist = 189.0
table_h = 892.0
wedge_h = 10.0

[Sensor]
scan_num    = 366
scan_res    = 3200
valid_len   = 720
intervalX   = 0.225
z_pitch     = 1600.0
z_oft       = -5.24288e+07
x_pitch     = 22500.0
unit       = "mm"
mount      = 980.0

[Robot] # ring.ini
xI = 855.0
yI = 1500.0

xA = -55.0
yA = 1500.0

xU = 1130.0
yU = 1320.0

[Scan]
path  = input/20260622
I     = ND\#\#\#\#\#\#\#\#_00003_00_R01_20260622_094827_B0.tif
A     = ND\#\#\#\#\#\#\#\#_00003_00_R01_20260622_094842_B1.tif
U     = ND\#\#\#\#\#\#\#\#_00003_00_R01_20260622_094858_B2.tif
LumaI = ND\#\#\#\#\#\#\#\#_00003_00_R01_20260622_094827_B0_luminance.tif
LumaA = ND\#\#\#\#\#\#\#\#_00003_00_R01_20260622_094842_B1_luminance.tif
LumaU = ND\#\#\#\#\#\#\#\#_00003_00_R01_20260622_094858_B2_luminance.tif

[Ring]
spk_form  = ""
is_turned = false
type = GR
mke_top   = 81
mke_mid   = 45
mke_btm   = 9
da = 990
di = 810
h  = 90
```

## Authors

Chongbiao Wang

## Version History


* 0.1
    * Initial Release