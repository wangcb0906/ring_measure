# Project Title

Deployment on Windows with Dll in C#

## How to deploy on Windows

### 1. Build the libraries

``` shell
# Build the Archlinux dependencies, see requirements.md
rm build-win -r
cmake -S . -B build-win -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/mingw-w64-x86_64.cmake"
# Note: Only the printed deployed dependencies are needed to run this algorithm on Windows.
# The others could be omitted.
cmake --build build-win --target deploy -j
```
### 2. Test the example project

#### 2.1 Arrange the file structure and adapt the parameters

``` shell
user@Windows MINGW64 /d/test_cross_compile/example-csharp
$ tree -L 2
|-- ConfigInteropStructs.cs
|-- ParamInteropStructs.cs
|-- Program.cs    # Adapt the parameters accordingly, see comments in the code.
|-- RingCaliMethods.cs
|-- RingCalibration.cs
|-- RingCalibrationTest.csproj
|-- bin    # auto generated
|   `-- Debug
|-- input   # scans
|   |-- ND########_00004_00_R01_20260622_101647_B0.tif
|   |-- ND########_00004_00_R01_20260622_101703_B1.tif
|   `-- ND########_00004_00_R01_20260622_101718_B2.tif
|-- libs    # Built libraries in step 1
|   |-- libcore_shared.dll
|   |-- libfmt.dll
|   |-- libgcc_s_seh-1.dll
|   |-- libgomp-1.dll
|   |-- libjpeg-8.dll
|   |-- liblz4.dll
|   |-- liblzma-5.dll
|   |-- libspdlog.dll
|   |-- libssp-0.dll
|   |-- libstdc++-6.dll
|   |-- libtiff-6.dll
|   |-- libwinpthread-1.dll
|   `-- zlib1.dll
`-- obj    # auto generated
```

#### 2.2 Run *program.cs* in VSCode and check the results 

---

## Deployment Comparison with Linux

### 1. Overview
| Windows | Linux |
| :--- | :--- |
| **Status:** Active / Stable | **Status:** Active / Stable |
| **Size:** 13.1MB | **Size:** 1.2MB without precompiled libs |
| **Environment Details:** | **Environment Details:** |
| * Self-contained dlls | * Dynamic dlls |
| * C++/C# Wrappers | * Headers |
| **Speed:** 3.4s per ring | **Speed**: 8.3s per ring |
| * multi-thread possible | * multi-thread possible |
| * stack 16 bytes aligned |* stack 32 bytes aligned |

---

### 2. Consistency Check
> [!CAUTION]
The results on Windows and Linux are slightly different.
* Possible reasons: 
    * Libraries like Eigen rely heavily on vectorization (SSE, AVX, NEON)
    * Windows (MSVC) and Linux (GCC) compile floating-point math differently.
    * Linux often defaults to aggressive optimizations like -ffast-math
    * In iterative algorithms, tiny floating-point drift can compound into entirely different final results.

### 3. Binary comparisons

``` bash
# md5sum
7c1272be78b6af070c57a7879433f92d  cali/20260622/auto/calibrations/4_ND_20260622_4_08.bin
c9df3c40cfb0e1f57ed31294372c4c5a  build-win/deploy/cali/20260622/auto/calibrations/4_ND_20260622_4_08.bin
a1a9e6f55e6bc557f5af5405044b0e65  cali/20260622/auto/calibrations/4_ND_20260622_4_09.bin
8be66194485421b9fa550581fe25313a  build-win/deploy/cali/20260622/auto/calibrations/4_ND_20260622_4_09.bin
ca59b46db4b1ae8f33abd99212d36377  cali/20260622/auto/calibrations/4_ND_20260622_4_10.bin
b2b7b0c98d72892ae887e3966ccb6471  build-win/deploy/cali/20260622/auto/calibrations/4_ND_20260622_4_10.bin
```

### 4. Numerical comparisons

```diff
- a/cali/20260622/auto/calibrations/4_ND_20260622_4_08.ini
+ b/build-win/deploy/cali/20260622/auto/calibrations/4_ND_20260622_4_08.ini
@@ -4,15 +4,15 @@
 angleI=-50
 angleU=20.5
 armX=534
 armY=743.5
-corrAngleA=-5.631228
+corrAngleA=-5.65552
-corrAngleI=-5.712975
+corrAngleI=-5.71286
-corrAngleU=4.0063748
+corrAngleU=4.01509
-corrXA=-760.45178
+corrXA=-760.149
-corrXI=-760.73969
+corrXI=-760.743
-corrXU=206.29999
+corrXU=206.258
-corrYA=124.09131
+corrYA=124.254
-corrYI=124.68956
+corrYI=124.689
-corrYU=-694.49207
+corrYU=-694.619
 dist=189
 table_h=882
 wedge_h=10
@@ -24,6 +24,6 @@
 scan_res=3200
 unit="mm"
 valid_len=720
 x_pitch=22500
-z_oft=-52428800
+z_oft=-5.24288e+07
 z_pitch=1600
```


```diff
- a/cali/20260622/auto/calibrations/4_ND_20260622_4_09.ini
+ b/build-win/deploy/cali/20260622/auto/calibrations/4_ND_20260622_4_09.ini
@@ -4,15 +4,15 @@
 angleI=-50
 angleU=20.5
 armX=534
 armY=743.5
-corrAngleA=-5.6624589
+corrAngleA=-5.68978
-corrAngleI=-5.7118354
+corrAngleI=-5.71416
-corrAngleU=4.0183029
+corrAngleU=4.05167
-corrXA=-760.06458
+corrXA=-759.723
-corrXI=-760.61877
+corrXI=-760.589
-corrXU=206.25967
+corrXU=206.084
-corrYA=124.28613
+corrYA=124.467
-corrYI=124.85435
+corrYI=124.865
-corrYU=-694.64001
+corrYU=-695.171
 dist=189
 table_h=882
 wedge_h=10
@@ -24,6 +24,6 @@
 scan_res=3200
 unit="mm"
 valid_len=720
 x_pitch=22500
-z_oft=-52428800
+z_oft=-5.24288e+07
 z_pitch=1600
```

```diff
- a/cali/20260622/auto/calibrations/4_ND_20260622_4_10.ini
+ b/build-win/deploy/cali/20260622/auto/calibrations/4_ND_20260622_4_10.ini
# @@ -4,15 +4,15 @@ 
 angleI=-50
 angleU=20.5
 armX=534
 armY=743.5
-corrAngleA=-5.7159657
+corrAngleA=-5.70007
-corrAngleI=-5.6785536
+corrAngleI=-5.72856
-corrAngleU=3.9940414
+corrAngleU=3.97431
-corrXA=-759.39111
+corrXA=-759.593
-corrXI=-761.01672
+corrXI=-760.413
-corrXU=206.3942
+corrXU=206.503
-corrYA=124.63281
+corrYA=124.54
-corrYI=124.63707
+corrYI=124.95
-corrYU=-694.24854
+corrYU=-693.949
 dist=189
 table_h=882
 wedge_h=10
@@ -24,6 +24,6 @@
scan_res=3200
 unit="mm"
 valid_len=720
 x_pitch=22500
-z_oft=-52428800
+z_oft=-5.24288e+07
 z_pitch=1600
```


### 5. Comparison between C# and C++ internal apps

| Scan Location Log (Raw Values) | Corresponding INI Parameters |
| :--- | :--- |
| **Scan Location: IN**<br>`ViewAngle: -50`<br>`AngleCorr: -5.7128596`<br>`PosCorr: (-760.7433, 124.68919)` | <br>`corrAngleI=-5.71286`<br>`corrXI=-760.743`<br>`corrYI=124.689` |
| **Scan Location: OUT**<br>`ViewAngle: -50`<br>`AngleCorr: -5.6555243`<br>`PosCorr: (-760.1488, 124.25366)` | <br>`corrAngleA=-5.65552`<br>`corrXA=-760.149`<br>`corrYA=124.254` |
| **Scan Location: UNDER**<br>`ViewAngle: 20.5`<br>`AngleCorr: 4.015088`<br>`PosCorr: (206.25772, -694.61914)` | <br>`corrAngleU=4.01509`<br>`corrXU=206.258`<br>`corrYU=-694.619` |


## Extensions

The C++ code provides much more APIs than exported ones.
To make more APIs available, see *export.h* and *export.cpp* as an example on the C++ side.
To wrap the exported APIs in C# like built-in ones, see *ConfigInteropStructs.cs*, *RingCaliMethods.cs* and *RingCalibration.cs*.