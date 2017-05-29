# Intel® Media SDK tools
This repo contains utilities and tools to support Intel® Media SDK development.

# Tools
Here is a list of tools with build and usage instructions.

## metrics_calc_lite
This tool calculates objective metrics (`PSNR`, `SSIM`) for raw video files.

Tool supports Intel® Integrated Performance Primitives (Intel® IPP) optimizations, to enable it pass `-DUSE_IPP=ON` to `cmake`. It is `OFF` by default.
In order to use IPP you should have `IPP_ROOT` variable point to a directory with `IPP`'s `include` and `lib` folders.

Build on Linux:
```sh
cd <repo_name>/metrics_calc_lite
cmake .
make
```

Build on Windows (for Microsoft Visual Studio 2015*):
```
cd <repo_name>\metrics_calc_lite
cmake -G "Visual Studio 14 2015 Win64" .
```

Usage (to see full help run `metrics_calc_lite` without parameters):
```
metrics_calc_lite.exe <Options> <metric1> ... [<metricN>]... <plane1> ...[<planeN>] ...
Possible metrics are: psnr, apsnr, ssim
Possible planes are: y, u, v, overall, all
Required options are:
    -i1 <filename> - name of first file to compare
    -i2 <filename> - name of second file to compare
    -w  <integer> - width of sequences pixels
    -h  <integer> - height of sequences pixels

NOTES:    1. Different chromaticity representations can be compared on Y channel only.
          2. In case of 10 bits non-zero values must be located from bit #0 to bit #9.
             If such bits are located from bit #6 to bit #15 use parameters "-rshift1 6 -rshift2 6"
Example:
    metrics_calc_lite.exe -i1 foreman.yuv -i2 x264_decoded.yuv -w 352 -h 288 psnr all ssim y
    metrics_calc_lite.exe -i1 foreman.yuv -i2 x264_decoded.yuv -w 352 -h 288 -nopfm -st i420p -fs 20 0 1 psnr y
```

# See also
[Intel® Media SDK repo](https://github.com/Intel-Media-SDK/MediaSDK)
