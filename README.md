<h1>Structure from Motion</h1>
<h2>Content</h2>
This repository contains the code mentioned in my master's thesis. The thesis itself, i.e. Thesis.pdf also is part of this repository.

<p>
As the thesis, the repository is split into a matlab section and the C++ implementation for the NVIDIA Tegra K1 board.
</p>
<h2>Matlab</h2>
Look at matlab/sfm.m to find the main part of the SfM implemantation for Matlab. The first few lines handle the configuration of the approach.
The variable names should be self-explaining, the most important being
```
views = ...;
verbose = ...;
visualize = ...;
```
<p>
The views are imported using the Calibration Toolbox of Jean-Yves Bouguet [http://www.vision.caltech.edu/bouguetj/calib_doc/] which is very well documented.
Following the tutorial of the toolbox the images in matlab/calib_f0 were imported and the calibratio data saved into matlab/calib_f0/Calib_Results.mat
which is then loaded by the sfm.m script. The views to apply the SfM algorithm to are then selected by a list of view pairs
in brackets. I tried to comment the script to be as easily understandable as possible
</p>

<h2>C++ implementation</h2>
<p>
The C++ implementation of the Matlab algorithm can be found in tegra/ with main.cpp being the entry point for interested readers - as expected.
The program can be compiled using the Makefile. Due to time restraints I did not use autoconf or similar, therefore the Makefile is hardcoded and
might have to be adopted to your system. Looking into the Makefile you can see several targets, the primary one being <i>$PROJECT</i>, which is automatically called, when
running make. The <i>tegra</i> target is used to build for the Tegra platform. Consider that
a NVIDIA graphics card and the CUDA toolbox is needed to build the application. Additionally the following dependencies exist
<ul>
<li>OpenCV 3.0.0-beta (with non-free features plugin) (I built the library from source from tag: 3.0.0-beta (hash: ae4cb57), please refer to READMEs and tutorials for instructions on how to build it for your platform)</li>
<li>CUDA Toolbox 6.5</li>
<li>libPCL 1.8 (I built the library from source from master (hash: d0af48b)) has dependencies itself, e.g., VTK and libusb</li>
<li>libeigen3-dev (in Debian wheezy repositories)</li>
<li>libboost (in Debian wheezy repositories)</li>
</ul>
</p>

<p>
Consider adding the self compiled libraries to your library search path, or use
```
export LD_LIBRARY_PATH=/opt/opencv3/lib/:/opt/pcl/lib:/opt/cuda-6.5/lib64
```
so the application can find the necessary libraries.
</p>
<p>
Running the application with
```
./sfm --help
```
gives a list of options implemented. You might want to try the <i>-c 1</i> switch if you are using a laptop with integrated webcam
to tell the application to use the external camera. Using the <i>-l</i> switch opens the live stream in which the matches can
be shown using the [n] key, when the window is in focus. The [q] key exits.
</p>
<p>
Don't forget to run the camera.sh script before running the application. The script sets the camera's properties, especially it disables the auto focus which would change the calibration parameters. The script can be called
using the camera number as parameter (./camera.sh -c 1).
</p>
