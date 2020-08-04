**Status**: Heavy development (you might run into crashes as the library is being developed, sorry in advance).

# tysocDart: A locomotion toolkit, with Dart as physics backend.

[![Build Status](https://travis-ci.com/wpumacay/tysocDart.svg?branch=master)](https://travis-ci.com/wpumacay/tysocDart)

<!--![gif-demo-sample](https://media.giphy.com/media/ZDEAQSUraLao0fOhHi/giphy.gif)-->

This is an instance of the [**loco**](https://github.com/wpumacay/tysocCore) framework for locomotion, 
using [**DART**](http://dartsim.github.io/) as physics backend. As explained in the core repository, the idea
is to provide a kind of abstraction layer on top of various physics backends, and allow you to just
focus on making your experiment regardless of the details of each specific backend.

I will be adding more documentation as I develop the library, and sorry in advance as I might forget 
to update the docs from time to time. However, one main objective is to write comprehensive documentation, 
and I will be doing it on the go. If you have any suggestions/issues, just post an issue or contact me 
at wpumacay@gmail.com .

## Setup (WIP)

### Requirements

#### Ubuntu >= 16.04

Install dependencies for the visualizer:

```bash
# build-tools
sudo apt install make cmake pkg-config
# dependencies for the visualizer
sudo apt install libglfw3-dev libglew-dev
```

Install dependencies for `DART-sim`:

```bash
# DART-sim dependencies
sudo apt install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev libopenscenegraph-dev libbullet-dev
```

Install fork of `DART-sim` to the `~/.dart` location:

```bash
# Get our fork of dart-sim (adds some helper functionality)
git clone https://github.com/wpumacay/dart
# Build dart-sim in release-mode and using shared-libraries
cd dart && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=~/.dart
make -j4
# Install dart-sim to the ~/.dart location in your system
make install
# Add libraries to library PATH (add this line to the bottom of your .bashrc file)
export LD_LIBRARY_PATH="~/.dart/lib:$LD_LIBRARY_PATH"
```

### Building

#### Ubuntu >= 16.04

```bash
# clone this repository (comes without dependencies)
git clone https://github.com/wpumacay/tysocDart.git
# clone the third_party dependencies
cd tysocDart && ./scripts/setup_dependencies.sh
# build the project
mkdir build && cd build && cmake .. && make -j4
```