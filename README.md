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

```bash
sudo apt install make cmake pkg-config
sudo apt install libglfw3-dev libglew-dev
# DART-sim dependencies
sudo apt install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev libopenscenegraph-dev
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