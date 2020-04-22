# class-aggregator

class-aggregator provides the software to deduplicate and aggregate the edge information of a smart city in the context of the European Project CLASS (H2020, G.A. 780622)

## Dependencies

This projects depends on: 

  * CUDA 10.0
  * CUDNN 7.603
  * TENSORRT 6.01
  * OPENCV 3.4
  * yaml-cpp 0.5.2 
  * Eigen
  * GDal

```
sudo apt-get install -y libeigen3-dev \
                        python3-matplotlib \
                        python-dev \
                        libgdal-dev \
                        libcereal-dev \
                        libyaml-cpp-dev
```

required for tkCommon
```
sudo apt-get install -y libgles2-mesa-dev libglew-dev libmatio-dev libpcap-dev
bash scripts/install-glfw-3.3.sh
```

## How to compile this repo

Build with cmake.
```
git clone gitlab@git.hipert.unimore.it:dsapienza/class-aggregator.git
cd class-aggregator
mkdir build
cd build
cmake ..
make
```

## How to initialize or update submodule

```
git submodule update --init --recursive     # initialize
git submodule update --remote --recursive   # update all
```