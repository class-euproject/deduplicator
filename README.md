# class-aggregator

This branch has been developed by BSC based from the source code found in the master branch and provided by Università di Modena e Reggio Emilia. The code has been slightly modified in order to be adapted and integrated with the COMPSs workflow, and to do so, a binding between C++ and Python has been introduced in `src/modules.cpp`.

class-aggregator provides the software to deduplicate and aggregate the edge information of a smart city in the context of the European Project CLASS (H2020, G.A. 780622)

## Dependencies

This projects depends on: 

  * yaml-cpp 0.5.2 
  * Eigen
  * GDal

```
sudo apt-get install -y libeigen3-dev \
                        python3-matplotlib \
                        python-dev \
                        python3-dev \
                        libgdal-dev \
                        libcereal-dev \
                        libyaml-cpp-dev \
                        libpthread-stubs0-dev
```


## How to initialize or update submodule

```
git submodule update --init --recursive     # initialize
git submodule update --remote --recursive   # update all
```

## How to compile this repo

Build with cmake.
```
git clone https://github.com/class-euproject/deduplicator.git -b bsc 
cd deduplicator
mkdir build
cd build
cmake ..
make
```

## How to use

### Configuration
Edit "demo/class_aggregator_configuration_file.yaml":

* input_ports:   specify the listening port list
* output_ports:  specify the forwarding port list
* output_ips:    specify the forwarding ip list
* visualization: specify if or not to show the map viewer

N.B the ports number must equal to ips number

### Demo

```
cd build/
./class-aggregator -i path-to-configuration-file 
#./class-aggregator -i ../demo/class_aggregator_configuration_file.yaml
```

### Test
On an other terminal:
```
cd build/
./test-aggregator
```
