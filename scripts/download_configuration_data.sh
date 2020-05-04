#!/bin/bash

cd demo
wget https://cloud.hipert.unimore.it/s/LMF4KnmzBYz8tMK/download -O class-aggregator_data.zip
unzip -d class-aggregator_data class-aggregator_data.zip
rm class-aggregator_data.zip
mkdir data
mv class-aggregator_data/* data/
rm -r class-aggregator_data
cd data
mkdir class_edge_log
mkdir class_aggregate_log