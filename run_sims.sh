#!/bin/bash
cd /home/pedro/repos/ns-3-allinone/ns-3.36 
./ns3 run --command-template \
	"%s --routing-protocol=1 --out-dir=/home/pedro/simulacoes --x-size=5 --y-size=5 --clients=1 --time=300 --packet-size=512" \
	scratch/mesh_simulation_adr.cc
./ns3 run --command-template \
        "%s --routing-protocol=2 --out-dir=/home/pedro/simulacoes --x-size=5 --y-size=5 --clients=1 --time=300 --packet-size=512" \
        scratch/mesh_simulation_adr.cc
./ns3 run --command-template \
        "%s --routing-protocol=1 --out-dir=/home/pedro/simulacoes --x-size=10 --y-size=10 --clients=3 --time=300 --packet-size=512" \
        scratch/mesh_simulation_adr.cc
./ns3 run --command-template \
        "%s --routing-protocol=2 --out-dir=/home/pedro/simulacoes --x-size=10 --y-size=10 --clients=3 --time=300 --packet-size=512" \
        scratch/mesh_simulation_adr.cc \
./ns3 run --command-template \
        "%s --routing-protocol=1 --out-dir=/home/pedro/simulacoes --x-size=10 --y-size=10 --clients=3 --time=300 --packet-size=32" \
        scratch/mesh_simulation_adr.cc
./ns3 run --command-template \
        "%s --routing-protocol=2 --out-dir=/home/pedro/simulacoes --x-size=10 --y-size=10 --clients=3 --time=300 --packet-size=32" \
        scratch/mesh_simulation_adr.cc
./ns3 run --command-template \
        "%s --routing-protocol=1 --out-dir=/home/pedro/simulacoes --x-size=25 --y-size=25 --clients=5 --time=300 --packet-size=512" \
        scratch/mesh_simulation_adr.cc
./ns3 run --command-template \
        "%s --routing-protocol=2 --out-dir=/home/pedro/simulacoes --x-size=25 --y-size=25 --clients=5 --time=300 --packet-size=512" \
        scratch/mesh_simulation_adr.cc
