# SIVERT V2X simulation framework (NS3 part)


`SIVERT framework` implements a bi-directionally coupled simulations using [Veneris](http://pcacribia.upct.es/veneris/) [[1]](#1) and [NS3](https://www.nsnam.org) simulator, introducing the spatially-consistent Geometry based Stochastic Channel Models (GSCM) with account for antenna patterns using Effective Anerture Distribution Function (EADF). The LTE-V2X module is reused from the implementation kindly shared by [Eckerman et.al.](https://github.com/FabianEckermann/ns-3_c-v2x) [[2]](#2). Coupling is done via bi-directional API using the TCP/IP sockets between simulators using [ZeroMQ](https://zeromq.org). Veneris is extensively extended with GSCM channel modelling. API data structures are implemented using [google flatbuffer](https://google.github.io/flatbuffers/) cross platform serialization library.  


## Installation

- Please, follow official recommendation for ns-3 installation.
- SIVERT API has a dependency on ZeroMQ. [CPPZMQ 4.2.2](https://github.com/zeromq/cppzmq/releases/tag/v4.2.2) implementation was used on NS-3 part. Please, build and install it from sources following the instructions and make sure that either header correct path is added into your $PATH or the full paths to headers (`zmq.hpp` and `zhelpers.hpp`) are modified in the corresponding NS-3 SIVERT example scripts.

**NB** Current version of SIVERT is implemented for MacOS, Linux version should become available later.


## References

<a id="1">[1]</a>
Esteban Egea-Lopez, Fernando Losilla, Juan Pascual-Garcia and Jose Maria Molina-Garcia-Pardo, "Vehicular Network Simulation with Realistic Physics", IEEE Access, 2019, DOI:10.1109/ACCESS.2019.2908651

<a id="2">[2]</a>
F. Eckermann, M. Kahlert, C. Wietfeld, "Performance Analysis of C-V2X Mode 4 Communication Introducing an Open-Source C-V2X Simulator", In 2019 IEEE 90th Vehicular Technology Conference (VTC-Fall), Honolulu, Hawaii, USA, September 2019.
