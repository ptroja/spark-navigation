spark-navigation
================

This repository contains implementations of robot navigation
algorithms in Ada/SPARK:

* Vector Field Histogram Plus (VFH+)
* Nearness Diagram Navigation (ND)
* Smooth Nearness Diagram Navigation (SND)

The code is already integrated with the Player/Stage robot programming
environment and the integration with ROS is planned.

For details of the algorithms, configuration parameters and
references, see the list of Player [drivers][drivers].

Implementations of VFH+ and ND are based on the latest code from the
Player repository. Implementation of SND is based on the code from the
author's [website][SND], as the code is Player repository seems
outdated.

For each algorithm there are `cpp` and `ada` folders with the original
code in C/C++, but including our fixes, and the corresponding code in
Ada/SPARK, respectively.

Only the Ada/SPARK implementation of SND has been fully verified for
run-time safety (with the assumption that floating-point operations
are exact and do not overflow). The other implementations suffer from
poor programming style, which prevented us from proving its run-time
safety, and/or code features, such as *type invariants*, that are not
yet supported by SPARK.

Compilation
===========

C/C++
-----

The C++ code can be compiled using CMake and the standard procedure
for Player driver plugins. The path to the Player installation needs
to be set in the individual `CMakeLists.txt` files.

Ada/SPARK
---------

The code is split into "driver" parts in full Ada, which integrate the
algorithm with the Player, and "algorithm" parts in SPARK, which
implement the navigation procedures.

To build Player plugins use `gprbuild -P driver` in the `ada` folders
of the individual algorithms. The path to the Player installation
needs to be set in the `driver.gpr` files.

Verification
============

T.B.D.

[drivers]: http://playerstage.sourceforge.net/doc/Player-svn/player/group__drivers.html "drivers"

[SND]: http://motion.me.ucsb.edu/~joey/website/media.html "SND code"
