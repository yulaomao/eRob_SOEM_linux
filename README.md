***This is an open-source demo related to the eRob product, provided solely for reference by developers. Please note that issues within the open-source project are independent of the quality of eRob products. Users are advised to exercise caution while using the demo. We are not responsible for any damage caused by improper operations. For any project errors, please raise a query in the Issues section. Collaboration and forks to resolve open-source project issues are welcome.***

## Recommendations for EtherCAT Open-Source Master Users  

1. **Use a Real-Time Kernel System**  
   Ensure your operating system has a real-time kernel to guarantee consistent and precise communication.

2. **Isolate CPU Cores**  
   Perform CPU isolation to dedicate specific cores to EtherCAT processes, reducing interruptions and improving stability.

3. **Troubleshooting OP State Issues**  
   - Failure to enter OP state may be caused by errors in the **object dictionary mapping** or improper configuration of **DC (Distributed Clock) mode**.  
   - **eRob** only supports **DC mode**, and proper configuration of DC mode is crucial for system synchronization and precision.

4. **Read Mode-Specific Instructions**  
   Before using each mode, read the relevant operational instructions to ensure correct configuration and usage.

5. **Use the Official eRob Upper Computer Software**  
   eRob provides official upper computer software. Mastering the built-in **oscilloscope tool** will allow you to quickly locate issues with the EtherCAT master.

6. **Capture and Analyze EtherCAT Data**  
   Use packet capture tools to analyze EtherCAT output and log information to identify errors.


# Simple Open EtherCAT Master Library
[![Build Status](https://github.com/OpenEtherCATsociety/SOEM/workflows/build/badge.svg?branch=master)](https://github.com/OpenEtherCATsociety/SOEM/actions?workflow=build)

BUILDING
========


Prerequisites for all platforms
-------------------------------

 * CMake 3.9 or later


Windows (Visual Studio)
-----------------------

 * Start a Visual Studio command prompt then:
   * `mkdir build`
   * `cd build`
   * `cmake .. -G "NMake Makefiles"`
   * `nmake`

Linux & macOS
--------------

   * `mkdir build`
   * `cd build`
   * `cmake ..`
   * `make`

ERIKA Enterprise RTOS
---------------------

 * Refer to http://www.erika-enterprise.com/wiki/index.php?title=EtherCAT_Master

Documentation
-------------

See https://openethercatsociety.github.io/doc/soem/


Want to contribute to SOEM or SOES?
-----------------------------------

If you want to contribute to SOEM or SOES you will need to sign a Contributor
License Agreement and send it to us either by e-mail or by physical mail. More
information is available in the [PDF](http://openethercatsociety.github.io/cla/cla_soem_soes.pdf).
# SOEM_linux

# eRob_test result:

The results were obtained under an RTLinux environment, where `6 eRobs` were driven simultaneously and ran stably for `3 hours`. The image on the right shows the scheduling latency. Although scheduling latency became abnormal at the end of the program, the average scheduling latency remained very low, with normal latency around 100 microseconds during operation.

<div class="Result">
  <a>
    <img src="https://cdn-fusion.imgcdn.store/i/2024/293414c32d874812.png" alt="Result" style={{ width: '1000', height: 'auto' }} />
  </a>
</div>

