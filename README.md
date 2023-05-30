hardware-traffic-classifier
===========================

FPGA-based traffic classifier using the SVM algorithm. This version is made for NetFPGA 1G. Adaptation files for the Invea-Tech Combo 2x10 Gb/s board may also be made available on request.

Main modules:
 * `svm/svm_detection.v`: Generic FPGA SVM implementation, made to work in parallel with two possible kernels: RBF or CORDIC, depending on the configuration files;
 * `flows/flow_storage.v`: Mechanism to store flows in a fast and efficient way, with a low loss probability;
 * `flows/traffic_classifier.v`: Traffic classifier using the flow storage to gather data about flows and the SVM detection to decide the class of a flow.

The SVM classifier is configured in these sources with a specific SVM model used as example. The SVM model can be changed by modifying files `svm/config/svm_parameters*`. The best way to do that is to create a small script that will transform an SVM model into configuration files.

Sources are commented to make reuse easier. However, adaptation to a new implementation can be complex. Do not hesitate to ask if you need clarifications.

## Small FIFO

The code uses a `small_fifo` module. It's a very standard FIFO. Its model was taken from the NetFPGA project: [`small_fifo_v3.v`](https://github.com/NetFPGA/netfpga/blob/master/lib/verilog/core/utils/src/small_fifo_v3.v). Add this file to your project to be able to simulate or synthetise it.
