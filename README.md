# Caladan Policies

This repository implements two core-allocation policies (delay range and utilization range) on top of [Caladan](https://github.com/shenango/caladan).

## How to Run

Follow the [Caladan](https://github.com/shenango/caladan) instructions for cloning the repository, installing dependencies, and building. You can also use the scripts in the [caladan-all](https://github.com/shenango/caladan-all) repository to run experiments.

### Range-Based Core-Allocation Policies

To enable the range-based core-allocation policies, run the iokernel with the `range_policy` parameter and an `interval` of 5 us. For example:
```
sudo ./iokerneld simple range_policy interval 5
```

You can control which core-allocation policy is used for each application by setting parameters in their config files. For example, to run with the delay range policy with a range of 1-4 us, add to your config:
```
runtime_qdelay_lower_thresh_ns 1000
runtime_qdelay_upper_thresh_ns 4000
```
To run with the utilization range policy with a range of 75-95%, add to your config:
```
runtime_util_lower_thresh 0.75
runtime_util_upper_thresh 0.95
```

### Caladan's Core-Allocation Policy

To run with Caladan's simple core-allocation policy, omit the parameters above from your application config files, and run the iokernel with the simple core-allocation policy (`sudo ./iokerneld simple`).

## Supported Platforms

This code has been tested most thoroughly on Ubuntu 20.04 with kernel 5.4.0.

### NICs
This code has been tested with a 40 Gbits/s Mellanox Connect X-5 Bluefield NIC on the server side (we did not use the SmartNIC features) and a 100 Gbits/s Intel E810C NIC on the client side. We run Caladan in "queue steering mode" in which we reconfigure the mappings between NIC queues and cores when core allocations change because our NICs do not support Caladan's default "flow steering mode" (see [here](https://dspace.mit.edu/handle/1721.1/124072) for details about the modes). We enable queue steering mode by adding `enable_directpath qs` to the client and server config files. Users who have regular Mellanox Connect X-5 NICs may prefer to omit this and use flow steering mode instead.