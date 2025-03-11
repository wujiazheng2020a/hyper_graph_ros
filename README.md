# 1. What is HyperGraph ROS?
It is an open-source robot operating system that unifies intra-process, inter-process, and cross-device computation into a computational hypergraph for efficient message passing and parallel execution. In order to optimize communication, HyperGraph ROS dynamically selects the optimal communication mechanism while maintaining a consistent API. For intra-process messages, Intel-TBB Flow Graph is used with C++ pointer passing, which ensures zero memory copying and instant delivery. Meanwhile, inter-process and cross-device communication seamlessly switch to ZeroMQ. When a node receives a message from any source, it is immediately activated and scheduled for parallel execution by Intel-TBB. The computational hypergraph consists of nodes represented by TBB flow graph nodes and edges formed by TBB pointer-based connections for intra-process communication, as well as ZeroMQ links for inter-process and cross-device communication. This structure enables seamless distributed parallelism. Additionally, HyperGraph ROS provides ROS-like utilities such as a parameter server, a coordinate transformation tree, and visualization tools.

In the program, for the sake of convenience, it is simplified as **tros**. Where **t** signifies "Topology", Referring to the computational hypergraph structure that unifies intra-process, inter-process, and distributed execution as a single coherent model.

# 2. For citation
```bash
@misc{zhang2025hypergraph,
  title={HyperGraph ROS: An Open-Source Robot Operating System for Hybrid Parallel Computing based on Computational HyperGraph},
  author={Shufang Zhang and Jiazheng Wu and Jiacheng He and Kaiyi Wang and Shan An},
  year={2025},
  eprint={2503.05117},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2503.05117}
}
```

# 3. Installation Steps:

## 3.1 Installation of other libraries
```bash
sudo apt-get update
sudo apt-get install -y libgoogle-glog-dev libgtest-dev libyaml-cpp-dev libeigen3-dev doxygen protobuf-compiler libprotobuf-dev libzmq3-dev
```

## 3.2 Installation of TBB
```bash
git clone git@github.com:oneapi-src/oneTBB.git
```
### 3.2.1 Navigate to the project folder
```bash
cd oneTBB
```
### 3.2.2 Build
```bash
mkdir build
cd build
cmake ..
make -j 8
```
Choose the number of threads after 'j' based on the environment, ensuring memory isn't exceeded.
### 3.2.3 Install
```bash
sudo make install
```

## 3.3 HyperGraph ROS Installation
```bash
git clone https://github.com/wujiazheng2020a/hyper_graph_ros.git
```
### 3.3.1 Navigate to the project folder
```bash
cd tros
```
### 3.3.2 Build
```bash
mkdir build
cd build
cmake ..
make -j 8
```
Choose the number of threads after 'j' based on the environment, ensuring memory isn't exceeded.
### 3.3.3 Install
```bash
sudo make instal
```

# 4. Usage Guide
See web/index.htm or https://wujiazheng2020a.github.io/hyper_graph_ros/
