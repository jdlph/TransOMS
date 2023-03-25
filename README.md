# TransOMS

TransOMS is a suite of **state-of-the-art** open-source applications for **Trans**portation **O**ptimization, **M**odeling, and **S**imulation, which brings the **cutting-edge** researches and the **enterprise-grade** implementations all together into one place.

One of its major components is a *modern*, *cross-platform*, and *lightning-fast* **dynamic traffic assignment (DTA)** system. It features a complete redesign of [DTALite](https://github.com/asu-trans-ai-lab/DTALite) according to the deeply optimized [Path4GMNS](https://github.com/jdlph/Path4GMNS) using Modern C++ and enhancement to its core algorithms. Additional features are underway, including origin-destination demand estimation (ODME), analytical DTA based on fluid queue, and more.

## Design

Extensive efforts have been or will be in place to ensure its appearance and performance.
### Appearance
#### High-Quality and Cross-Platform
1. Follow the best coding practices including [C++ Core Guidelines](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines).
2. Prepare the source code in modern C++ of the new standards (up to [C++17](https://en.cppreference.com/w/cpp/17)) and avoid vendor-specific features.
3. Use CMake to define the build process across platforms.

#### Modern Configuration and Data Interchange
1. Ditch the legacy [settings.csv](https://github.com/jdlph/Path4GMNS/blob/master/tests/settings.csv) and use Yaml for [configuration](https://github.com/jdlph/TransOMS/blob/dev/data/Chicago_Sketch/settings.yml).
2. Introduce [JSON](https://json.org/example.html) as additional format to CSV for data interchange.

### Performance
#### Faster Path Engine
1. Adopt *[the most efficient deque implementation of the modified label correcting (MLC) algorithm](https://github.com/jdlph/Path4GMNS/blob/master/engine/path_engine.cpp)* from Path4GMNS.
2. Implement [heap-Dijkstra's algorithm](https://github.com/jdlph/shortest-path-algorithms/blob/release/src/spalgm.py) as an alternative path engine (for speed comparison).
3. Maintain the link predecessor only rather than both link and node predecessors as node predecessor can be easily inferred from link predecessor.
4. Store columns between each OD pair in a hash table along with a customer hash function for column. This enables a constant-time check on whether a newly identified column / path is existing or not. Its background information can be found [here](https://github.com/jdlph/Path4GMNS/tree/dev#more-on-the-column-generation-module).

#### Better Memory Management
1. Avoid unnecessary copying through the move semantics.
2. Fully utilize stack memory. Directly store / move some objects into a container (with the move semantics) and retrieve them later without restoring heap memory (as pointers). This can improve the performance, reduce potential memory fragmentation, and prevent possible risk of memory leak.
3. Space-efficient column representation to avoid duplications among similar columns. Details will be provided later.
4. Only keep link path in memory and derive node path whenever is needed.

#### Flying and Portable I/O
1. Adopt the memory-mapping-based [MIOCSV](https://github.com/jdlph/MIOCSV) to handle CSV parsing and writing, which works seamless on different OS's and platforms.
2. Eliminate unnecessary buffer flushing (i.e., use '\n' rather than std::endl).
3. Use multithreading to boost the I/O-bounded processes.

#### Others
The **ongoing development** can be tracked through [Projects](https://github.com/users/jdlph/projects/2). Other potential enhancements are documented in [Refactoring](https://github.com/jdlph/DTALite#refactoring).

## Build

TransOMS is built on [C++17](https://en.cppreference.com/w/cpp/17) with [yaml-cpp](https://github.com/jbeder/yaml-cpp) parsing configuration file[^1] and [OpenMP](https://www.openmp.org/about/openmp-faq/#WhatIs) managing parallelization.

yaml-cpp is precompiled as a static library and embedded in [lib](lib/). OpenMP requires an additional runtime library. Its detailed installation instruction is summarized [here](https://path4gmns.readthedocs.io/en/latest/usecases.html#target-to-paragraph).

The build process is defined in [CMakeLists.txt](CMakeLists.txt) along with the above dependency specifications. You will need [CMake](https://cmake.org/download/) (3.1.0 or higher) to build the executable by running the following commands.

```bash
# from the root directory of TransOMS
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ cmake --build .
```

Many modern IDEs or text editors (e.g., [VS Code](https://code.visualstudio.com/)) support CMake as an alterative build system, which offers an integrated experience of building, debugging, and testing a CMake project (without command lines). If you want to take this advantage, a step-by-step tutorial on VS Code is listed [here](https://code.visualstudio.com/docs/cpp/cmake-linux#_create-a-cmake-hello-world-project). The comprehensive documentation can be retrieved on [GitHub](https://github.com/microsoft/vscode-cmake-tools/blob/main/docs/README.md).
## Benchmark

coming soon!

## Reference

[^1]: TransOMS now supports Windows (x86), Linux (x86), macOS (x86), and Apple Silicon. Make sure you use **Clang** as the complier for **macOS**. Using g++ will lead to **compatibility issue** with yaml-cpp, which is built using Clang.