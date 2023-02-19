# openDTA

Rebuild [DTALite](https://github.com/asu-trans-ai-lab/DTALite) according to the deeply optimized [Path4GMNS](https://github.com/jdlph/Path4GMNS) using Modern C++. Our goal is to build a **state-of-the-art** dynamic traffic assignment (DTA) system that is **enterprise-grade** and **lightning-fast** across platforms.

Extensive efforts have been or will be in place to ensure its appearance and performance.

## Appearance
### High-Quality and Cross-Platform
1. Follow the [C++ Core Guidelines](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) and some other best coding practices.
2. Write the source code in modern C++ following the new standards (up to C++20) and avoiding vendor-specific features.
3. Use CMake to define the building process.

### Modern Configuration and Data Interchange
1. Use Yaml for [configuration files](https://github.com/jdlph/Path4GMNS/blob/master/tests/settings.yml), while ditch [CSV](https://github.com/jdlph/Path4GMNS/blob/master/tests/settings.csv).
2. Adopt JSON as additional format to CSV for data interchange.

## Performance
### Faster Path Engine
1. Adopt [the most efficient deque implementation of the modified label correcting (MLC) algorithm](https://github.com/jdlph/Path4GMNS/blob/master/engine/path_engine.cpp) from Path4GMNS.
2. Implement [heap-Dijkstra's algorithm](https://github.com/jdlph/shortest-path-algorithms/blob/release/src/spalgm.py) as an alternative.
3. Maintain the link predecessor only rather than both link and node predecessors. Node predecessor can be easily inferred from link predecessor.
4. Store columns between each OD pair in a hash table along with a customer hash function for column. This enables almost constant-time check whether a newly identified column / path is existing or not.

### Better Memory Management
1. Avoid unnecessary copying through move semantics.
2. With move semantics, it is also possible to store some objects and retrieve them directly and without restoring heap memory (as pointers), which can reduce potential memory fragmentation.
3. Space-efficient column storing. Details will be provided later.

### Flying and Portable I/O
1. Adopt the memory-mapping-based [MIOCSV](https://github.com/jdlph/MIOCSV) to handle CSV parsing and writing, which works seamless on different OS's and platforms.
2. Avoid unnecessary buffer flushing (i.e., use '\n' rather than std::endl).

### Others
See [Refactoring](https://github.com/jdlph/DTALite#refactoring) for details.