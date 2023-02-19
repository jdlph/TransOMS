# openDTA

Rebuild [DTALite](https://github.com/asu-trans-ai-lab/DTALite) according to the deeply optimized [Path4GMNS](https://github.com/jdlph/Path4GMNS) using Modern C++. Our goal is to build an enterprise-grade and lightning-fast dynamic traffic assignment (DTA) system that works across platforms.

Extra efforts will be in place to the following parts to improve its performance.

1. Faster shortest path algorithm implementation.
   1. The most efficient deque implementation of the modified label correcting (MLC) algorithm.
   2. Try heap Dijkstra as well.
   3. Only maintain the link predecessor rather than both link and node predecessors. Node predecessor can be easily inferred from link predecessor.
   4. Fast identify a column / path is existing or not via a customer hash key. Store columns between each OD pair in a hash table come up a hash function for column.
2. Memory.
   1. Avoid unnecessary copying and reduce potential memory fragmentation.
   2. Space-efficient column storing.
3. Faster I/O.
   1. MIOCSV
   2. Avoid unnecessary buffer flushing (i.e., use '\n' rather than std::endl)
4. Use yaml-based setting file.
5. Adopt json as additional format for data exchanging.
6. Others. See [Refactoring](https://github.com/jdlph/DTALite#refactoring) for details.