# FastSLAM

![unit test badge](https://github.com/USC-ACTLab/FastSLAM/actions/workflows/run-unit-tests.yml/badge.svg)

Implementation of FastSLAM 1.0 as presented by Montemerlo et al. (2002)

## Building the Library

To build the library, run the following commands from the root directory:
```bash
cmake -B build -S .
cmake --build build -j2
```

To build the tests, instead run:
```bash
cmake -B build -S . -DBUILD_TESTS=ON
cmake --build build -j2
cd build && ctest
```