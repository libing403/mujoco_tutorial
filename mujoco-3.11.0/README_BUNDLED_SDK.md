# Bundled MuJoCo SDK

This directory contains the official MuJoCo 3.11.0 binary distribution for
Linux x86-64.  It is vendored so the tutorial examples build without fetching
or compiling MuJoCo.

- License: `LICENSE`
- Third-party notices: `THIRD_PARTY_NOTICES`
- Public headers: `include/`
- Runtime library: `lib/libmujoco.so.3.11.0`
- Official utilities and samples: `bin/`, `sample/`

The tutorial CMake projects use only `include/` and `lib/`, at this fixed
repository-relative location. This SDK does not make the binaries compatible
with other operating systems or CPU architectures. Users on another platform
must replace this directory with the matching official 3.11.0 SDK while
preserving the same `include/` and `lib/` layout.
