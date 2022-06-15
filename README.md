# Mount Doom
By Armin Wells, Ben Bals, Davis Issac, Jonas Schmidt, Katrin Casel, Leo Wendt, Niko Hastrich, Otto Kißig, Robin Wersich, Sebastian Angrick, and Theresa Hradilak.

Mount Doom is an exact solver for directed feedback vertex set.

We approach this problem by first exhaustively applying a set of reduction rules to obtain a potentially smaller kernel. In order to find a minimum DFVS on this remaining kernel, we solve a series of derived Vertex Cover instances.  We provide a theoretical [solver description](https://github.com/BenBals/mount-doom-description/raw/main/exact.pdf) and its [source code](https://github.com/BenBals/mount-doom-description).

It was created as part of Hasso Plattner Institute's participation of the [2022 PACE Challenge](https://pacechallenge.org/2022/).

[Hasso Plattner Institute](https://hpi.de) / Uni Potsdam
[mount-doom@lists.myhpi.de](mailto:mount-doom@lists.myhpi.de)

This work is available to you under GPL v3, see the LICENSE file.
  
## Build environment setup using nix

You can use nix to get your toolchain (compiler + cmake).

1. [Install nix](https://nixos.org/download.html).
2. [Setup nix flakes](https://nixos.wiki/wiki/Flakes).
3. Simply run `nix develop` to get a shell you can work in. Alternatively, use `nix develop .#static` to get an environment in which to build static binaries.

## Manual installation
You need a modern `gcc` and `cmake` and install the boost development libaries for your system (e.g. `boost-dev` on Apline).

## Configure the build
```shell
cmake -S . -B build-release-static -DCMAKE_BUILD_TYPE=Release -DDOOM_STATIC=ON
```

- Remove `-DDOOM_STATIC=ON` to build a dynamic executable
- All dependencies (except doom) are dynamically fetched as sources by CMake.

## Compile the project
Run  `cmake --build build -j4 --target doom`

## Run the solver
Pipe a graph in the [PACE input format](https://pacechallenge.org/2022/tracks/#input-format) into the process, e.g. `cat my_instance | doom`.
