# Orodruin
By Armin Wells, Ben Bals, Davis Issac, Jonas Schmidt, Katrin Casel, Leo Wendt, Niko Hastrich, Otto Kißig, Robin Wersich, Sebastian Angrick, and Theresa Hradilak.

Orodruin is a heuristic solver for directed feedback vertex set. It was created as part of Hasso Plattner Institute's participation of the [2022 PACE Challenge](https://pacechallenge.org/2022/).

[Hasso Plattner Institute](https://hpi.de) / Uni Potsdam
[mount-doom@lists.myhpi.de](mailto:mount-doom@lists.myhpi.de)

This work is available to you under GPL v3, see the LICENSE file.
  
## Build environment setup using nix

You can use nix to get your toolchain (compiler + cmake).

1. [Install nix](https://nixos.org/download.html).
2. [Setup nix flakes](https://nixos.wiki/wiki/Flakes).
3. Simply run `nix develop` to get a shell you can work in.

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

## Run the tests
From inside the `build` directory, run ~ctest~.

## CLion

CLion's environment management is a hot steaming mess. We don't know all the answers yet. Here are some things that
worked for us:

- Start CLion from inside the nix shell
- Don't used the bundled CMake but create new 'toolchain' where you just put `cmake` as the path to CMake.
- Don't use Ninja. In the CMake settings, use the generator `Unix Makefiles` or set the CMake
  options `-G "Unix Makefiles"`.
- We use clang-format as formatter. CLion can automatically format according to the style guide. For this click on "4
  spaces" in the lower right corner and then click "Enable ClangFormat"

## MacOS

- Install boost manually (e.g. `brew install boost`).

# Solver Description
Can be found the `main` branch.

# Formatting
- You can use the `scripts/format.sh` script to format everything in the project.
