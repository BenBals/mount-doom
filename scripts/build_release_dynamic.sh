cmake -S . -B build-release-dynamic -DCMAKE_BUILD_TYPE=Release -DDOOM_STATIC=OFF
cmake --build build-release-dynamic -j4 --target doom
