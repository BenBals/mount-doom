cmake -S . -B build-release-static -DCMAKE_BUILD_TYPE=Release -DDOOM_STATIC=ON
cmake --build build-release-static -j4 --target doom
