#!/bin/bash

base_dir=$(cd "$(dirname "$0")";pwd)
mode="release"
if [ $1 ]; then mode=$1; fi

echo "--------|||  BUILD JS  |||--------"

echo "|||  GENERATE |||"
cd physx/
./generate_projects.bat emscripten-js

echo "|||  COMPILE |||"
cd compiler/emscripten-js-$mode
ninja

echo "--------|||  BUILD WASM  |||--------"

cd $base_dir

echo "|||  GENERATE |||"
cd physx/
./generate_projects.bat emscripten-wasm

echo "|||  COMPILE |||"
cd compiler/emscripten-wasm-$mode
ninja

echo "|||  COPY  |||"

cd $base_dir
mkdir -p $base_dir/builds
cp $base_dir/physx/bin/emscripten/$mode/physx.$mode.asm.js $base_dir/builds/physx.$mode.asm.js
cp $base_dir/physx/bin/emscripten/$mode/physx.$mode.wasm.js $base_dir/builds/physx.$mode.wasm.js
cp $base_dir/physx/bin/emscripten/$mode/physx.$mode.wasm.wasm $base_dir/builds/physx.$mode.wasm.wasm

echo "|||  FINISH  |||"

read