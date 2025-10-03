# Jeen Robotics Core

## Build (Host)

```bash
scripts/install_deps.sh

cmake . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake
cmake --build build
```

## Build (Android)

Conan profile (save as ~/.conan2/profiles/android):
```
include(default)

[settings]
os=Android
os.api_level=29
arch=armv8
compiler=clang
compiler.version=18
compiler.libcxx=c++_shared
compiler.cppstd=17

[conf]
tools.android:ndk_path=/usr/local/share/android-ndk
# or it might be: $HOME/Library/Android/sdk/ndk/27.0.11902837
```

```bash
scripts/install_android_deps.sh

cmake . -B build-android -G Ninja -DCMAKE_TOOLCHAIN_FILE=build-android/conan_toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NATIVE_API_LEVEL=29 -DBUILD_SHARED_LIBS=ON -DBUILD_SANDBOX=OFF
cmake --build build-android

# Copy to jr-front
find build-android -name "*.so" -exec cp {} $jr_front_root/android/app/src/main/jniLibs/arm64-v8a \;
```
