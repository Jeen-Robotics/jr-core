# Jeen Robotics Core

## Build for Android

```bash
cmake -B build-android -DCMAKE_TOOLCHAIN_FILE=$HOME/Library/Android/sdk/ndk/27.0.11902837/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NATIVE_API_LEVEL=29 -DBUILD_SHARED_LIBS=ON -DBUILD_SANDBOX=OFF
cmake --build build-android

# Copy to jr-front
find build-android -name "*.so" -exec cp {} $jr_front_root/android/app/src/main/jniLibs/arm64-v8a \;
```
