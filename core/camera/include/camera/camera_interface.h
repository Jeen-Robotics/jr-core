#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void initializeCamera();
bool isCameraInitialized();
void stopCamera();
uint8_t *processFrame(int width, int height);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_INTERFACE_H