#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void init();

void start_recording(const char* save_directory);

void stop_recording();

void shutdown();

#ifdef __cplusplus
}
#endif
