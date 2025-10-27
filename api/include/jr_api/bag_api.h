#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bag_writer bag_writer_t;

bag_writer_t* bag_writer_create(const char* path);

void bag_writer_destroy(bag_writer_t* writer);

void bag_writer_record_topic(bag_writer_t* writer, const char* topic);

void bag_writer_record_all(bag_writer_t* writer);

#ifdef __cplusplus
}
#endif
