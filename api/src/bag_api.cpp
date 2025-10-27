#include "jr_api/bag_api.h"

#include <bag/bag_writer.hpp>

struct bag_writer {
  jr::mw::BagWriter writer;
};

bag_writer_t* bag_writer_create(const char* path) {
  return new bag_writer_t{jr::mw::BagWriter{path}};
}

void bag_writer_destroy(const bag_writer_t* writer) {
  if (writer) {
    delete writer;
  }
}

void bag_writer_record_topic(bag_writer_t* writer, const char* topic) {
  if (writer) {
    writer->writer.record_topic(topic);
  }
}

void bag_writer_record_all(bag_writer_t* writer) {
  if (writer) {
    writer->writer.record_all();
  }
}
