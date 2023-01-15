#pragma once

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message_source {
  uint8_t station_id;
  bool io_1;
  bool io_2;
  bool io_3;

}struct_message_source;


typedef struct struct_message_target {
  // uint8_t station_id;
  bool io_21;
  bool io_22;
  bool io_23;
  bool io_41;
  bool io_42;
  bool io_43;

}struct_message_target ;
