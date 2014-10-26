// Arduino MAVLink test code.

#define MAVLINK10

#include <FastSerial.h>
#include <GCS_MAVLink.h>
#include <Time.h>

//Plane 0's info
//#define PLANE_ID 0
//#define OTHER_PLANE_ID 1
//#define SOURCE_ID 101
//
//#define PLANE_ALT 200
//
//#define START_LAT 353295420
//#define START_LON -1207548140
//#define START_ALT PLANE_ALT
//#define END_LAT  353264570
//#define END_LON -1207528840
//#define END_ALT PLANE_ALT

//Plane 1's info
#define PLANE_ID 1
#define OTHER_PLANE_ID 0
#define SOURCE_ID 102

#define PLANE_ALT 250

#define START_LAT 353264570
#define START_LON -1207528840
#define START_ALT PLANE_ALT
#define END_LAT 353295420
#define END_LON -1207548140
#define END_ALT PLANE_ALT

#define SERIAL_BAUD 57600
#define XBEE_BAUD 9600
#define NUM_PLANES 2

#define STATE_MANUAL 1
#define STATE_GO_TO_START 2
#define STATE_GO_TO_FINISH 3
#define STATE_COLLISION_DETECTED 4

#define CUSTOM_MODE_MANUAL 0
#define CUSTOM_MODE_STABILIZE 2
#define CUSTOM_MODE_AUTO 10

FastSerialPort0(Printer);
FastSerialPort1(Serial);
FastSerialPort3(xbee);

#define SIGFIG_CONVERT 10000000.0

typedef struct _gps_t {
  unsigned long time_now;
  int32_t lat;
  int32_t lon;
  int32_t alt;
  int32_t heading;
  int16_t vx;
  int16_t vy;
  int16_t vz;
} gps_t;

typedef struct _waypoint_t {
  gps_t waypoint;
} waypoint_t;

typedef struct _plane_t {
  int curr_state;
  int next_state;
  gps_t curr_position;
  waypoint_t curr_waypoint;
  waypoint_t start_waypoint;
  waypoint_t end_waypoint;
} plane_t;

typedef struct _air_space_t {
  plane_t planes[NUM_PLANES];
} air_space_t;

air_space_t curr_airspace;

typedef struct _header_t {
  uint8_t source_id;
  uint8_t dest_id;
  uint8_t seq;
  uint8_t ttl;
  uint16_t message_type;
  uint16_t message_length;
} header_t __attribute__((packed));

typedef struct _vehicle_system_status_t {
  int64_t time_stamp;
  uint16_t vehicle_id;
  uint8_t vehicle_mode;
  uint8_t vehicle_state;
} vehicle_system_status_t __attribute__((packed));

typedef struct _vehicle_global_position_t {
  uint64_t time_stamp;
  uint16_t vehicle_id;
  int32_t lat;
  int32_t lon;
  int32_t alt;
  int32_t heading;
  int16_t x_speed;
  int16_t y_speed;
  int16_t z_speed;
} vehicle_global_position_t __attribute__((packed));

#define VEHICLE_SYSTEM_PACKET_SIZE 12
#define VEHICLE_GLOBAL_PACKET_SIZE 32

typedef struct _packet_system_status_t {
  int32_t sync;
  header_t header;
  uint8_t data[VEHICLE_SYSTEM_PACKET_SIZE];
  int8_t checksum;
} packet_system_status_t __attribute__((packed));

typedef struct _packet_global_position_t {
  int32_t sync;
  header_t header;
  uint8_t data[VEHICLE_GLOBAL_PACKET_SIZE];
  int8_t checksum;
} packet_global_position_t __attribute__((packed));

mavlink_message_t msg;
mavlink_status_t status;

void setup()
{ 
  Serial.begin(SERIAL_BAUD);
  Printer.begin(XBEE_BAUD);
  xbee.begin(XBEE_BAUD);
  
  curr_airspace.planes[PLANE_ID].start_waypoint.waypoint.lat = START_LAT;
  curr_airspace.planes[PLANE_ID].start_waypoint.waypoint.lon = START_LON;
  curr_airspace.planes[PLANE_ID].start_waypoint.waypoint.alt = START_ALT;
  
  curr_airspace.planes[PLANE_ID].end_waypoint.waypoint.lat = END_LAT;
  curr_airspace.planes[PLANE_ID].end_waypoint.waypoint.lon = END_LON;
  curr_airspace.planes[PLANE_ID].end_waypoint.waypoint.alt = END_ALT;
  
  curr_airspace.planes[PLANE_ID].curr_state = STATE_MANUAL;
  curr_airspace.planes[PLANE_ID].next_state = STATE_MANUAL;
  
  clear_waypoints();
    
  Printer.println("FINISHED SETUP");
}

#define FIRST_COUNTER_INCREMENT 100
#define SECOND_COUNTER_INCREMENT 200

unsigned long first_counter = FIRST_COUNTER_INCREMENT;
unsigned long second_counter = SECOND_COUNTER_INCREMENT;

void loop() {
  
  unsigned long time_now = millis();
  
  if(time_now >= first_counter) {
    read_from_apm();
    
    sense_and_avoid();
    
    first_counter += FIRST_COUNTER_INCREMENT;
  }
  
  if(time_now >= second_counter) {  
    send_curr_plane_info();
    second_counter += SECOND_COUNTER_INCREMENT;
  }
  
  receive_info();
}

/* APM */

#define SYSTEM_ID MAV_TYPE_GCS
#define COMPONENT_ID MAV_COMP_ID_MISSIONPLANNER
#define TYPE MAV_TYPE_GCS
#define SYSTEM_TYPE MAV_TYPE_FIXED_WING
#define MAV_AUTOPILOT MAV_AUTOPILOT_INVALID
#define SYSTEM_MODE MAV_MODE_PREFLIGHT
#define CUSTOM_MODE 0
#define SYSTEM_STATE MAV_STATE_STANDBY

void read_from_apm() {
  send_heartbeat();
  send_request_for_data_stream_position();
  poll_for_response();
}

void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, TYPE, MAV_AUTOPILOT, SYSTEM_MODE, CUSTOM_MODE, SYSTEM_STATE);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
  //Printer.println("HEARTBEAT SENT");
}

void send_request_for_data_stream_position() {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMPONENT_ID, &msg, SYSTEM_TYPE, 0, MAV_DATA_STREAM_POSITION, 100, 1);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
  //Printer.println("REQUEST DATA POSITION SENT");
}

void poll_for_response() {
  
  while(Serial.available()) {
    uint8_t c = Serial.read();
    
    if(mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
      
      //Printer.print("MESSAGEID: ");
      //Printer.println(msg.msgid);
      
      if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t *heartbeat = (mavlink_heartbeat_t *)msg.payload64;
        
        if(curr_airspace.planes[PLANE_ID].curr_state == STATE_MANUAL) {
          if(heartbeat->custom_mode == CUSTOM_MODE_AUTO) {
            curr_airspace.planes[PLANE_ID].next_state = STATE_GO_TO_START;
          }
        } else if(curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_START) {
          if(heartbeat->custom_mode == CUSTOM_MODE_MANUAL) {
            curr_airspace.planes[PLANE_ID].next_state = STATE_MANUAL;
          } else if (heartbeat->custom_mode == CUSTOM_MODE_STABILIZE) {
            curr_airspace.planes[PLANE_ID].next_state = STATE_GO_TO_FINISH;
          }
        } else if(curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_FINISH) {
          if(heartbeat->custom_mode == CUSTOM_MODE_MANUAL) {
            curr_airspace.planes[PLANE_ID].next_state = STATE_MANUAL;
          } else if (heartbeat->custom_mode == CUSTOM_MODE_AUTO) {
            //curr_airspace.planes[PLANE_ID].next_state = STATE_GO_TO_START;
          }
        } else if(curr_airspace.planes[PLANE_ID].curr_state == STATE_COLLISION_DETECTED) {
          if(heartbeat->custom_mode == CUSTOM_MODE_MANUAL) {
            curr_airspace.planes[PLANE_ID].next_state = STATE_MANUAL;
          } 
        }
        
//        Printer.println("HEARTBEAT");
//        Printer.print("  CUSTOM MODE: ");
//        Printer.println(heartbeat->custom_mode);
//        Printer.print("  TYPE: ");
//        Printer.println(heartbeat->type);
//        Printer.print("  AUTO: ");
//        Printer.println(heartbeat->autopilot);
//        Printer.print("  BASE MODE: ");
//        Printer.println(heartbeat->base_mode);
//        Printer.print("  SYSTEM STATUS: ");
//        Printer.println(heartbeat->system_status);
//        Printer.print("  VERSION: ");
//        Printer.println(heartbeat->mavlink_version);
//        Printer.println();
      } else if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t *gps = (mavlink_global_position_int_t *)msg.payload64;
        
        curr_airspace.planes[PLANE_ID].curr_position.time_now = now();
        curr_airspace.planes[PLANE_ID].curr_position.lat = gps->lat;
        curr_airspace.planes[PLANE_ID].curr_position.lon = gps->lon;
        curr_airspace.planes[PLANE_ID].curr_position.alt = gps->alt / 1000;
        curr_airspace.planes[PLANE_ID].curr_position.vx = gps->vx;
        curr_airspace.planes[PLANE_ID].curr_position.vy = gps->vy;
        curr_airspace.planes[PLANE_ID].curr_position.vz = gps->vz;
        curr_airspace.planes[PLANE_ID].curr_position.heading = gps->hdg;
      }
    }
  }
}

/* XBEE */

void send_curr_plane_info() {
  send_system_status();
  send_global_position();
}

#define SYNC 1129336832
#define DEST_ID 0
#define SEQUENCE 0
#define TTL 0
#define MESSAGE_TYPE_VEHICLE_SYSTEM_STATUS 2000
#define MESSAGE_TYPE_VEHICLE_GLOBAL_POSITION 2002

#define MESSAGE_LEN_VEHICLE_SYSTEM_STATUS 12
#define MESSAGE_LEN_VEHICLE_GLOBAL_POSITION 32

#define PACKET_LEN_VEHICLE_SYSTEM_STATUS 25
#define PACKET_LEN_VEHICLE_GLOBAL_POSITION 45

int8_t calculate_checksum(uint8_t *data, int data_len) {
  int8_t checksum = 0;
  
  for(int i=0; i<data_len; i++) {
    checksum += ~data[i] + 1;
  }
  
  return checksum;
}

#define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )	
#define ntohs(x) htons(x)	
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \	
                   ((x)<< 8 & 0x00FF0000UL) | \	
                   ((x)>> 8 & 0x0000FF00UL) | \	
                   ((x)>>24 & 0x000000FFUL) )

#define ntohl(x) htonl(x)

#define VEHICLE_MODE_MANUAL 1
#define VEHICLE_MODE_NAVIGATION 3
#define VEHICLE_MODE_COLLISION_AVOIDANCE 4

#define VEHICLE_STATE_READY 1

void send_system_status() {
  header_t header;
  header.source_id = SOURCE_ID;
  header.dest_id = DEST_ID;
  header.seq = SEQUENCE;
  header.ttl = TTL;
  header.message_type = htons(MESSAGE_TYPE_VEHICLE_SYSTEM_STATUS);
  header.message_length = htons(MESSAGE_LEN_VEHICLE_SYSTEM_STATUS);
  
  vehicle_system_status_t data;
  data.time_stamp = now();
  data.vehicle_id = htons  (SOURCE_ID);
  
  if(curr_airspace.planes[PLANE_ID].curr_state == STATE_MANUAL) {
    data.vehicle_mode = VEHICLE_MODE_MANUAL;
  } else if(curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_START || curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_FINISH) {
    data.vehicle_mode = VEHICLE_MODE_NAVIGATION;
  } else if(curr_airspace.planes[PLANE_ID].curr_state == STATE_COLLISION_DETECTED) {
    data.vehicle_mode = VEHICLE_MODE_COLLISION_AVOIDANCE;
  }
  
  //data.vehicle_state = VEHICLE_STATE_READY;
  
  data.vehicle_state = curr_airspace.planes[PLANE_ID].curr_state;
  
  packet_system_status_t packet;
  packet.sync = htonl(SYNC);
  packet.header = header;
  memset(packet.data, 0, MESSAGE_LEN_VEHICLE_SYSTEM_STATUS);
  memcpy(packet.data, &data, MESSAGE_LEN_VEHICLE_SYSTEM_STATUS);
  packet.checksum = calculate_checksum((uint8_t *)&packet, PACKET_LEN_VEHICLE_SYSTEM_STATUS);
  
  int written = xbee.write((uint8_t *)&packet, PACKET_LEN_VEHICLE_SYSTEM_STATUS);

//  Printer.println("SENDING SYSTEM STATUS PACKET");
//  Printer.print("  WROTE: ");
//  Printer.println(written);
}

void send_global_position() {  
  header_t header;
  header.source_id = SOURCE_ID;
  header.dest_id = DEST_ID;
  header.seq = SEQUENCE;
  header.ttl = TTL;
  header.message_type = htons(MESSAGE_TYPE_VEHICLE_GLOBAL_POSITION);
  header.message_length = htons(MESSAGE_LEN_VEHICLE_GLOBAL_POSITION);
  
  vehicle_global_position_t data;
  data.time_stamp = now();
  data.vehicle_id = htons(SOURCE_ID);
  data.lat = htonl(curr_airspace.planes[PLANE_ID].curr_position.lat);
  data.lon = htonl(curr_airspace.planes[PLANE_ID].curr_position.lon);
  data.alt = htonl(curr_airspace.planes[PLANE_ID].curr_position.alt);
  data.x_speed = htons(curr_airspace.planes[PLANE_ID].curr_position.vx / 100);
  data.y_speed = htons(curr_airspace.planes[PLANE_ID].curr_position.vy / 100);
  data.z_speed = htons(curr_airspace.planes[PLANE_ID].curr_position.vz / 100);
  data.heading = htonl(curr_airspace.planes[PLANE_ID].curr_position.heading * 1 * 10^4);
  
  packet_global_position_t packet;
  packet.sync = htonl(SYNC);
  packet.header = header;
  memset(packet.data, 0, MESSAGE_LEN_VEHICLE_GLOBAL_POSITION);
  memcpy(packet.data, &data, MESSAGE_LEN_VEHICLE_GLOBAL_POSITION);
  packet.checksum = calculate_checksum((uint8_t *)&packet, PACKET_LEN_VEHICLE_GLOBAL_POSITION);
  
  int written = xbee.write((uint8_t *)&packet, PACKET_LEN_VEHICLE_GLOBAL_POSITION);
 
//  Printer.println("WRITTEN:");
//  Printer.print("  SYNC: ");
//  Printer.println(packet->sync);
//  Printer.println("  HEADER:");
//  Printer.print("    SOURCE: ");
//  Printer.println(packet->header.source_id);
//  Printer.print("    DEST: ");
//  Printer.println(packet->header.dest_id);
//  Printer.print("    SEQ: ");
//  Printer.println(packet->header.seq);
//  Printer.print("    TTL: ");
//  Printer.println(packet->header.ttl);
//  Printer.print("    MESSAGE TYPE: ");
//  Printer.println(packet->header.message_type);
//  Printer.print("    MESSAGE LENGTH: ");
//  Printer.println(packet->header.message_length);
//  Printer.print("  CHECKSUM: ");
//  Printer.println(calculate_checksum(packet->data, packet->header.message_length) - packet->checksum);

//  Printer.println("SENDING GLOBAL POSITION PACKET");
//  Printer.print("  WROTE: ");
//  Printer.println(written);
}

uint8_t *buffer;
int buffer_len = 0;

void receive_info() {
  
  int avail = xbee.available();
  
  if(avail > 0) {
    
    uint8_t *temp = (uint8_t *)calloc(sizeof(uint8_t), avail);
    int num_bytes_read = xbee.readBytes((char *)temp, avail);
    
    uint8_t *new_buffer = (uint8_t *)calloc(sizeof(uint8_t), buffer_len + num_bytes_read);
    if(buffer != NULL) {
      memcpy(new_buffer, buffer, buffer_len);
      free(buffer);
    }
    memcpy(new_buffer + buffer_len, temp, num_bytes_read);
    free(temp);
    
    buffer = new_buffer;
    buffer_len += num_bytes_read;
  }
  
  while(buffer_len >= 4) {
    int32_t *sync_check = (int32_t *)buffer;
    
//    Printer.print("SYNC: ");
//    Printer.println(*sync_check);
//    Printer.print("BUFFER LEN: ");
//    Printer.println(buffer_len);
    
    if(*sync_check == htonl(SYNC)) {
      
      if(buffer_len >= 12) {
        process_buffer();
        break;
      } else {
        break;
      }
    } else {
      uint8_t *realloc_buffer = (uint8_t *)calloc(buffer_len - 1, 1);
      memcpy(realloc_buffer, buffer + 1, buffer_len - 1); 
      free(buffer);
      
      buffer = realloc_buffer;
      buffer_len -= 1;
    }
  }
}

#define LOWER_LAT 35.0
#define HIGHER_LAT 36.0
#define LOWER_LON -120.0
#define HIGHER_LON -121.0
#define LOWER_ALT 0
#define HIGHER_ALT 250

void process_buffer() {
  header_t *header = (header_t *)&buffer[4];
  
//  Printer.println("PACKET:");
//  Printer.println("  HEADER:");
//  Printer.print("    SOURCE: ");
//  Printer.println(header->source_id);
//  Printer.print("    DEST: ");
//  Printer.println(header->dest_id);
//  Printer.print("    SEQ: ");
//  Printer.println(header->seq);
//  Printer.print("    TTL: ");
//  Printer.println(header->ttl);
//  Printer.print("    MESSAGE TYPE: ");
//  Printer.println(ntohs(header->message_type));
//  Printer.print("    MESSAGE LENGTH: ");
//  Printer.println(ntohs(header->message_length));
//  Printer.print("  CHECKSUM: ");
//  Printer.println(calculate_checksum(packet->data) - packet->checksum);
  
  if(ntohs(header->message_type) == MESSAGE_TYPE_VEHICLE_SYSTEM_STATUS) {
    if(buffer_len >= PACKET_LEN_VEHICLE_SYSTEM_STATUS) {
      packet_system_status_t *packet = (packet_system_status_t *)buffer;
      vehicle_system_status_t *vss = (vehicle_system_status_t *)packet->data;
      
      curr_airspace.planes[OTHER_PLANE_ID].curr_state = vss->vehicle_state;
      
      uint8_t *realloc_buffer = (uint8_t *)calloc(buffer_len - PACKET_LEN_VEHICLE_SYSTEM_STATUS, 1);
      memcpy(realloc_buffer, buffer + PACKET_LEN_VEHICLE_SYSTEM_STATUS, buffer_len - PACKET_LEN_VEHICLE_SYSTEM_STATUS);
      free(buffer);
      
      buffer = realloc_buffer;
      buffer_len = buffer_len - PACKET_LEN_VEHICLE_SYSTEM_STATUS;
    }
  } else if(ntohs(header->message_type) == MESSAGE_TYPE_VEHICLE_GLOBAL_POSITION) {
    if(buffer_len >= PACKET_LEN_VEHICLE_GLOBAL_POSITION) {
      packet_global_position_t *packet = (packet_global_position_t *)buffer;
      vehicle_global_position_t *vgp = (vehicle_global_position_t *)packet->data;
      
//      Printer.print("LAT: ");
//      Printer.println(ntohl(vgp->lat) / SIGFIG_CONVERT);
//      Printer.print("LON: ");
//      Printer.println((int32_t)ntohl(vgp->lon) / SIGFIG_CONVERT);
//      Printer.print("ALT: ");
//      Printer.println(ntohl(vgp->alt));
          
      if((ntohl(vgp->lat) / SIGFIG_CONVERT < HIGHER_LAT && ntohl(vgp->lat) / SIGFIG_CONVERT > LOWER_LAT) &&
         ((int32_t) ntohl(vgp->lon) / SIGFIG_CONVERT < LOWER_LON && (int32_t)ntohl(vgp->lon) / SIGFIG_CONVERT > HIGHER_LON) &&
         (ntohl(vgp->alt) < HIGHER_ALT && ntohl(vgp->alt) > LOWER_ALT)) {
        curr_airspace.planes[OTHER_PLANE_ID].curr_position.lat = ntohl(vgp->lat);
        curr_airspace.planes[OTHER_PLANE_ID].curr_position.lon = ntohl(vgp->lon);
        curr_airspace.planes[OTHER_PLANE_ID].curr_position.alt = ntohl(vgp->alt);
        curr_airspace.planes[OTHER_PLANE_ID].curr_position.time_now = vgp->time_stamp;
        
        print_current_airspace();
      }
      
      uint8_t *realloc_buffer = (uint8_t *)calloc(buffer_len - PACKET_LEN_VEHICLE_GLOBAL_POSITION, 1);
      memcpy(realloc_buffer, buffer + PACKET_LEN_VEHICLE_GLOBAL_POSITION, buffer_len - PACKET_LEN_VEHICLE_GLOBAL_POSITION);
      free(buffer);
      
      buffer = realloc_buffer;
      buffer_len = buffer_len - PACKET_LEN_VEHICLE_GLOBAL_POSITION;
    }
  } else {
    uint8_t *realloc_buffer = (uint8_t *)calloc(buffer_len - 1, 1);
    memcpy(realloc_buffer, buffer + 1, buffer_len - 1);
    free(buffer);
    
    buffer = realloc_buffer;
    buffer_len = buffer_len - 1;
  }
}

/* SENSE AND AVOID */

#define TARGET_SYSTEM MAV_TYPE_FIXED_WING
#define TARGET_COMPONENT MAV_COMP_ID_ALL
#define MAV_FRAME MAV_FRAME_GLOBAL
//#define MAV_COMMAND MAV_CMD_NAV_WAYPOINT
#define MAV_COMMAND MAV_CMD_NAV_LOITER_UNLIM
#define IS_CURRENT 1
#define AUTO_CONTINUE 0
//#define HOLD_TIME 0
#define ACCEPT_RADIUS 60
//#define PASS_THROUGH 0
#define YAW_ANGLE 0

void sense_and_avoid() {
  sense_collision();
  state_machine();
}

#define ACCEPTABLE_DISTANCE_AWAY 160

void sense_collision() {
  //calculate distance from other aircraft
  double dist_away = calculate_distance_between_points();
  
//  Printer.print("DISTANCE AWAY: ");
//  Printer.println(dist_away);
  
  if(dist_away <= ACCEPTABLE_DISTANCE_AWAY) {
    if(curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_FINISH) {
//      Printer.println("COLLISION DETECTED");
      curr_airspace.planes[PLANE_ID].next_state = STATE_COLLISION_DETECTED;
    }
  } else {
    if(curr_airspace.planes[PLANE_ID].curr_state == STATE_COLLISION_DETECTED) {
      curr_airspace.planes[PLANE_ID].next_state = STATE_GO_TO_FINISH;
    }
  }
}

#define RAD_EARTH_M 6371000

double to_radians(double angle_in_degrees) {
  return angle_in_degrees * PI / 180.0;
}

double calculate_distance_between_points() {
  double lat1 = (double)curr_airspace.planes[OTHER_PLANE_ID].curr_position.lat / SIGFIG_CONVERT;
  double lat2 = (double)curr_airspace.planes[PLANE_ID].curr_position.lat / SIGFIG_CONVERT;
  
  double lon1 = (double)curr_airspace.planes[OTHER_PLANE_ID].curr_position.lon / SIGFIG_CONVERT;
  double lon2 = (double)curr_airspace.planes[PLANE_ID].curr_position.lon / SIGFIG_CONVERT;
  
  double dLat = to_radians(lat2 - lat1);
  double dLon = to_radians(lon2 - lon1);
  
  double dist_rads = sqrt(dLat * dLat + dLon * dLon);
  
  return RAD_EARTH_M * dist_rads;
}

void state_machine() {
  
  if(curr_airspace.planes[PLANE_ID].curr_state != curr_airspace.planes[PLANE_ID].next_state) {
    curr_airspace.planes[PLANE_ID].curr_state = curr_airspace.planes[PLANE_ID].next_state;
    
    if(curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_START) {
      curr_airspace.planes[PLANE_ID].curr_waypoint = curr_airspace.planes[PLANE_ID].start_waypoint;
      assign_current_waypoint();
    } else if(curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_FINISH) {
      curr_airspace.planes[PLANE_ID].curr_waypoint = curr_airspace.planes[PLANE_ID].end_waypoint;
      assign_current_waypoint();
    } else if(curr_airspace.planes[PLANE_ID].curr_state == STATE_COLLISION_DETECTED) {
      /* CALCULATING NEW WAYPOINT */
      calculate_new_waypoint(&curr_airspace.planes[PLANE_ID].curr_waypoint);
      assign_current_waypoint();
    }
  }
  
//  Printer.print("CURRENT STATE: ");
//  Printer.println(curr_airspace.planes[PLANE_ID].curr_state);
}

void assign_current_waypoint() {
  
  send_count();
  
  /* SEND HOME */
  wait_for_request();
  send_waypoint(0);
  
  /* SEND POINT */
  wait_for_request();
  send_waypoint(1);
  
  wait_for_ack();
  send_ack();
  
  send_current_waypoint();
  wait_for_current();
  
  /* MISSION START */
  send_mission_start();
  
  /* SET MODE */
  set_mode();
}

void send_count() {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_count_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, TARGET_COMPONENT, 2);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
  //Printer.println("  SENDING COUNT");
}

void wait_for_request() {
  while(true) {
    while(Serial.available()) {
      uint8_t c = Serial.read();
    
      if(mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
        
//        Printer.print("MSG ID: ");
//        Printer.println(msg.msgid);
        
//        if(msg.msgid == 253) {
//          mavlink_statustext_t *statustext = (mavlink_statustext_t *)msg.payload64;
//          
//          Printer.print("  SEVERITY: ");
//          Printer.println(statustext->severity);
//          Printer.print("  TEXT: ");
//          Printer.println(statustext->text);
//        }
        
        if(msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST) {
          return;
        }
      }
    }
  }
}

void send_waypoint(uint16_t seq) {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  uint8_t is_current;
  
  float lat, lon, alt;
  
  /* HOME IS CURRENT LOCATION */
  if(seq == 0) {
    lat = (float)curr_airspace.planes[PLANE_ID].curr_position.lat / SIGFIG_CONVERT;
    lon = (float)curr_airspace.planes[PLANE_ID].curr_position.lon / SIGFIG_CONVERT;
    alt = (float)curr_airspace.planes[PLANE_ID].curr_position.alt;
    is_current = 0;
  /* OTHER WAYPOINT TO GO TO */
  } else if (seq == 1) {
    lat = (float)curr_airspace.planes[PLANE_ID].curr_waypoint.waypoint.lat / SIGFIG_CONVERT;
    lon = (float)curr_airspace.planes[PLANE_ID].curr_waypoint.waypoint.lon / SIGFIG_CONVERT;
    alt = (float)curr_airspace.planes[PLANE_ID].curr_waypoint.waypoint.alt;
    is_current = 1;
  }
  
//  Printer.println("  TRYING TO ASSIGN POINT: ");
//  Printer.print("    LAT: ");
//  Printer.println(lat, 7);
//  Printer.print("    LON: ");
//  Printer.println(lon, 7);
//  Printer.print("    ALT: ");
//  Printer.println(alt, 3);
  
  mavlink_msg_mission_item_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, TARGET_COMPONENT, 
    seq, MAV_FRAME, MAV_COMMAND, is_current, AUTO_CONTINUE, 0, 0, ACCEPT_RADIUS, 
    YAW_ANGLE, lat, lon, alt);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
  //Printer.println("  MISSION ITEM SENT");
}

void wait_for_ack() {
  while(true) {
    while(Serial.available()) {
      uint8_t c = Serial.read();
    
      if(mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
        
//        Printer.print("MSG ID: ");
//        Printer.println(msg.msgid);
        
//        if(msg.msgid == 253) {
//          mavlink_statustext_t *statustext = (mavlink_statustext_t *)msg.payload64;
//          
//          Printer.print("  SEVERITY: ");
//          Printer.println(statustext->severity);
//          Printer.print("  TEXT: ");
//          Printer.println(statustext->text);
//        }
        
        if(msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
//          mavlink_mission_ack_t *mission_ack = (mavlink_mission_ack_t *)msg.payload64;
//          
//          Printer.print("  ACK TYPE: ");
//          Printer.println(mission_ack->type);
          
          return;
        }
      }
    }
  }
}

void send_ack() {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, TARGET_COMPONENT, MAV_MISSION_ACCEPTED);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
//  Printer.println("  MISSION ACK SENT");
}

void send_current_waypoint() {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_set_current_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, TARGET_COMPONENT, 1);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
//  Printer.println("  MISSION SET CURRENT SENT");
}

void wait_for_current() {
  while(true) {
    while(Serial.available()) {
      uint8_t c = Serial.read();
    
      if(mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
        
//        Printer.print("MSG ID: ");
//        Printer.println(msg.msgid);
        
//        if(msg.msgid == 253) {
//          mavlink_statustext_t *statustext = (mavlink_statustext_t *)msg.payload64;
//          
//          Printer.print("  SEVERITY: ");
//          Printer.println(statustext->severity);
//          Printer.print("  TEXT: ");
//          Printer.println(statustext->text);
//        }
        
        if(msg.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
//          mavlink_mission_current_t *mission_current = (mavlink_mission_current_t *)msg.payload64;
//          
//          Printer.print("  SEQ CURRENT: ");
//          Printer.println(mission_current->seq);
          
          return;
        }
      }
    }
  }
}

void send_mission_start() {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_item_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, TARGET_COMPONENT, 
    1, MAV_FRAME, MAV_CMD_MISSION_START, 1, 0, 1, 1, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
}

#define BASE_MODE_AUTO 217 

void set_mode() {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_set_mode_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, BASE_MODE_AUTO, CUSTOM_MODE_AUTO);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
}

void clear_waypoints() {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_clear_all_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, TARGET_COMPONENT);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
}

#define NEW_WAYPOINT_DISTANCE_AWAY .0045045045

void calculate_new_waypoint(void *buff) {
  waypoint_t *new_waypoint = (waypoint_t *)buff;
  
  double lat1 = (double)curr_airspace.planes[PLANE_ID].curr_position.lat / SIGFIG_CONVERT;
  double lat2 = (double)curr_airspace.planes[OTHER_PLANE_ID].curr_position.lat / SIGFIG_CONVERT;
  
  double lon1 = (double)curr_airspace.planes[PLANE_ID].curr_position.lon / SIGFIG_CONVERT;
  double lon2 = (double)curr_airspace.planes[OTHER_PLANE_ID].curr_position.lon / SIGFIG_CONVERT;
  
  double d_lat = (lat2 - lat1);
  double sgn_lat = d_lat < 0.0 ? -1.0 : 1.0;
  double d_lon = (lon2 - lon1);
  
  double slope = d_lon / d_lat;
  
  double mid_lat = (lat2 + lat1) / 2.0;
  double mid_lon = (lon2 + lon1) / 2.0;
  
  double new_lat = mid_lat + sgn_lat * NEW_WAYPOINT_DISTANCE_AWAY * sqrt((slope * slope) / (1 + slope * slope));
  double new_lon = mid_lon + sgn_lat * sqrt(NEW_WAYPOINT_DISTANCE_AWAY * NEW_WAYPOINT_DISTANCE_AWAY - (new_lat - mid_lat) * (new_lat - mid_lat));
//  
//  double denominator = sqrt(slope * slope + 1.0);
//  
//  double new_lat = mid_lat + sgn_lat * (NEW_WAYPOINT_DISTANCE_AWAY * slope) / denominator;
//  double new_lon = -1.0 * sgn_lat * slope / denominator + mid_lon;
  
//  Printer.print("LAT1: ");
//  Printer.println(lat1, 7);
//  Printer.print("LON1: ");
//  Printer.println(lon1, 7);
//  
//  Printer.print("LAT2: ");
//  Printer.println(lat2, 7);
//  Printer.print("LON2: ");
//  Printer.println(lon2, 7);
//  
//  Printer.print("SLOPE: ");
//  Printer.println(slope, 7);
//  
//  Printer.print("MID LAT: ");
//  Printer.println(mid_lat, 7);
//  Printer.print("MID LON: ");
//  Printer.println(mid_lon, 7);
//  
//  Printer.print("NEW LAT: ");
//  Printer.println(new_lat, 7);
//  Printer.print("NEW LON: ");
//  Printer.println(new_lon, 7);
  
  new_waypoint->waypoint.lat = new_lat * SIGFIG_CONVERT;
  new_waypoint->waypoint.lon = new_lon * SIGFIG_CONVERT;
  new_waypoint->waypoint.alt = PLANE_ALT;
}

/* HELPER */
void print_current_airspace() {
  Printer.print("NUM PLANES: ");
  Printer.println(NUM_PLANES);
  
  int i;
  for(i=0; i<NUM_PLANES; i++) {
    Printer.print("  PLANE ID: ");
    Printer.println(i);
    Printer.print("  PLANE STATE:");
    Printer.println(curr_airspace.planes[i].curr_state);
    Printer.println("  CURRENT POSITION:");
    Printer.print("    LAT: ");
    Printer.println(curr_airspace.planes[i].curr_position.lat);
    Printer.print("    LON: ");
    Printer.println(curr_airspace.planes[i].curr_position.lon);
    Printer.print("    ALT: ");
    Printer.println(curr_airspace.planes[i].curr_position.alt);
    Printer.print("    TIME NOW: ");
    Printer.println(curr_airspace.planes[i].curr_position.time_now);
    Printer.println();
  }
}


