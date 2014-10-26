// Arduino MAVLink test code.

#define MAVLINK10

#include <FastSerial.h>
#include <GCS_MAVLink.h>

#define PLANE_ID 1
#define OTHER_PLANE_ID 0

#define SERIAL_BAUD 57600
#define XBEE_BAUD 9600
#define GET_PARAMS_TIMEOUT 200
#define NUM_PLANES 2

#define STATE_MANUAL 1
#define STATE_GO_TO_START 2
#define STATE_GO_TO_FINISH 3
#define STATE_COLLISION_DETECTED 4

#define START_LAT 353284000
#define START_LON -1207518000
#define START_ALT 100000
#define END_LAT 353280000
#define END_LON -1207520000
#define END_ALT 120000

FastSerialPort0(Printer);
FastSerialPort1(Serial);
FastSerialPort3(xbee);

#define SYSTEM_ID 1
#define COMPONENT_ID MAV_COMP_ID_IMU
#define TYPE MAV_TYPE_GCS
#define SYSTEM_TYPE MAV_TYPE_FIXED_WING
#define MAV_AUTOPILOT MAV_AUTOPILOT_INVALID
#define SYSTEM_MODE MAV_MODE_PREFLIGHT
#define CUSTOM_MODE 0
#define SYSTEM_STATE MAV_STATE_STANDBY

#define SIGFIG_CONVERT 10000000.0

typedef struct _gps_t {
  int32_t lat;
  int32_t lon;
  int32_t alt;
  int32_t time_usec;
} gps_t;

typedef struct _waypoint_t {
  gps_t waypoint;
} waypoint_t;

typedef struct _plane_t {
  int curr_state;
  gps_t curr_position;
  waypoint_t curr_waypoint;
  waypoint_t start_waypoint;
  waypoint_t end_waypoint;
} plane_t;

typedef struct _air_space_t {
  plane_t planes[NUM_PLANES];
} air_space_t;

air_space_t curr_airspace;
bool switch_flipped = false;

#define PACKET_LEN 21
#define PACKET_ID 255
                   
typedef struct _packet_t {
  int8_t packet_id;
  int32_t plane_id;
  gps_t curr_location;
} packet_t __attribute__((packed));

uint8_t *buffer;
int buffer_len = 0;

void setup()
{ 
  //delay(30000);
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
  //switch_flipped = true;
  
  if(xbee.available()) {
    uint8_t temp;
    temp = xbee.peek();
    
    while(temp != PACKET_ID) {
      char c;
      xbee.read();
      
      temp = xbee.peek();
    }
  }
    
  Printer.println("FINISHED SETUP");
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
  
  mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMPONENT_ID, &msg, SYSTEM_TYPE, 0, MAV_DATA_STREAM_POSITION, 10, 1);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
  //Printer.println("REQUEST DATA POSITION SENT");
}

mavlink_message_t msg;
mavlink_status_t status;

void poll_for_response() {
  
  while(Serial.available()) {
    uint8_t c = Serial.read();
    
    if(mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
      
      //Printer.print("MESSAGEID: ");
      //Printer.println(msg.msgid);
      
      if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t *heartbeat = (mavlink_heartbeat_t *)msg.payload64;
        
//        Printer.println("HEARTBEAT");
//        Printer.print("  TYPE: ");
//        Printer.println(heartbeat->type);
//        Printer.print("  AUTO: ");
//        Printer.println(heartbeat->autopilot);
//        Printer.print("  VERSION: ");
//        Printer.println(heartbeat->mavlink_version);
//        Printer.println();
      } else if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        mavlink_gps_raw_int_t *gps = (mavlink_gps_raw_int_t *)msg.payload64;
        
        curr_airspace.planes[PLANE_ID].curr_position.lat = gps->lat;
        curr_airspace.planes[PLANE_ID].curr_position.lon = gps->lon;
        curr_airspace.planes[PLANE_ID].curr_position.alt = gps->alt;
        curr_airspace.planes[PLANE_ID].curr_position.time_usec = gps->time_usec;
        
        // GET COG OR VEL????
        // THIS MESSAGE WILL HAVE STATE CHANGE INFORMATION
      } else if (msg.msgid == -1) {
        // flip switch if state has changed
      }
    }
  }
}

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
    Printer.print("    TIME USEC: ");
    Printer.println(curr_airspace.planes[i].curr_position.time_usec);
    Printer.println();
  }
}

void exchange_info() {
  if(PLANE_ID == 0) {
    send_curr_plane_info();
    receive_curr_plane_info();
  } else {
    receive_curr_plane_info();
    send_curr_plane_info();
  }
}

void send_curr_plane_info() {
    packet_t packet;
    packet.packet_id = PACKET_ID;
    packet.plane_id = PLANE_ID;
    packet.curr_location = curr_airspace.planes[PLANE_ID].curr_position;
    
    xbee.write((uint8_t *)&packet, PACKET_LEN);
    //Printer.println("SENDING PACKET");
}

int packets_recv=0;

void receive_curr_plane_info() {
  
    int avail = xbee.available();
    
    //Printer.println("WAITING FOR COMMUNICATION");
    
    while(avail < PACKET_LEN) {
      avail = xbee.available();
    }
    
    //Printer.print("AVAIL: ");
    //Printer.println(avail);

    uint8_t *temp = (uint8_t *)calloc(sizeof(uint8_t), avail);
    
    int num_bytes_read = xbee.readBytes((char *)temp, avail);
    
    uint8_t *new_buffer = (uint8_t *)calloc(sizeof(uint8_t), num_bytes_read + buffer_len);
    memcpy(new_buffer, buffer, buffer_len);
    free(buffer);
    memcpy(new_buffer + buffer_len, temp, num_bytes_read);
    free(temp);
    buffer_len = num_bytes_read+buffer_len;
    
    buffer = new_buffer;
    
    while(buffer_len >= PACKET_LEN) {
      
      packets_recv += 1;
      
      packet_t *packet = (packet_t *)buffer;
      
      if(packets_recv > 100) {
        curr_airspace.planes[packet->plane_id].curr_position.lat = packet->curr_location.lat;
        curr_airspace.planes[packet->plane_id].curr_position.lon = packet->curr_location.lon;
        curr_airspace.planes[packet->plane_id].curr_position.alt = packet->curr_location.alt;
        curr_airspace.planes[packet->plane_id].curr_position.time_usec = packet->curr_location.time_usec;
      }
      
      uint8_t *realloc_buffer = (uint8_t *)calloc(sizeof(uint8_t), buffer_len - PACKET_LEN);
      memcpy(realloc_buffer, buffer + PACKET_LEN, buffer_len - PACKET_LEN);
      free(buffer);
      buffer = realloc_buffer;
      buffer_len -= PACKET_LEN;
    }
}

#define TARGET_SYSTEM MAV_TYPE_FIXED_WING
#define TARGET_COMPONENT MAV_COMP_ID_ALL
#define MAV_FRAME MAV_FRAME_GLOBAL
#define MAV_COMMAND MAV_CMD_NAV_WAYPOINT
#define IS_CURRENT 1
#define AUTO_CONTINUE 0
#define HOLD_TIME 0
#define ACCEPT_RADIUS 10
#define PASS_THROUGH 0
#define YAW_ANGLE 0

void assign_current_waypoint() {
  mavlink_message_t msg;
  uint8_t buff[MAVLINK_MAX_PACKET_LEN];
  
  float lat = (float)curr_airspace.planes[PLANE_ID].curr_waypoint.waypoint.lat / SIGFIG_CONVERT;
  float lon = (float)curr_airspace.planes[PLANE_ID].curr_waypoint.waypoint.lon / SIGFIG_CONVERT;
  float alt = (float)curr_airspace.planes[PLANE_ID].curr_waypoint.waypoint.lat / 1000.0;
  
  mavlink_msg_mission_item_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, TARGET_COMPONENT, 
    1, MAV_FRAME, MAV_COMMAND, IS_CURRENT, AUTO_CONTINUE, HOLD_TIME, ACCEPT_RADIUS, PASS_THROUGH, 
    YAW_ANGLE, lat, lon, alt);
  uint16_t len = mavlink_msg_to_send_buffer(buff, &msg);
  
  Serial.write(buff, len);
  //Printer.println("MISSION ITEM SENT");
}

void calculate_new_waypoint(void *buff) {
  waypoint_t *new_waypoint = (waypoint_t *)buff;
  // CALCULATE WAYPOINT SO THAT PLANE AVOIDS COLLISION 
}

#define ACCEPTABLE_DISTANCE_AWAY 15
#define RAD_EARTH_M 6371000

double to_radians(double angle_in_degrees) {
  return angle_in_degrees * PI / 180.0;
}

double calculate_distance_between_points() {
  double lat1 = (double)curr_airspace.planes[0].curr_position.lat / SIGFIG_CONVERT;
  double lat2 = (double)curr_airspace.planes[1].curr_position.lat / SIGFIG_CONVERT;
  double lon1 = (double)curr_airspace.planes[0].curr_position.lon / SIGFIG_CONVERT;
  double lon2 = (double)curr_airspace.planes[1].curr_position.lon / SIGFIG_CONVERT;
  
  double dLat = to_radians(lat2 - lat1);
  double dLon = to_radians(lon2 - lon1);
  
  double dist_rads = sqrt(dLat * dLat + dLon * dLon);
  
  return RAD_EARTH_M * dist_rads;
}

void sense_collision() {
  //calculate distance from other aircraft
  double dist_away = calculate_distance_between_points();
  
//  Printer.print("DISTANCE AWAY: ");
//  Printer.println(dist_away);
  
  if(dist_away <= ACCEPTABLE_DISTANCE_AWAY) {
    if(curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_FINISH) {
      switch_flipped = true;
    }
  } else {
    if(curr_airspace.planes[PLANE_ID].curr_state == STATE_COLLISION_DETECTED) {
      switch_flipped = true;
    }
  }
}

void state_machine() {
  if(curr_airspace.planes[PLANE_ID].curr_state == STATE_MANUAL) {
    if(switch_flipped) {
      curr_airspace.planes[PLANE_ID].curr_state = STATE_GO_TO_START;
      curr_airspace.planes[PLANE_ID].curr_waypoint = curr_airspace.planes[PLANE_ID].start_waypoint;
      
      assign_current_waypoint();
      switch_flipped = false;
    }
  } else if (curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_START) {
    if(switch_flipped) {
      curr_airspace.planes[PLANE_ID].curr_state = STATE_GO_TO_FINISH;
      curr_airspace.planes[PLANE_ID].curr_waypoint = curr_airspace.planes[PLANE_ID].end_waypoint;
      
      assign_current_waypoint();
      switch_flipped = false;
    }
  } else if (curr_airspace.planes[PLANE_ID].curr_state == STATE_GO_TO_FINISH) {
    if(switch_flipped) {
      curr_airspace.planes[PLANE_ID].curr_state = STATE_COLLISION_DETECTED;
      calculate_new_waypoint(&curr_airspace.planes[PLANE_ID].curr_waypoint);
      
      assign_current_waypoint();
      switch_flipped = false;
    }
  } else if (curr_airspace.planes[PLANE_ID].curr_state == STATE_COLLISION_DETECTED) {
    if(switch_flipped) {
      curr_airspace.planes[PLANE_ID].curr_state = STATE_GO_TO_FINISH;
      curr_airspace.planes[PLANE_ID].curr_waypoint = curr_airspace.planes[PLANE_ID].end_waypoint;
      
      assign_current_waypoint();
      switch_flipped = false;
    }
  } 
}

void loop() {
  send_heartbeat();
  send_request_for_data_stream_position();
  poll_for_response();
  
  //exchange_info();
  
  sense_collision();
  state_machine();
  
  print_current_airspace();
 
  delay(GET_PARAMS_TIMEOUT);
}
