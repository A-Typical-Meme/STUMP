#include <Bitcraze_PMW3901.h>
#include <PixhawkArduinoMAVLink.h>
#include <checksum.h>
#include <mavlink_conversions.h>
#include <mavlink_get_info.h>
#include <mavlink_helpers.h>
#include <mavlink_sha256.h>
#include <mavlink_types.h>
#include <protocol.h>
#include <HardwareSerial.h>

#define flowPin 10

//flow sensor
Bitcraze_PMW3901 flow(flowPin);

//MAVLink
HardwareSerial &hs = Serial;
PixhawkArduinoMAVLink mav(hs);

//-----------------------------variables setup-----------------------------//
// raw or calculations
float sum_flow_x = 0; 
float sum_flow_y = 0;
unsigned long long pxSystemTime = 0;
unsigned long long prevTime = 0;
int sampleCount = 0;
uint64_t timeOffset = 0;

//finalized data
uint32_t flowdt = 0;
uint8_t quality = 0;
float delta_x = 0.0f;
float delta_y = 0.0f;
uint64_t timestamp = 0;
uint8_t sensor_id = 1;

//constants
const uint64_t collectTime = 15000;

//functions proto
void readFlow(float &sum_x, float &sum_y, uint8_t &quality);

void setup() {

  //start serial out for debugging
  Serial.begin(57600);

  //setup flow sensor
  while (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    delay(500);
  }

  //setup MAVLink
  while (!mav.begin()) {
    Serial.println("Initialization of MAVLink on Serial failed");
    delay(500);
  }

  mav.Stream();
  mav.readSysTime(pxSystemTime);
  prevTime = micros();
  timeOffset = micros();

}

void loop() {
  
  //collect flow data until collectionTime is surpassed
  while(flowdt < collectTime) {

    flowdt = micros() - prevTime;
    readFlow(sum_flow_x, sum_flow_y, quality);
    sampleCount++;
  }
  
  //finalize flow data
  delta_x = float(sum_flow_x) / 385.0f;
  delta_y = float(sum_flow_y) / 385.0f;

  //finalize quality data
  if(sampleCount > 0) {
    quality /= sampleCount;
  }
  else quality = 0;

  //rotation?

  //final timestamp
  timestamp = pxSystemTime + (micros() - timeOffset);

  //send data
  mav.sendFlowData(timestamp, flowdt, delta_x, delta_y, quality, sensor_id);

  //reset all values
  delta_x = 0;
  delta_y = 0;
  quality = 0;
  sampleCount = 0;
  prevTime = micros();

}

//function defs
void readFlow(float &sum_x, float &sum_y, uint8_t &quality) {

  //local vars for register reading
  int16_t flow_x, flow_y;
  uint8_t qual;

  //read STUMP data
  flow.readSTUMP(flow_x, flow_y, qual);

  //check if data is valid
  if (flow_x > 240 || flow_y > 240 || flow_x < -240 || flow_y < -240) {
    Serial.println("Bad flow data!");
    return;
  }

  //set referenced values
  quality += qual;
  sum_x += flow_x;
  sum_y += flow_y;

  return;
}

/* plan is to calculate and process data using the PMW3901 driver as a basis,
while using a simple PMW3901 library for starting and basic I/O.

needed values:

int16_t delta_x_raw 
int16_t delta_y_raw 
uint8_t qual 

these are taken directly from data input^^

uint64_t timestamp = hrt_absolute_time();
uint64_t dt_flow = timestamp - _previous_collect_timestamp;

the timestamp should be calculated by adding micros() to the input
original timestamp of the MAVLink heartbeat

then, multiple samples are taken of the above until the sample time is reached
which is set at

const uint64_t _collect_time{15000}

because this happens over multiple loops, the flow in x and y must be summed as follows

int flow_sum_x
int flow_sum_y

finally, once collection is complete, use sums to find final delta_x and delta_y

delta_x = (float)_flow_sum_x / 385.0f;		
delta_y = (float)_flow_sum_y / 385.0f;	

*/
