#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "Adafruit_MAX31855.h"

#define CAN_ID 0x07E8

#define OIL_PIN A5
#define MAP_PIN A3
#define ECT_PIN A1

#define MAXCLK 10
#define MAXCS 9
#define MAXDO 8

Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

#define ECT_LUT_LEN 77

uint16_t ECT_LUT[ECT_LUT_LEN] = {
  1001, 997, 993, 987, 982, 975, 968, 959, 950, 939, 928, 914, 900, 884, 867, 848, 829, 807, 785,
  761, 737, 710, 684, 657, 629, 600, 572, 544, 516, 489, 461, 435, 409, 385, 360, 338, 315, 295,
  275, 257, 239, 223, 207, 194, 180, 168, 156, 145, 135, 126, 117, 109, 101, 95, 88, 82, 77, 72,
  67, 63, 58, 55, 51, 48, 45, 42, 39, 37, 35, 33, 31, 29, 27, 26, 24, 23, 22
};

int16_t ect_lut_idx_to_c(uint8_t idx)
{
  return (int16_t) ((float) idx * 2.5 - 39.5);
}

void setup()
{
  Serial.begin(9600);

  if (thermocouple.begin())
  {
    Serial.println("MAX31855 init ok.");
  }
  else
  {
    Serial.println("You MAX31855 go fuck yourself.");
  }

  if(Canbus.init(CANSPEED_500))
  {
      Serial.println("CAN init ok.");
  }
  else
  {
      Serial.println("You CAN go fuck yourself.");
  }
}

void loop()
{
  tCAN msg;
  while (mcp2515_check_message())
	{
    if (mcp2515_get_message(&msg))
    {
      switch (msg.data[1])
      {
        case 0x01:
          current_data(msg.data[2]);
          break;
        case 0x09:
          vehicle_info(msg.data[2]);
          break;
        default:
          Serial.print("Unsupported service: ");
          Serial.println(msg.data[1], HEX);
          return;
      }
    }
  }
}

void send_msg(tCAN& msg)
{
  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&msg);
}

void current_data(uint8_t pid)
{
  tCAN res;
  res.id = CAN_ID;
  res.header.rtr = 0x00;
  res.header.length = 0x08;
  res.data[1] = 0x41;
  res.data[2] = pid;
  int tmpi;
  uint16_t tmpu16;
  float tmpf;
  double tmpd;
  switch (pid)
  {
    case 0x00:
      // Supported query
      /*
      Supported PIDs:
      0x05
      0x0B
      0x33
      0x78
      */
      res.data[0] = 0x6;
      res.data[3] = 0b00001000;  // 0x01-0x08
      res.data[4] = 0b00100000;  // 0x09-0x10
      res.data[5] = 0b00000000;  // 0x11-0x18
      res.data[6] = 0b00000000;  // 0x19-0x20
      send_msg(res);
      break;
    case 0x05:
      // ECT (C)
      tmpi = analogRead(ECT_PIN);
      for (tmpu16 = 0; tmpu16 < ECT_LUT_LEN; ++tmpu16)
        if (tmpi > ECT_LUT[tmpu16]) break;
      tmpi = ect_lut_idx_to_c(tmpu16);
      res.data[0] = 0x03;
      res.data[3] = (uint8_t) (tmpi + 40);
      send_msg(res);
      break;
    case 0x0B:
      // MAP (kPa)
      tmpf = (float) analogRead(MAP_PIN) * 0.301 - 101;
      tmpu16 = (uint16_t) (tmpf + 200);
      res.data[0] = 0x04;
      res.data[3] = (uint8_t) (tmpu16 >> 8);
      res.data[4] = (uint8_t) (tmpu16 & 0x00FF);
      send_msg(res);
      break;
    case 0x33:
      // Oil pressure (kPa)
      tmpf = (float) analogRead(OIL_PIN) * 0.831 - 90.7;
      if (tmpf < 0) tmpf = 0;
      tmpu16 = (uint16_t) tmpf;
      res.data[0] = 0x04;
      res.data[3] = (uint8_t) (tmpu16 >> 8);
      res.data[4] = (uint8_t) (tmpu16 & 0x00FF);
      send_msg(res);
      break;
    case 0x78:
      // EGT (C)
      tmpd = thermocouple.readCelsius() * 2;
      tmpu16 = (uint16_t) tmpd;
      res.data[0] = 0x04;
      res.data[3] = (uint8_t) (tmpu16 >> 8);
      res.data[4] = (uint8_t) (tmpu16 & 0x00FF);
      send_msg(res);
      break;
    default:
      Serial.print("Unsupported current data PID: ");
      Serial.println(pid, HEX);
      break;
  }
}


void vehicle_info(uint8_t pid)
{
  tCAN res;
  res.id = CAN_ID;
  res.header.rtr = 0x00;
  res.header.length = 0x08;
  switch (pid)
  {
    case 0x0A:
      // ECU name
      res.data[0] = 0x10;
      res.data[1] = 0x0F;
      res.data[2] = 0x49;
      res.data[3] = pid;
      res.data[4] = 'H';  // not sure why this is needed
      res.data[5] = 'H';
      res.data[6] = 'A';
      res.data[7] = 'R';
      send_msg(res);

      res.data[0] = 0x21;
      res.data[1] = 'R';
      res.data[2] = 'I';
      res.data[3] = 'S';
      res.data[4] = 'O';
      res.data[5] = 'N';
      res.data[6] = ' ';
      res.data[7] = 'u';
      send_msg(res);

      res.data[0] = 0x22;
      res.data[1] = 'w';
      res.data[2] = 'u';
      send_msg(res);

      break;
    default:
      Serial.print("Unsupported vehicle info PID: ");
      Serial.println(pid, HEX);
      break;
  }
}
