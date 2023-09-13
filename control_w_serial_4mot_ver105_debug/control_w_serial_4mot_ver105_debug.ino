#include <string.h>
#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#endif

#define pos_close 10
#define pos_medium 400


const uint8_t DXL_ID = 1;
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;
const uint8_t DXL_ID4 = 4;
const float DXL_PROTOCOL_VERSION = 2.0;


uint16_t pos_p_gain[4] = {400,400,400,400}; // default value = 400

const uint8_t DXL_IDs[4] = {DXL_ID, DXL_ID2, DXL_ID3, DXL_ID4};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;
int mt1_home = 2047;
int mt2_home = 2047;
int mt3_home = 2047;
int mt4_home = 2047;
int32_t pwm_slope = 140;

int goalpos_tmp[4] = {mt1_home, mt2_home, mt3_home, mt4_home};
int profVeloArr[4] = {0,0,0,0};
bool mtActivate[4] = {false,false,false,false};
int mtActivateD = 0;
void setup() {
  // put your setup code here, to run once:

  // Use UART port of DYNAMIXEL Shield to debug.
  Serial.begin(9600);
  Serial2.begin(9600);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);
  dxl.ping(DXL_ID2);

  // Turn off torque when configuring items in EEPROM area
  for(int i = 0; i<4; i++){
    dxl.torqueOff(DXL_IDs[i]);
    dxl.setOperatingMode(DXL_IDs[i], OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_IDs[i], 30000);//new
    dxl.torqueOn(DXL_IDs[i]);
  }
}

void loop() {
  sendcurrentpos();
  if(Serial.available()>0){
    String input = Serial.readStringUntil('\n');

    int values[12];
    int index = 0;
    int lastindex = -1;
    for(int i = 0; i<input.length();i++){
      if(input[i] == ','){
        values[index++] = input.substring(lastindex + 1, i).toInt();
        lastindex = i;
      }
    }
    values[index] = input.substring(lastindex+1).toInt();
    setProfAcc(values[0]); //new
    for(int i = 0; i<4; i++){
      goalpos_tmp[i] = values[i+4];
      if(values[i+8] != 0){
        pos_p_gain[i] = values[i+8];
      }
    }
    if(values[3] != mtActivateD) {
      motorActiveSW(values[3]);
      mtActivateD = values[3];
    }

    switch(values[2]){
      case 1:
        motorSpeedRegulation(values[1]);
        break;
      case 0:
        motorSpeedIdent(values[1]);
        break;
      default:
        break;
    }
  for(int i = 0; i<4; i++){
    if(!mtActivate[i]) continue;//
    dxl.setGoalPosition(DXL_IDs[i], goalpos_tmp[i]);
    if(dxl.readControlTableItem(POSITION_P_GAIN, DXL_IDs[i]) != pos_p_gain[i]) dxl.writeControlTableItem(POSITION_P_GAIN, DXL_IDs[i], pos_p_gain[i]);
  } 
  sendcurrentpos();
  }
}

void sendcurrentpos(){
  Serial.print(dxl.readControlTableItem(PROFILE_ACCELERATION, DXL_ID));  
  Serial.print(',');
  for(int i = 0; i<4; i++){
    Serial.print(int(dxl.getPresentPosition(DXL_IDs[i])));
    Serial.print(',');
  }
  Serial.println(0);
}

// new

void writeProfAcc(int32_t value){
  for(int i = 0; i<4; i++){
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_IDs[i], value);
  }
}

void setProfAcc(int gear_input){
  switch(gear_input){
    case 1:
    writeProfAcc(25);
    break;
    case 2:
    writeProfAcc(50);
    break;
    case 3:
    writeProfAcc(75);
    break;
    default:
    writeProfAcc(0);
    break;
  }
}

void motorActiveSW(int torqueBinaryD){
  int dataTmp = torqueBinaryD;
  for(int i = 0; i<4; i++){
    if(dataTmp % 10 == 1) mtActivate[3-i] = true;
    else if(dataTmp % 10 == 0) mtActivate[3-i] = false;
    dataTmp /= 10;
  }
}

void motorSpeedRegulation(int maxSpeed){
  int posdiff[4];
  int maxDiff = 0;
  int maxDiffIndex = 0;
  for(int i = 0; i<4; i++){
    if(!mtActivate[i]) continue;
    posdiff[i] = abs(dxl.getPresentPosition(DXL_IDs[i]) - goalpos_tmp[i]);
    if(posdiff[i] >= maxDiff){
      maxDiff = posdiff[i];
      maxDiffIndex = i;
    }
  }
  for(int i = 0; i<4; i++){
    //if(!mtActivate[i] || dxlIsRun()) continue;
    profVeloArr[i] = (int)(maxSpeed * (float)posdiff[i] / maxDiff);
    dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_IDs[i],profVeloArr[i]);
  }
}

void motorSpeedIdent(int idntSpeed){
  for(int i = 0; i<4; i++){
    if(!mtActivate[i]) continue;
    profVeloArr[i] = idntSpeed;
    dxl.writeControlTableItem(PROFILE_VELOCITY,DXL_IDs[i],profVeloArr[i]);
  }
}