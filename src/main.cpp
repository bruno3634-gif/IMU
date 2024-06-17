#include <Arduino.h>
#include <LSM6DSOXSensor.h>
#include <ESP32_CAN.h>
#include <BluetoothSerial.h>

TWAI_Interface CAN1(1000, 21, 22);

#define SR 104 // Sample rate. Options are: 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333 and 6667 Hz.
#define WTM_LV 199 // Watermark threshold level. Max samples in this FIFO configuration is 512 (accel and gyro only).

/** LSM6DSOX i2c address:
 * LSM6DSOX_I2C_ADD_L: 0x6A (default)
 * LSM6DSOX_I2C_ADD_H: 0x6B 
 **/
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);
BluetoothSerial bl;

void setup() {

  Serial.begin(115200);
  bl.begin("IMU");
  // Comment this line to skip waiting for serial:
  while(!Serial) delay(10);
  
  // i2c, fast mode
  Wire.begin();
  Wire.setClock(400000);

  // Initialize sensors
  lsm6dsoxSensor.begin();
  if (lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK) {
    Serial.println("Success enabling accelero and gyro");
  } else {
    Serial.println("Error enabling accelero and gyro");
    abort();
  }

  // Check device id
  uint8_t id;
  lsm6dsoxSensor.ReadID(&id);
  if (id != LSM6DSOX_ID) {
    Serial.println("Wrong id for LSM6DSOX sensor. Check that device is plugged");
    abort();
  } else {
    Serial.println("Success checking id for LSM6DSOX sensor");
  }

  // Set accelerometer scale. Available values are: 2, 4, 8, 16 G
  lsm6dsoxSensor.Set_X_FS(2);

  
  // Set gyroscope scale. Available values are: 125, 250, 500, 1000, 2000 dps
  lsm6dsoxSensor.Set_G_FS(250);
  
  // Set accelerometer Output Data Rate. Available values are: 1.6, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
  lsm6dsoxSensor.Set_X_ODR(SR);
  // Set gyroscope Output Data Rate. Available values are 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
  lsm6dsoxSensor.Set_G_ODR(SR);
  
  // Set FIFO Batch Data Rate for accelerometer and gyroscope. Available values are: 0, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
  lsm6dsoxSensor.Set_FIFO_X_BDR(SR);
  lsm6dsoxSensor.Set_FIFO_G_BDR(SR);

  /** Set FIFO operation mode. Available values are:
   * LSM6DSOX_BYPASS_MODE: FIFO is not used, the buffer content is cleared
   * LSM6DSOX_FIFO_MODE: bufer continues filling until it becomes full. Then it stops collecting data.
   * LSM6DSOX_STREAM_MODE: continuous mode. Older data are replaced by the new data.
   * LSM6DSOX_STREAM_TO_FIFO_MODE: FIFO buffer starts operating in Continuous mode and switches to FIFO mode when an event condition occurs.
   * LSM6DSOX_BYPASS_TO_STREAM_MODE: FIFO buffer starts operating in Bypass mode and switches to Continuous mode when an event condition occurs.
   * */
  lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // flush any previous value in FIFO before start
  lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // start batching in continous mode
  
  // Set FIFO watermark level. Can be used to check when the number of samples in buffer reaches this defined threshold level.
  lsm6dsoxSensor.Set_FIFO_Watermark_Level(WTM_LV);
  
  // FIFO size can be limited to the watermark level by setting the STOP_ON_WTM flag to 1
  //lsm6dsoxSensor.Set_FIFO_Stop_On_Fth(1);

  Serial.println("Starting...");
}

void loop() {

  static uint8_t wtmStatus = 0; // FIFO watermark status
  uint8_t fullStatus = 0; // FIFO full status
  uint16_t numSamples = 0; // number of samples in FIFO
  uint8_t Tag; // FIFO data sensor identifier
  int32_t acceleration[3]; // X, Y, Z accelerometer values in mg
  int32_t rotation[3]; // X, Y, Z giroscope values in mdps

  // Get number of samples in buffer
  lsm6dsoxSensor.Get_FIFO_Num_Samples(&numSamples);
  Serial.print("Samples in FIFO: "); Serial.println(numSamples);
  Serial.flush();

  // Check if FIFO threshold level was reached.
  lsm6dsoxSensor.Get_FIFO_Watermark_Status(&wtmStatus);

  if (wtmStatus != 0) {
    Serial.println("-- FIFO Watermark level reached!, fetching data.");
    Serial.flush();

    // fetch data from FIFO
    for (uint16_t i = 0; i < WTM_LV; i++) {

      lsm6dsoxSensor.Get_FIFO_Tag(&Tag); // get data identifier
      
      // Get gyroscope data
      if (Tag == 1) { 
        lsm6dsoxSensor.Get_FIFO_G_Axes(rotation);
        #if 1 // set to 1 for printing values
        Serial.print("mdps: "); Serial.print(rotation[0]); 
        Serial.print(", "); Serial.print(rotation[1]); 
        Serial.print(", "); Serial.print(rotation[2]); 
        Serial.println();
        Serial.flush();
        #endif
      } 
      
      // Get accelerometer data
      else if (Tag == 2) {
        lsm6dsoxSensor.Get_FIFO_X_Axes(acceleration); 
        #if 1 // set to 1 for printing values
        Serial.print("mG: "); Serial.print(acceleration[0]); 
        Serial.print(", "); Serial.print(acceleration[1]); 
        Serial.print(", "); Serial.print(acceleration[2]); 
        Serial.println();
        Serial.flush();
        #endif
      }
    }
  }

  // Check if FIFO is full.
  lsm6dsoxSensor.Get_FIFO_Full_Status(&fullStatus);

  if (fullStatus != 0) {
    Serial.println("-- FIFO is full!, consider reducing Watermark Level or Buffer Data Rate.\nFlushing data from FIFO.");
    Serial.flush();
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // flush FIFO data
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // continue batching
  }
  int X_a = acceleration[0] * 9.80665;
  int Y_a = acceleration[1] * 9.80665;
  int Z_a = acceleration[2] * 9.80665;


  int R_x = rotation[0] * 9.80665;
  int R_y = rotation[1] * 9.80665;
  int R_z = rotation[2] * 9.80665;

  uint8_t A[6];
  
  uint8_t R[6];




    A[0] = (X_a >> 8) & 0xFF;
    A[1] = X_a & 0xFF;

    A[2] = (Y_a >> 8) & 0xFF;
    A[3] = Y_a & 0xFF;

    A[4] = (Z_a >> 8) & 0xFF;
    A[5] = Z_a & 0xFF;



    R[0] = (R_x >> 8) & 0xFF;
    R[1] = R_x & 0xFF;

    R[2] = (R_z >> 8) & 0xFF;
    R[3] = R_z & 0xFF;

    R[4] = (R_y >> 8) & 0xFF;
    R[5] = R_y & 0xFF;
  
delay(10); // FIFO continues batching while we sleep 

CAN1.TXpacketBegin(0x400,0);
for(int i = 0; i<6;i++)
{
  CAN1.TXpacketLoad(A[i]);
}


delay(10);


CAN1.TXpacketBegin(0x401,0);
for(int i = 0; i<6;i++)
{
  CAN1.TXpacketLoad(R[i]);
}

if(bl.available())
{
  bl.print("X: ");
  bl.print(X_a);

  bl.print("  Y: ");
  bl.print(Y_a);

  bl.print("  Z: ");
  bl.println(Z_a);



  bl.print("R_x: ");
  bl.print(R_x);

  bl.print("  R_y: ");
  bl.print(R_y);

  bl.print("  R_z: ");
  bl.println(R_z);
}


}