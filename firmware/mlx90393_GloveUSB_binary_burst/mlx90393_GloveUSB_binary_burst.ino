#include "MLX90393.h" //https://github.com/tesshellebrekers/arduino-MLX90393
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux

QWIICMUX myMux;

MLX90393 mlx0;
MLX90393 mlx1;
MLX90393 mlx2;
MLX90393 mlx3;

uint8_t mlx0_i2c = 0x0C;
uint8_t mlx1_i2c = 0x0D;
uint8_t mlx2_i2c = 0x0E;
uint8_t mlx3_i2c = 0x0F;

//uint8_t mux_id = 7;

MLX90393::txyz data0;
MLX90393::txyz data1;
MLX90393::txyz data2;
MLX90393::txyz data3;

uint8_t status1;
uint8_t i;
void setup(void)
{
  Wire.begin();
  Wire.setClock(100000);
  
  Serial.begin(115200);
  /* Wait for serial on USB platforms. */
  while (!Serial) {}
  Serial.println("Starting Mux");
  while (myMux.begin(0x70) == false)
  {
    Serial.println("Mux not detected. Freezing...");
    delay(5);
  }
  Serial.println("Mux detected");

  setup_mags(0);
  setup_mags(2);
  setup_mags(3);
  setup_mags(5);
  setup_mags(7);

}

void setup_mags(uint8_t mux_id){

  myMux.setPort(mux_id); delay(1);
  
  status1 = mlx0.begin(mlx0_i2c, -1, Wire);
  Serial.println(status1, BIN);
  status1 = mlx1.begin(mlx1_i2c, -1, Wire);
  Serial.println(status1, BIN);
  status1 = mlx2.begin(mlx2_i2c, -1, Wire);
  Serial.println(status1, BIN);
  status1 = mlx3.begin(mlx3_i2c, -1, Wire);
  Serial.println(status1, BIN);
  
  mlx0.startBurst(0xF);
  mlx1.startBurst(0xF);
  mlx2.startBurst(0xF);
  mlx3.startBurst(0xF);
 
}

void get_binary_data(uint8_t mux_id){

  myMux.setPort(mux_id);

  delay(1);
  mlx0.readBurstData(data0);
  mlx1.readBurstData(data1);
  mlx2.readBurstData(data2);
  mlx3.readBurstData(data3);

  Serial.write((byte*)&data0, sizeof(data0));
  Serial.write((byte*)&data1, sizeof(data1)); 
  Serial.write((byte*)&data2, sizeof(data2)); 
  Serial.write((byte*)&data3, sizeof(data3));  
}

void get_data(uint8_t mux_id){

  myMux.setPort(mux_id);
  
  delay(1);
  mlx0.readBurstData(data0);
  mlx1.readBurstData(data1);
  mlx2.readBurstData(data2);
  mlx3.readBurstData(data3);

  Serial.print(data0.x);
  Serial.print("\t");
  Serial.print(data0.y);
  Serial.print("\t");
  Serial.print(data0.z);
  Serial.print("\t");
  Serial.print(data0.t);
  Serial.print("\t");

  Serial.print(data1.x); 
  Serial.print("\t");
  Serial.print(data1.y);
  Serial.print("\t");
  Serial.print(data1.z);
  Serial.print("\t");
  Serial.print(data1.t);
  Serial.print("\t");

  Serial.print(data2.x); 
  Serial.print("\t");
  Serial.print(data2.y);
  Serial.print("\t");
  Serial.print(data2.z);
  Serial.print("\t");
  Serial.print(data2.t);
  Serial.print("\t");

  Serial.print(data3.x); 
  Serial.print("\t");
  Serial.print(data3.y);
  Serial.print("\t");
  Serial.print(data3.z);
  Serial.print("\t");
  Serial.print(data3.t);
  Serial.print("\t");
}

void loop(void) {

  get_binary_data(0);
  get_binary_data(2);
  get_binary_data(3);
  get_binary_data(5);
  get_binary_data(7);

//  get_data(0);
//  get_data(2);
//  get_data(3);
//  get_data(5);
//  get_data(7);
  Serial.println();
  
  delay(10); //20ms delay for each step of burst data rate
}
