#include <Servo.h>
#include <Scheduler.h>
#include <Wire.h>
#include <KeyboardController.h>


int FR = 5;
int RL = 6;
int R = 11;
int G = 10;
int B = 9;
int Ra = 0;
int Ga = 0;
int Ba = 0;
int Wa = 0;
int W = 12;
int key;
USBHost usb;
KeyboardController keyboard(usb);

int tmp102Address = 0x48;
int bmp085Address = 0x77;
const byte OSS = 2;
short ac1 ;                 // AC1
short ac2  ;                // AC2
short ac3 ;                 // AC3
unsigned short ac4  ;       // AC4
unsigned short ac5 ;        // AC5
unsigned short ac6 ;        // AC6
short b1 ;                  // B1
short b2 ;                  // B2
short mb ;                  // MB
short mc  ;                 // MC
short md ;                  // MD
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

unsigned long lastmillis=0;
unsigned int servoval;
Servo myservo;
int tempp;
float cel;
float temp1;
float pres;
float rh;
int celsius;
int relativeHumidity;
float pressure;




void setup()
{
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(W, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  Serial.begin(9600);
  Wire.begin();
  //myservo.attach(12);
  analogReadResolution(10);
  analogWriteResolution(8);
  bmp085Calibration();
  Serial1.begin(9600);
  
  
  Serial.println("Program started");
  delay(500);
  //Scheduler.startLoop(loop2);
}


void loop() {
  float cel = getTemperature();
  float temp1 = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pres = bmp085GetPressure(bmp085ReadUP());
  float rh  = getHumidity(cel); 
  usb.Task();
  if(key == 44)
  { 
        Serial.print("Temperature:  ");
        Serial.print(cel); 
        Serial.println(" Degrees Celsius");
        Serial.print("Pressure:  ");
        Serial.print((pres/100), 2); //whole number only.
        Serial.println(" hPa");
        Serial.print("Humidity:  ");
        if (rh >=100) rh = 100;
        Serial.print(rh);
        Serial.println(" RH%");
        Serial.println();//line break
  
}
}


void keyPressed() {
  //Serial.print("Pressed:  ");
  key = keyboard.getOemKey();
  lights();
  drive();
}

void keyReleased() {
  key = 0;
  //Serial.print("Released: ");
  //printKey();
  stop();

}




void drive()
{
    if(key == 82)
    {
      Serial1.print(1);
      
    }
    if(key == 81)
    {
      Serial1.print(2);
    }
    if (key == 80)
    {
      Serial1.print(3);
    }
    if (key == 79);
    {
      Serial1.print(4);
    }
}

void stop()
{
  Serial1.print(0);
}
void lights()
{


  if((key >= 58 && key <= 61) || (key >= 30 && key <= 33))
  {
  if(key == 58) Ra += 10;
  if(key == 59) Ga += 10;
  if(key == 60) Ba += 10;
  if(key == 61) Wa += 10;
  if(key == 30) Ra -= 10;
  if(key == 31) Ga -= 10;
  if(key == 32) Ba -= 10;
  if(key == 33) Wa -= 10;
  if (Ra > 99) Ra = 100;
  if (Ra <=0 ) Ra = 0;
  if (Ga > 99) Ga = 100;
  if (Ga <=0 ) Ga = 0;
  if (Ba > 99) Ba = 100;
  if (Ba <=0 ) Ba = 0;
  if (Wa > 99) Wa = 100;
  if (Wa <=0 ) Wa = 0;

  Serial.print(Ra);
  Serial.print("%R, ");
  Serial.print(Ga);
  Serial.print("%G, ");
  Serial.print(Ba);
  Serial.print("&B, ");
  Serial.print(Wa);
  Serial.println("%W, ");


  analogWrite(R, map(Ra, 0, 100,0,255));
  analogWrite(B, map(Ga, 0, 100,0,255));
  analogWrite(G, map(Ba, 0, 100,0,255));
  analogWrite(W, map(Wa, 0, 100,0,255));
  return;
}
}











float getTemperature(){
  Wire.requestFrom(tmp102Address,2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 

  float celsius = TemperatureSum*0.0625;
  return celsius;
}

void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
  // ac1 = 408;                  // AC1
  // ac2 = -72;                  // AC2
  // ac3 = -14383;               // AC3
  // ac4 = 32741;                // AC4
  // ac5 = 32757;                // AC5
  // ac6 = 23153;                // AC6
  // b1 = 6190;                  // B1
  // b2 = 4;                     // B2
  // mb = -32767;                // MB
  // mc = -8711;                 // MC
  // md = 2868;                  // MD

}

float getHumidity(float degreesCelsius){
  //caculate relative humidity
  float supplyVolt = 3.3;

  // read the value from the sensor:
  int HIH4030_Value = analogRead(A0);
  float voltage = HIH4030_Value/1023. * supplyVolt; // convert to voltage value

  // convert the voltage to a relative humidity
  // - the equation is derived from the HIH-4030/31 datasheet
  // - it is not calibrated to your individual sensor
  //  Table 2 of the sheet shows the may deviate from this line
  float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
  float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius); //temperature adjustment 

  return trueRH;
}


float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(bmp085Address);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(bmp085Address, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(bmp085Address);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(bmp085Address, 2);
  while(Wire.available()<2);
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(bmp085Address);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(bmp085Address);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

