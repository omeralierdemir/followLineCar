#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>


#include <SD.h>

File myFile;

#include <QTRSensors.h>
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
QTRSensorsRC qtrrc((unsigned char[]) {
  A2, A3, A0, A1, 5, 6, 7, 8
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

unsigned int sensorValues[NUM_SENSORS];
const int motor_A_direction = 0; // sol motor
const int motor_B_direction = 10; // sag motor
//const int motor_B_fren = 8;
//const int motor_A_fren = 9;
const int motor_A_speed = 3;
const int motor_B_speed = 9;
int M1_minumum_speed = 140;    //minimum speed of the Motor1
int M2_minumum_speed = 140;  //minimum speed of the Motor2
int M1_maksimum_speed = 190; //max. speed of the Motor1
int M2_maksimum_speed = 190 ; //max. speed of the Motor2
float dinamik = 100000;
float KP = 0.037;
//float KP1 = 0.015;//*//Robotu gozlemle. Hedef yalpalasa bile cizgiyi takip etmesidir. Robot cizgiyi kaybediyorsa degeri dusur, salinim yapmiyor ve yavasliyor ise arttir. ( Note: Kp < Kd)
//float KP2 = 0.001;
//float KP3 = 0.019;//*
//float KP4 = 0.013;
//float KP5 = 0.022;//*
//float KP6 = 0.025;
float KD = 0.12; //Eger cizgiyi takip edebiliyorsak artik kd degerini 1 yap. salinim azalana kadar bu degeri arttirmaya devam et. Deger ondalik sayi olabilir.
float KI = 0;/*Eger robot cizgiyi kararli bir sekilde takip ediyorsa, bu degere 1-0.5 arasinda bir deger ver. Ki degeri cok yuksek ise robot cok hizli salinim yapacaktir.
               eger deger cok dusukse herhangi bir hissedilir fark goremeyeceksin. Sayi az az arttirilmalidir.                 */
int flag = 0;
float  gyroF =0;
int lastError = 0;
int integral = 0;
float gyroT = 0;
void setup()
{
  DDRD |=(1<<0) | (1<<3);
  DDRB |= (1 << 1) | (1<<2);
 
    Serial.begin(9600);
    Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);
   
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
  SD.remove("test.txt");
  myFile = SD.open("test.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
  //  Serial.print("Writing to test.txt...");
    
   // myFile.println("testing 1, 2, 3.");
  // close the file:
    myFile.close();
 //   Serial.println("done.");
  }
  
 
  //************************************ kalibrasyon
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  // print the calibration minimum values measured when emitters were on
 
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  
  hiz(0, 0);
}
void loop()
{
  
  unsigned int position = qtrrc.readLine(sensorValues);
  int error = position - 3500; // 3500 değeri, sensörlerin çizgiyi tamamen ortaladığında okunan position değeridir.
  
 // KP=error/dinamik; // 0.004<kp<0,035
  
/*  if(KP<0) // negatifse pozitif yap
  {
    KP=KP*(-1);
  }
  
  if(KP<0.004) // minimum kp değeri
  {
    KP=0.004;
  }*/
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
   
  gyroT = euler.x() - gyroF ;
  integral = integral + error;
  int turev = error - lastError; 
  int pozitif_error=error; // error mutlak değeri


  /* Display the floating point data */
  
  
 /* if(pozitif_error<0) // poizitif error değeri...
  {
    pozitif_error= error*(-1);
  }
  
  if(pozitif_error<1000) {
    KD=pozitif_error/4545; // 0.13<kd<0.22
    if(KD<0.13) {
      KD=0.13;
    }
  }
  else if (pozitif_error<2000)
  {
    KD=pozitif_error/6250; //0.22<kd<0.30
    if(KD<0.22) {
      KD=0.22;
    }
  }
  else if (pozitif_error<3500)
  {
    KD=pozitif_error/8333;  //0.30<kd<0.42
    if(KD<0.30) {
      KD=0.30;
    }
  } */
  
  int oran=KP * error;
  int td=KD * turev;
  float gyroY = gyroT * 3.5;
  int motorSpeed =  oran + td + (KI * integral) + gyroY;
  lastError = error;
  int leftMotorSpeed ;
   gyroF =euler.x();
  if(M1_minumum_speed - motorSpeed > 0)
  {
     leftMotorSpeed = M1_minumum_speed - motorSpeed;
  }
  else
  {
     leftMotorSpeed = 0;
  }
 

  int rightMotorSpeed ;
  if(  M2_minumum_speed + motorSpeed >0)
  {
   rightMotorSpeed = M2_minumum_speed + motorSpeed;
    
  }
  else
  {
    rightMotorSpeed = 0;
  }
   if(leftMotorSpeed > 190)
  {
    
     leftMotorSpeed = 190;
  }
  if(rightMotorSpeed > 190)
  {
    
     rightMotorSpeed = 190;
  }
  hiz(leftMotorSpeed, rightMotorSpeed);
  ileri(leftMotorSpeed, rightMotorSpeed);
  // 0 max. yansıma
  // 1000 min. yansıma
  //Serial.print('P');
  Serial.print(error);
  Serial.print('\t');
  
/*  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  { 
  //  Serial.print('s');
    Serial.print(sensorValues[i]);
      Serial.print('\t');
  }
  */
 // Serial.print();
  // kp, kd, ki, motorların min. hız ve maxi. hız verileri araba çizgiyi takip ederken değiştirilebiliyor.
  // data değişkeni, seri haberleşme ile hangi verinin değişeceğinin verisini tutar.

   myFile = SD.open("test.txt", FILE_WRITE);
  //kp, kd, ki, motorların min. ve max. değerlerinin kontrol satırı
  if (myFile) {
  myFile.print("X: ");
  myFile.print(euler.x());
   myFile.print('\t');
  myFile.print(KP,6);
  myFile.print('\t');
  myFile.print(KD,4);
  myFile.print('\t');
//  myFile.print(KI);
 // myFile.print('\t');
  myFile.print(position);
  myFile.print('\t');
  myFile.print(motorSpeed);
  myFile.print('\t');
  myFile.print(error);
  myFile.print('\t');
  myFile.print(oran);
  myFile.print('\t');
  myFile.print(turev);
  myFile.print('\t');
  myFile.print(td);
  myFile.print('\t');
  myFile.print(rightMotorSpeed);
  myFile.print('\t');
  myFile.println(leftMotorSpeed);
  myFile.close();
   } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
    
  }
  Serial.print("X: ");
  Serial.print(euler.x());
   Serial.println();
}
void saga_don()
{
  digitalWrite(motor_A_direction, HIGH); //sol motor
  digitalWrite(motor_B_direction, HIGH); //sag motor
 
 
  hiz(150, 100);
}
void sola_don()
{
  digitalWrite(motor_B_direction, HIGH); //sag motor
  digitalWrite(motor_A_direction, HIGH); //sol motor

  hiz(100, 150);
}
void ileri(int motor1speed, int motor2speed)
{
  digitalWrite(motor_A_direction, HIGH);
  digitalWrite(motor_B_direction, HIGH);

 
}

void hiz(int speed_a, int speed_b)
{
  analogWrite(motor_A_speed, speed_a);
  analogWrite(motor_B_speed, speed_b);
}
void geri()
{
  digitalWrite(motor_A_direction, LOW);
  
  digitalWrite(motor_B_direction, LOW);
 
}
