#include <SimpleDHT.h>
#include <LiquidCrystal.h>
int fan=4;
int pinDHT11 = A0;
SimpleDHT11 dht11(pinDHT11);
int smokeA0 = A1;
int sensorThres = 300;//smoke
int sensorValue;
int thresholdValue=400;//gas
int acetoneTHreshold=600;
int humidityTHreshold=80;
int R0=176;
int R2=1000;
float RS;
float PPM_acetone;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("temperature and humidity data");
  pinMode(smokeA0, INPUT);
  pinMode(fan,OUTPUT);
  digitalWrite(fan,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(300);
  // start working...
  Serial.println("=================================");
  Serial.println("Sample DHT11...");
  
  // read without samples.
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT11 failed, err="); Serial.print(SimpleDHTErrCode(err));
    Serial.print(","); Serial.println(SimpleDHTErrDuration(err)); delay(1000);
    return;
  }
  //Serial.print("Sample OK: ");
  Serial.print((int)temperature); Serial.print("*C, "); 
  Serial.print((int)humidity); Serial.println(" H");
  
  // DHT11 sampling rate is 1HZ.
  //delay(300);

  int analogSensor = analogRead(smokeA0);
 Serial.println("MQ2...");
  Serial.print("Smoke value: ");
  Serial.println(analogSensor);
  // Checks if it has reached the threshold value
 
  if (analogSensor > sensorThres)
  {
    /*digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    tone(buzzer, 1000, 200);*/
    Serial.print("smoke detected!!\n");
  }
  else
  {
    /*digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
    noTone(buzzer);*/
    Serial.print("normal\n");
  }

  //delay(300);
 Serial.println("MQ135...");
sensorValue = analogRead(A2);       // read analog input pin 0
float volts=sensorValue*5;
volts/=1023;
RS=R2*(1-volts);
RS=RS/volts;

PPM_acetone=159.6-133*(RS/R0);
Serial.print("Air Quality = ");
Serial.print(sensorValue, DEC);               // prints the value read
Serial.println(" PPM CO2");
Serial.print(PPM_acetone);
Serial.println(" PPM ACETONE");


if(sensorValue>thresholdValue)
{
  Serial.println("Polluted air");
}
else{
  Serial.println("Normal air");
}


if((int)temperature>=32||analogSensor > sensorThres||(sensorValue>thresholdValue)||PPM_acetone>acetoneTHreshold||(int)humidity>humidityTHreshold){
    //Serial.println(temperature);
    digitalWrite(fan,LOW);
}
else digitalWrite(fan,HIGH);

delay(1000*3);          

while(Serial.available()>0){
  //inputByte= Serial.read();
  //Serial.println(inputByte);
  /*if (inputByte=='Z'){
  Serial.println("H");
  }
  else if (inputByte=='z'){
  Serial.println("L");
  } */

  }

}
