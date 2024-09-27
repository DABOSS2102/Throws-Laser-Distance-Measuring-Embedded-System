#include<SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <LiquidCrystal_I2C.h> // Library for LCD

SoftwareSerial mySerial(16, 17);// RX, TX 
MPU6050 mpu(Wire); //gpio 32 to scl, gpio 33 to sda
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 16 column and 2 rows

//laser vars
byte auto_baud = 0x55;
byte continous_exit = 0x58;
byte rd_status[5] = {0xAA, 0x80, 0x00, 0x00, 0x80};
byte continous_auto[9]  = {0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25};
int  Data = 0;
byte Read[13] = {0};
unsigned int sigQ; // sinal quality
int offset;
int t = 15; // time between commands
int c ;  // counter

//vars
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 200;    // the debounce time; increase if the output flickers
unsigned long mpuTimer = 0; // Timer for Gyroscope
byte clickCount = 0; // Current state of process
float yaw = 0; pitch = 0; // yaw is horizontal and pitch is vertical 
float yawOfCenter, distanceToCenter = 0; // data for center of circle
float radius = 0, finalDistance = 0, laserLength = 0, // distances stored as mmm

//Interupt for button
void IRAM_ATTR buttonPressed(){
  if ((millis() - lastDebounceTime) > debounceDelay) {
    clickCount++; // Increment the count
    lastDebounceTime = millis(); // Save the time of the last press

    if(clickCount == 10){
      clickCount = 7;
    }
    //These are for testing
    //Serial.print("Clicked: ");
    //Serial.println(clickCount);
  }
}

void setup(){
  mySerial.begin(19200); // Start SoftwareSerial
  Serial.begin(19200); // Start Hardware Serial
  pinMode(23, INPUT_PULLUP); // Set the pin as input with internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(23), buttonPressed, RISING); // Attach the interrupt
  Wire.begin(33,32); // Starts I2C connection
  byte status = mpu.begin(); // Starts Gyroscope
  while(status != 0) {} // stop everything if could not connect to MPU6050
  delay(1000); // Waits for a second
  mpu.calcOffsets(); // Sets current position for MPU
  Serial.println("MPU ON"); // Hardware monitor says gyroscope on
  delay (1200); // Waits 1.2 seconds
  lcd.init(); // initialize the lcd
  lcd.backlight(); 
  Serial.println("LCD ON");
  mySerial.begin(19200);//Baudrate for comms with sensor
  mySerial.setTimeout(0.01);
  while (mySerial.read() == 0x00) {} // wait for 0x00
  Serial.println("Sensor ON");
  mySerial.write(0x55);//auto baudrate commas // working ok
  delay(1200);
  mySerial.write(0x58); // stop continuous
  delay((2t)+2100);
}

void loop(){
  mpu.update();
  laserControl();
  display();
  printDebug();
  delay(100);
}

void display(){
  switch(clickCounter){
      case 0: 
        controlDisplay("", "Devince On", "", "");
      break;
      case 1:
      controlDisplay("", "Set", "Center Point", "");
      break;
      case 2: case 5: case 8:
        lcd.setCursor(4, 1);
        lcd.print("Calculating");
      break;
      case 3:
        String tempString = "SQ: ";
        tempString += sigQ;
        controlDisplay("", "Center Point", "Set!", tempString.toCharArray());
      break;
      case 4:
        controlDisplay("", "Set", "Edge Point", "");
      break;
      case 6:
        String tempString = "SQ: ";
        tempString += sigQ;
        controlDisplay("", "Edge Point", "Set!", tempString.toCharArray());
      break;
      case 7:
        controlDisplay("", "Get", "Distance", "");
      break;
      case 9:
        String mDistanceString = String((actualDistance/1000),2);
        mDistanceString += "m";
        String iDistanceString = String((convertMMtoIN()/12),0);
        iDistanceString += "ft ";
        iDistanceString += String((convertMMtoIN()-int(round(convertMMtoIN()/12))),2);
        iDistanceString += "in";
        String tempString = "SQ: ";
        tempString += sigQ;
        controlDisplay("Distance", mDistanceString, iDistanceString, tempString);
      break;
      default:
      break;
    }
}

void controlDisplay(char[] l1, char[] l2, char[] l3, char[] l4){
  lcd.clear();
  displayLine(l1,0);
  displayLine(l2,1);
  displayLine(l3,2);
  displayLine(l4,3);
}

void displayLine(char[] s, byte row){
  //gets size of char array
  byte size = sizeof(s);

  //gets the size needed to text aligned center
  if(size%2==1){
    size = (size+1)/2;
  }
  else{
    size = size/2;
  }

  //checks to see if string too big and prints 
  if(size>9){
    lcd.setCursor((9-size),row);
    lcd.print(s);
  }
  else{
    lcd.setCursor(0,row);
    lcd.print("line error");
  } 
}

void laserControl(){
  //if the laser is supposed to be on
  if(clickCount != 0 && clickCount != 3 && clickCount != 6 && clickCount != 9){
    Data = 0;
    sigQ = 0;
    //turns on laser
    mySerial.write(continous_auto, 9);
    delay(t);
    //waits for datat to come back
    while (mySerial.available() > 0) {
      mySerial.readBytes(Read, 13);
    }
    //read if byte 3 is output
    if (Read[3] == 0x22) {
      Data = (Read[6] << 24) | (Read[7] << 16) | (Read[8] << 8) | Read[9];
      sigQ =  (Read[10] << 8) | (Read[11]);
    }

    //Do stuff if there is data and is processing data
    if(Data != 0 && (clickCount == 2 || clickCount == 5 || clickCount == 8){
      //get the angles
      getAngles();
      //get the x distance to point from laser
      calculateXDistanceFromLaserToMark();
      if(clickCount == 2){
        //setting center data
        //sets center horizontal position
        yawOfCenter = yaw;
        //sets x distance to center
        distanceToCenter = laserLength;
        clickCount++;

        //Debug
        //Serial.print("Center Point Yaw: ");
        //Serial.println(yawOfCenter);
        //Serial.print("Distance to Center: ");
        //Serial.println(distanceToCenter);
      }else if(clickCount == 5){
        //setting edge data
        radius = calculateDistanceFromCenterToPoint();
        clickCount++;

        //Debug
        //Serial.print("Radius: ");
        //Serial.println(radius);
      }else if(clickCount == 8){
        //data for mark
        finalDistance = calculateDistanceFromCenterToPoint() - radius;
        clickCount++;

        //Debug
        //Serial.print("Distance: ");
        //Serial.println(finalDistance);
      }

      //Debug
      //Serial.print("Click ");
      //Serial.print(clickCounter);
      //Serial.print(" data: ");
      //Serial.print(Data);
      //Serial.println(" mm");
      //checkStatus();
    }
  }
  else{
    //turns off laser if its currently on 
    if(laserOn){
      laserOn = false;
      mySerial.write(0x58); // stop continuous
    }
  }
}

// Sets angles
void getAngles(){
  //sets the yaw and pitch
  yaw = ((mpu.getAngleZ()* M_PI) / 180);
  pitch = ((abs(mpu.getAngleY())* M_PI) / 180);
  //Debug
  //Serial.println(yaw);
  //Serial.println(abs(mpu.getAngleZ()));
  //Serial.println(pitch);
 // Serial.println(abs(mpu.getAngleY()));
}

//gets x length and sets it to laser length
void calculateXDistanceFromLaserToMark(){
  //does the actual setting 
  laserLength = abs(Data*cos(pitch));
  if(pitch == 0){
    laserLength = Data;  
  }

  //Debug
  //Serial.print("Data: ");
  //Serial.println(Data);
  //Serial.print("pitch: ");
  //Serial.println(pitch);
  //Serial.print("X distance to Mark: ");
  //Serial.println(laserLength);
}

float calculateDistanceFromCenterToPoint(){
  //gets angle between points
  float tempYaw;
  if((yaw <= 0 && yawOfCenter >= 0)|| (yaw >= 0 && yawOfCenter <= 0)){
    tempYaw = abs(yaw) + abs(yawOfCenter);
  }
  else{
    if(yaw > yawOfCenter){
      tempYaw = yaw - yawOfCenter;
    }
    else{
      tempYaw = yawOfCenter - yaw;
    }
  }

  //Debug
  //Serial.print("tempYaw: ");
  //Serial.println(tempYaw);
  //Serial.print("distanceToCenter: ");
  //Serial.println(distanceToCenter);
  //Serial.print("Laser Length: ");
  //Serial.println(laserLength);

  //sends distance between points
  if(tempYaw == 0){
    float dtcTemp = abs(distanceToCenter);
    float llTemp = abs(laserLength);
    if(dtcTemp >= llTemp){
      return abs(dtcTemp-llTemp);
    }
    else{
      return abs(llTemp-dtcTemp);
    }
  }
  //math to find last side of a SAS triangle
  return (sqrt(((distanceToCenter*distanceToCenter)+(laserLength*laserLength))-((2*distanceToCenter*laserLength)*cos(tempYaw))));
}

void printDebugs(){
  Serial.print("Current Click Count: ");
  Serial.println(clickCount);
  Serial.print("Data: ");
  Serial.print(Data);
  Serial.print(" Signal Quality: ");
  Serial.println(sigQ);
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" laserLength: ");
  Serial.println(laserLength);
  Serial.print("Yaw of Center: ");
  Serial.print(yawOfCenter);
  Serial.print(" Distance to Center: ");
  Serial.println(distanceToCenter);
  Serial.print("Radius: ");
  Serial.println(radius);
  Serial.print("Final Distance: ");
  Serial.println(finalDistance);
}