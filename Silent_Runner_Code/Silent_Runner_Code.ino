#include <Servo.h>
// download at https://github.com/GreyGnome/PinChangeInt/archive/master.zip
#include <PinChangeInt.h>

//defines the servo object
Servo ServoL;  //Servo fin stern left
Servo ServoR;  //Servo fin stern right
Servo ServoU;  //Servo fin stern up

#define CH2pin 4 //CH2 = Left/Right Channel
#define CH3pin 7 //CH3 = Up/Down Channel

//defines the servo pins, they must be PWM
#define ServoLpin 5 //Servo fin stern left
#define ServoRpin 6 //Servo fin stern right
#define ServoUpin 9 //Servo fin stern up



//variables for the maneuver calculations
int Rotation;
int Xaxis; 

// interrupt stuff
volatile uint8_t updateFlagsShared;

volatile uint16_t vRotation;
volatile uint16_t vXaxis;

uint32_t ulRotationStart;
uint32_t ulXaxisStart;

#define ROTATION_FLAG 1
#define XAXIS_FLAG 2
#define ROLL_FLAG 4

uint8_t updateChannel;



int ServoLout = 90; //Servo fin stern left
int ServoRout = 90; //Servo fin stern right
int ServoUout = 90; //Servo fin stern up

//max range of the servos
#define MIN_A 10
#define MAX_A 170
#define MAX_DEG 500  
#define MAX_DEG_SQUARE 250000
#define MIN_PWM 1500

// divisor for correcting rudder 
#define YCORRECTDIV 2

// stuff for value correction via averaging
#define NUMREADINGS 8 //chose high values for smoother but slower control

int readings_CH2[ NUMREADINGS ] ;
int readings_CH3[ NUMREADINGS ] ;
int index = 0;  

void setup()
{

  ServoL.attach(ServoLpin);
  ServoR.attach(ServoRpin);
  ServoU.attach(ServoUpin);

  pinMode(CH2pin, INPUT);
  pinMode(CH3pin, INPUT);

  //setting the smoothing arrays to zero
  for (byte thisReading = 0; thisReading < NUMREADINGS; thisReading++) {
    readings_CH2[thisReading] = 0.0;
    readings_CH3[thisReading] = 0.0;
  }

  PCintPort::attachInterrupt(CH2pin, calcRotation,CHANGE);
  PCintPort::attachInterrupt(CH3pin, calcXaxis,CHANGE);

} //setup end

void loop()
{
  static uint16_t vRotationIn;
  static uint16_t vXaxisIn;

  
  static uint8_t updateFlags;

  if(updateFlagsShared)
  {
    noInterrupts(); 

    
    updateFlags = updateFlagsShared;
    if(updateFlags & ROTATION_FLAG)
    {
      vRotationIn = vRotation;
    }

    if(updateFlags & XAXIS_FLAG)
    {
      vXaxisIn = vXaxis;
    }

    updateFlagsShared = 0;

    interrupts(); 
  }

  if(updateFlags & ROTATION_FLAG) {
    Rotation -= readings_CH2[index];
    int Rotation_tmp = -(vRotationIn-MIN_PWM); 
    Rotation += Rotation_tmp;

    readings_CH2[index] = Rotation_tmp;

    updateChannel |= ROTATION_FLAG;
  }

  if(updateFlags & XAXIS_FLAG) {
    Xaxis -= readings_CH3[index];
    int Xaxis_tmp = (vXaxisIn-MIN_PWM);
    Xaxis += Xaxis_tmp;

    readings_CH3[index] = Xaxis_tmp;

    updateChannel |= XAXIS_FLAG;
  }
  
  updateFlags = 0;

  if((updateChannel & ROTATION_FLAG) && (updateChannel & XAXIS_FLAG)) {
    
    updateChannel = 0;
    
    index = (index + 1) % NUMREADINGS;

    /* Control Matrix for inverted Y fins
     
     | Rotation  |  Xaxis  |
     ServoLeft   |    +      |    +    |
     ServoRight  |    +      |    -    |
     ServoUp     |    -      |   N/A   |     */

    ServoLout = ((br(Rotation) / YCORRECTDIV) + cubic(br(Xaxis))); //calculation of maneuvers
    ServoRout = ((br(Rotation) / YCORRECTDIV) - cubic(br(Xaxis)));
    ServoUout = (-br(Rotation));

    ServoL.write(forServo(ServoLout));
    ServoR.write(forServo(ServoRout));
    ServoU.write(forServo(ServoUout));
  }

} //loop end


// interrupt routines
void calcRotation() {
  if(digitalRead(CH2pin) == HIGH)
  { 
    ulRotationStart = micros();
  }
  else
  { 
    vRotation = (uint16_t)(micros() - ulRotationStart);
    updateFlagsShared |= ROTATION_FLAG;
  }
}

void calcXaxis() {
  if(digitalRead(CH3pin) == HIGH)
  { 
    ulXaxisStart = micros();
  }
  else
  { 
    vXaxis = (uint16_t)(micros() - ulXaxisStart);
    updateFlagsShared |= XAXIS_FLAG;
  }
}

// cubic function to fit into [MIN_DEG, MAX_DEG]
inline int cubic(double in) {
  return (in * in * in) / MAX_DEG_SQUARE;
}
// by readings 
inline int br(double in) {
  return in / NUMREADINGS;
}


inline long forServo(int s) {
  if (s >= MAX_DEG) {
    s = MAX_DEG;
  } //setÂ´s MAX_DEG as maximum for the servo movement
  if (s <= -MAX_DEG) {
    s = -MAX_DEG;
  } //setÂ´s -MAX_DEG as minimum for the servos
  return map(s,-MAX_DEG,MAX_DEG,MIN_A,MAX_A); //maps the values from -90 to 90 to the necessary output 10 to 170
}





