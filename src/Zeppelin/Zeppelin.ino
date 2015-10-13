#define SENS_L 0
#define SENS_R 1
#define SENS_F 2
#define SENS_D 3

#define PROP_BL 0 
#define PROP_BR 1
#define PROP_DO 2

#define VIB_MOT 3 // PORT

// Animation constants
#define ANIM_ENABLED 0
#define ANIM_OBJECTIVE 1
#define ANIM_INITVAL 2
#define ANIM_CURRENTVAL 3
#define ANIM_TIMEINIT 4
#define ANIM_DURATION 5

#define DEBUG_ANIM 0

#define DESIRED_ALTITUDE 100
#define DESIRED_SIDES_DISTANCE 60

#define MINIMUM_SPEED 40
#define MAXIMUM_SPEED 255

long latestBrainTick = -1;
byte brainTickTime = 50;

long latestAnimTick = -1;
byte animTickTime = 10;

void setup() {
  Serial.begin(9600);
  
  Serial.print("Initializing sensors... ");
  
  setupSensors();
  
  pinMode(VIB_MOT, OUTPUT);
  
  setupPropellers();

  vibrate(250);
  delay(50);
  vibrate(100);
  
  Serial.println("DONE");
  //delay(500000);
}

void loop() {
  /* the animation tick */
  /* the animation needs to be run more often. so the tick time is every 10ms (default) */
  if( ((millis() - latestAnimTick) > animTickTime) || latestAnimTick == -1 ) {
    _propellerAnimationFrame(PROP_BL);
    _propellerAnimationFrame(PROP_BR);
    _propellerAnimationFrame(PROP_DO);
    
    latestAnimTick = millis();
  }

  /* the brain tick */
  /* we do not need to run the Zeppelin's logic loop every single clock, so we make a tick time of 50ms (default) */
  if( ((millis() - latestBrainTick) > brainTickTime) || latestBrainTick == -1 ) {
    updateSensors();

    /* altitude logic */
    byte upPropellerSpeed = MINIMUM_SPEED; // by default, the propeller is 'off'
    byte currentAltitude = getSensorDistance(SENS_D); // then we collect the current balloon altitude
    byte distance = (DESIRED_ALTITUDE - currentAltitude); // get the distance (in cm) from the desired position

    /* the balloon is lower than desired, need to pull it up */
    if(distance > 0) {
      byte speedVariation = MAXIMUM_SPEED - MINIMUM_SPEED;
      float speedIncreaseByCM = (float) speedVariation / (float) DESIRED_ALTITUDE;
      upPropellerSpeed = MINIMUM_SPEED + (speedIncreaseByCM * distance);
    } else {
      /* the balloon tends to fall, so no need to reverse this propeller */
    }

    /* left/right logic */
    byte leftPropellerSpeed = MINIMUM_SPEED;
    byte rightPropellerSpeed = MINIMUM_SPEED;
    byte currentLeftDistance = getSensorDistance(SENS_L);
    byte currentRightDistance = getSensorDistance(SENS_R);

    byte currentLeftSpeed = getPropellerSpeed(PROP_BL);
    byte currentRightSpeed = getPropellerSpeed(PROP_BR);

    if( currentLeftDistance < DESIRED_SIDES_DISTANCE ) {
      byte leftVariation = DESIRED_SIDES_DISTANCE - currentLeftDistance;

      
    }


    /* fire animations */
    startPropellerAnimation(PROP_BL, leftPropellerSpeed, 100);
    startPropellerAnimation(PROP_BR, rightPropellerSpeed, 100);
    startPropellerAnimation(PROP_DO, upPropellerSpeed, 100);
    
    /*
    reportSensor(SENS_F);
    reportSensor(SENS_L);
    reportSensor(SENS_R);
    reportSensor(SENS_D);
    
    Serial.println("-== END OF REPORT ==-");
    */
    
    latestBrainTick = millis();
  }
  
  delay(1);
}

/*
 * -=============================-
 *  VIBRATION ENGINEERING STUFF 
 * -=============================-
 */
void vibrate(int duration) {
  digitalWrite(VIB_MOT, HIGH);
  delay(duration);
  digitalWrite(VIB_MOT, LOW);
}

/*
 * -=============================-
 *  PROPELLER ENGINEERING STUFF 
 * -=============================-
 */

String prop_labels[3] = { "Back_Left", "Back_Right", "Down" };
int PROP[3] = { 9, 10, 11 };

long PROPL_ANIMATIONS[3][6] = {
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 }
};

void preparePropeller(int propeller) {
  pinMode(PROP[propeller], OUTPUT);
  digitalWrite(PROP[propeller], LOW);
}

void setupPropellers() {
  preparePropeller(PROP_BL);
  preparePropeller(PROP_BR);
  preparePropeller(PROP_DO);
  
  setPropellerSpeed(PROP_BL, 65);
  setPropellerSpeed(PROP_BR, 65);
  setPropellerSpeed(PROP_DO, 65);
  
  delay(300);
  
  startPropellerAnimation(PROP_BL, 255, 18000);
  startPropellerAnimation(PROP_BR, 255, 8000);
  startPropellerAnimation(PROP_DO, 255, 3000);
}

void setPropellerSpeed(int propeller, int speed) {
  PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] = speed;
  analogWrite(PROP[propeller], speed);
}

byte getPropellerSpeed(int propeller) {
  return (byte) PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL];
}

void stopPropeller(int propeller, int duration) {
  resetPropellerAnimation(propeller);
  
  if( duration == 0 ) {
    setPropellerSpeed(propeller, 0);
  } else {
    startPropellerAnimation(propeller, 0, duration);
  }
}

void fullSpeedPropeller(int propeller, int duration) {
  PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] = 255;
  setPropellerSpeed(propeller, 255);
}

/* PROPELLER ANIMATIONS */
/* This methods were created for smooth acceleration and breaking of the propellers, due to the Zeppeling long momentum. */

void resetPropellerAnimation(int propeller) {
  PROPL_ANIMATIONS[propeller][0] = 0;
  PROPL_ANIMATIONS[propeller][1] = 0;
  PROPL_ANIMATIONS[propeller][2] = 0;
  //PROPL_ANIMATIONS[propeller][3] = 0;
  PROPL_ANIMATIONS[propeller][4] = 0;
  PROPL_ANIMATIONS[propeller][5] = 0;
}

void startPropellerAnimation(int propeller, int objective, int duration) {
  resetPropellerAnimation(propeller); // clear previous animation garbage
  
  PROPL_ANIMATIONS[propeller][ANIM_OBJECTIVE] = objective;
  PROPL_ANIMATIONS[propeller][ANIM_DURATION] = duration;
  PROPL_ANIMATIONS[propeller][ANIM_TIMEINIT] = millis();
  PROPL_ANIMATIONS[propeller][ANIM_INITVAL] = PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL];
  PROPL_ANIMATIONS[propeller][ANIM_ENABLED] = 1;
}

void _propellerAnimationFrame(int propeller) {
  if(PROPL_ANIMATIONS[propeller][ANIM_ENABLED]) {
    float framesPerMs;
    
    long currentTime = millis();

    long initVal   = PROPL_ANIMATIONS[propeller][ANIM_INITVAL];
    long objective = PROPL_ANIMATIONS[propeller][ANIM_OBJECTIVE];
    
    long diffTime  = currentTime - PROPL_ANIMATIONS[propeller][ANIM_TIMEINIT];
    int diffFrames = objective - initVal;
    
    long animDuration = PROPL_ANIMATIONS[propeller][ANIM_DURATION];
    
    if( animDuration == 0 ) {
      framesPerMs = (float) diffFrames; // immediate action
    } else {
      framesPerMs = (float) diffFrames / PROPL_ANIMATIONS[propeller][ANIM_DURATION]; // calculate frames per ms
    }
    
    float currentStage = framesPerMs * (float) diffTime;
    
    PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] = floor(initVal + currentStage);
  
    if( (initVal < objective && PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] >= objective)
        || (initVal > objective && PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] <= objective) ) {
      PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] = objective;
      PROPL_ANIMATIONS[propeller][ANIM_ENABLED] = 0;
    }
    
    analogWrite(PROP[propeller], PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL]);

    if ( DEBUG_ANIM ) Serial.print("Current Speed Animation Stage for Propeller: "); if ( DEBUG_ANIM ) Serial.println(currentStage);
    if ( DEBUG_ANIM ) Serial.print("Current Speed for Propeller: "); if ( DEBUG_ANIM ) Serial.println(PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL]);
  } else {
    if ( DEBUG_ANIM ) Serial.println("No animation enabled for Propeller " + prop_labels[propeller]);
  }
}

/*
 * -=========================-
 *  ULTRASONIC SENSOR STUFF 
 * -=========================-
 */

int SENS[4][2] = { {A0, A1}, {4,5}, {6,7}, {8,9} }; // Sensors Ports
const int sens_rounds = 3; // rounds to consider a valid distance from sensors

String sens_labels[4] = { "Left", "Right", "Front", "Down" }; // human readable labels for sensors
int sensReadCount[4] = { 0, 0, 0, 0 }; // read count for each sensor
int currentAverages[4] = { -1, -1, -1, -1 }; // valid averages for zeppelin usage
int latestAverages[4] = { -1, -1, -1, -1 }; // current unofficial averages that will be put on valid averages after x rounds
int latestDistances[4] = { -1, -1, -1, -1 }; // latest distance gotten from sensors

void setupSensors() {
  prepareSensor(SENS_L);
  prepareSensor(SENS_R);
  prepareSensor(SENS_F);
  prepareSensor(SENS_D);
}

void prepareSensor(int sensor) {
  pinMode(SENS[sensor][0], OUTPUT);
  pinMode(SENS[sensor][1], INPUT);
}

byte getSensorDistance(int sensor) {
  return (byte) currentAverages[sensor];
}

void updateSensors() {
  extractAverage(SENS_F);
  extractAverage(SENS_L);
  extractAverage(SENS_R);
  extractAverage(SENS_D);
}

long extractAverage(int sensor) {
  long distance = readSensorDistance(sensor);
  
  if( latestAverages[sensor] == -1 ) {
    latestAverages[sensor] = distance;
  } else {
    latestAverages[sensor] = (latestAverages[sensor] + distance) / 2;
  }
   
  if( ++sensReadCount[sensor] >= sens_rounds ) {
    currentAverages[sensor] = latestAverages[sensor];
    sensReadCount[sensor] = 0;
    latestAverages[sensor] = -1;
  }
}

long readSensorDistance(int sensor) { 
  int sens_trigger = SENS[sensor][0];
  int sens_echo = SENS[sensor][1];
  
  digitalWrite(sens_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(sens_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(sens_trigger, LOW);
  
  long duration = pulseIn(sens_echo, HIGH);
  long distance = (duration / 2) / 29.1;
  
  latestDistances[sensor] = distance;
  
  return distance;
}

void reportSensor(int sensor) {
  byte logValid = 1,
       logUnAvg = 1,
       logLatest = 1;
  
  if( logValid ) {
    Serial.print("Valid average for SENS_"); Serial.print(sens_labels[sensor]); Serial.print(": ");
    Serial.print(currentAverages[sensor]);
    Serial.println(" cm");
  }
  
  if( logUnAvg ) {
    Serial.print("Unofficial average for SENS_"); Serial.print(sens_labels[sensor]); Serial.print(": ");
    Serial.print(latestAverages[sensor]);
    Serial.println(" cm");
  }
  
  if( logLatest ) {
    Serial.print("Latest distance gotten from SENS_"); Serial.print(sens_labels[sensor]); Serial.print(": ");
    Serial.print(latestDistances[sensor]);
    Serial.println(" cm");
  }
  
  if( logValid || logUnAvg || logLatest ) {
    Serial.println("------------");
  }
}
