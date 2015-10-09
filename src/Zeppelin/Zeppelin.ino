#define SENS_L 0
#define SENS_R 1
#define SENS_F 2
#define SENS_D 3

#define PROP_BL 0 
#define PROP_BR 1
#define PROP_DO 2

// Animation constants
#define ANIM_ENABLED 0
#define ANIM_OBJECTIVE 1
#define ANIM_INITVAL 2
#define ANIM_CURRENTVAL 3
#define ANIM_TIMEINIT 4
#define ANIM_DURATION 5

void setup() {
  Serial.begin(9600);
  
  Serial.print("Initializing sensors... ");
  
  prepareSensor(SENS_L);
  prepareSensor(SENS_R);
  prepareSensor(SENS_F);
  prepareSensor(SENS_D);

  preparePropeller(PROP_BL);
  preparePropeller(PROP_BR);
  preparePropeller(PROP_DO);
  
  Serial.println("DONE");
}

void loop() {
  _propellerAnimationFrame(PROP_BL);
  _propellerAnimationFrame(PROP_BR);
  _propellerAnimationFrame(PROP_DO);

  /*extractAverage(SENS_F);
  extractAverage(SENS_L);
  extractAverage(SENS_R);
  extractAverage(SENS_D); 
  
  reportSensor(SENS_F);  
  reportSensor(SENS_L);  
  reportSensor(SENS_R);  
  reportSensor(SENS_D);
  
  Serial.println("-== END OF REPORT ==-");*/
  
  delay(1);
}

/*
 * -=============================-
 *  PROPELLER ENGINEERING STUFF 
 * -=============================-
 */

String prop_labels[3] = { "Back_Left", "Back_Right", "Down" };
int PROP[3] = { A0, A1, A2 };

void preparePropeller(int propeller) {
  pinMode(PROP[propeller], OUTPUT);
  digitalWrite(PROP[propeller], LOW);
}

void setPropellerSpeed(int propeller, int speed) {
  analogWrite(PROP[propeller], speed);
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
  setPropellerSpeed(propeller, 255);
}

/* PROPELLER ANIMATIONS */
/* This methods were created for smooth acceleration and breaking of the propellers, due to the Zeppeling long momentum. */

long PROPL_ANIMATIONS[3][6] = {
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 }
};

void resetPropellerAnimation(int propeller) {
  PROPL_ANIMATIONS[propeller][0] = 0;
  PROPL_ANIMATIONS[propeller][1] = 0;
  PROPL_ANIMATIONS[propeller][2] = 0;
  PROPL_ANIMATIONS[propeller][3] = 0;
  PROPL_ANIMATIONS[propeller][4] = 0;
  PROPL_ANIMATIONS[propeller][5] = 0;
}

void startPropellerAnimation(int propeller, int objective, int duration) {
  resetPropellerAnimation(propeller); // clear previous animation garbage
  
  PROPL_ANIMATIONS[propeller][ANIM_OBJECTIVE] = objective;
  PROPL_ANIMATIONS[propeller][ANIM_DURATION] = duration;
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

    Serial.print("Current Speed for Propeller: "); Serial.println(currentStage);
    Serial.println(PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL]);
  } else {
    Serial.println("No animation enabled for Propeller " + prop_labels[propeller]);
  }
}

/*
 * -=========================-
 *  ULTRASONIC SENSOR STUFF 
 * -=========================-
 */

int SENS[4][2] = { {2,3}, {4,5}, {6,7}, {8,9} }; // Sensors Ports
const int sens_rounds = 3; // rounds to consider a valid distance from sensors

String sens_labels[4] = { "Left", "Right", "Front", "Down" }; // human readable labels for sensors
int sensReadCount[4] = { 0, 0, 0, 0 }; // read count for each sensor
int currentAverages[4] = { -1, -1, -1, -1 }; // valid averages for zeppelin usage
int latestAverages[4] = { -1, -1, -1, -1 }; // current unofficial averages that will be put on valid averages after x rounds
int latestDistances[4] = { -1, -1, -1, -1 }; // latest distance gotten from sensors

void prepareSensor(int sensor) {
  pinMode(SENS[sensor][0], OUTPUT);
  pinMode(SENS[sensor][1], INPUT);
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
  long duration, distance;
  int sens_trigger, sens_echo;
  
  sens_trigger = SENS[sensor][0];
  sens_echo = SENS[sensor][1];
  
  digitalWrite(sens_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(sens_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(sens_trigger, LOW);
  
  duration = pulseIn(sens_echo, HIGH);
  distance = (duration / 2) / 29.1;
  
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
