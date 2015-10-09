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

int SENS[4][2] = { {2,3}, {4,5}, {6,7}, {8,9} };
int PROP[3] = { A0, A1, A2 };

const int rounds = 3; // rounds to consider a valid distance from sensors

char labels[4] = { 'L', 'R', 'F', 'D' };
int currentAverages[4] = { -1, -1, -1, -1 }; // valid averages for zeppelin usage
int latestAverages[4] = { -1, -1, -1, -1 }; // current unofficial averages that will be put on valid averages after x rounds
int latestDistances[4] = { -1, -1, -1, -1 }; // latest distance gotten from sensors

int readCount[4] = { 0, 0, 0, 0 };

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

void prepareSensor(int sensor) {
  pinMode(SENS[sensor][0], OUTPUT);
  pinMode(SENS[sensor][1], INPUT);
}

void preparePropeller(int propeller) {
  pinMode(PROP[propeller], OUTPUT);
  digitalWrite(PROP[propeller], LOW);
}

void setPropellerSpeed(int propeller, int speed) {
  analogWrite(PROP[propeller], speed);
}

void stopPropeller(int propeller) {
  setPropellerSpeed(propeller, 0);
}

void fullSpeedPropeller(int propeller) {
  setPropellerSpeed(propeller, 255);
}

void reportSensor(int sensor) {
  int logValid = 1;
  int logUnAvg = 1;
  int logLatest = 1;
  
  if( logValid ) {
    Serial.print("Valid average for SENS_"); Serial.print(labels[sensor]); Serial.print(": ");
    Serial.print(currentAverages[sensor]);
    Serial.println(" cm");
  }
  
  if( logUnAvg ) {
    Serial.print("Unofficial average for SENS_"); Serial.print(labels[sensor]); Serial.print(": ");
    Serial.print(latestAverages[sensor]);
    Serial.println(" cm");
  }
  
  if( logLatest ) {
    Serial.print("Latest distance gotten from SENS_"); Serial.print(labels[sensor]); Serial.print(": ");
    Serial.print(latestDistances[sensor]);
    Serial.println(" cm");
  }
  
  if( logValid || logUnAvg || logLatest ) {
    Serial.println("------------");
  }
}

long PROPL_ANIMATIONS[3][6] = {
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0 }
};

void _propellerAnimationFrame(int propeller) {
  if(PROPL_ANIMATIONS[propeller][ANIM_ENABLED]) {
    long currentTime = millis();

    long initVal = PROPL_ANIMATIONS[propeller][ANIM_INITVAL];
    long objective = PROPL_ANIMATIONS[propeller][ANIM_OBJECTIVE];
    
    long diffTime  = currentTime - PROPL_ANIMATIONS[propeller][ANIM_TIMEINIT];
    int diffFrames = objective - initVal;
    
    float framesPerMs = (float) diffFrames / PROPL_ANIMATIONS[propeller][ANIM_DURATION];
    float shouldvePassed = framesPerMs * (float) diffTime;
    
    PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] = floor(initVal + shouldvePassed);
  
    if((initVal < objective && PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] >= objective)
        || (initVal > objective && PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] <= objective)) {
      PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL] = objective;
      PROPL_ANIMATIONS[propeller][ANIM_ENABLED] = 0;
    }

    Serial.print("frames/ms: "); Serial.println(shouldvePassed);
    Serial.println(PROPL_ANIMATIONS[propeller][ANIM_CURRENTVAL]);
  } else {
    Serial.println("No animation enabled for Propeller " + (propeller+1));
  }
}

void loop() {
  _propellerAnimationFrame(PROP_BL);
  

  
  /*
   * 

Tenho um objetivo de numero. A cada loop eu vejo o quanto foi a velocidade inicial, quanto tempo se passou e quanto falta p o objetivo.
Tiro a diferenca entre esses dois valores, pego o quanto deveria estar apos a qtd de tempo q se passou e seto a velocidade

   */
  /*extractAverage(SENS_F);
  extractAverage(SENS_L);
  extractAverage(SENS_R);
  extractAverage(SENS_D); 
  
  reportSensor(SENS_F);  
  reportSensor(SENS_L);  
  reportSensor(SENS_R);  
  reportSensor(SENS_D);
  
  Serial.println("-== END OF REPORT ==-");*/
  
  delay(90);
}

long extractAverage(int sensor) {
  long distance = readSensorDistance(sensor);
  
  if( latestAverages[sensor] == -1 ) {
    latestAverages[sensor] = distance;
  } else {
    latestAverages[sensor] = (latestAverages[sensor] + distance) / 2;
  }
   
  if( ++readCount[sensor] >= rounds ) {
    currentAverages[sensor] = latestAverages[sensor];
    readCount[sensor] = 0;
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
