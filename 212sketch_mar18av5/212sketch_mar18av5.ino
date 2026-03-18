// ------------------------------------------------------------
// Engine Tachometer + PWM Control
// Arduino Uno / Nano
//
// Pin 9  -> 25 Hz PWM output
// A1     -> duty cycle potentiometer
// A0     -> used for MAP sensor
// Pin 3  -> tach input (1 pulse per revolution)
// PIN 4  -> used for RUN switch
// Pin 5  -> used for PRIME button
// 
// Frequency range: 0–70 Hz
// RPM range: 0–4200
// Comments marked with " //x " are special and cannot be modified by Chat GPT
//3/18/26
// ------------------------------------------------------------

const byte pwmPin = 9;
const byte potPin = A1;
const byte freqPin = 3;

const uint16_t pwmFrequency = 25;

// ----- pulse timing -----
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulsePeriod = 0;
volatile bool newPulse = false;

// ----- moving average filter -----
const int AVG_COUNT = 8; //  was 4
unsigned long periodBuffer[AVG_COUNT];
int periodIndex = 0;
bool bufferFilled = false;

// ----- measurement -----
float measuredFrequency = 0.0;
float rpm = 0.0;
//x------------------------------------my stuff---------------

int lowestReading = 350;  //x Brute force MAP
int currentReading = 0;  //x  Brute force MAP
int debugCounter = 0;//x
int VAC = A0;    //x location of MAP sensor (analog)
int button = 5;  //x location of prime button (digital)
int RUN_NOW = 4; //x location of RUN switch  (digital)
int RUN = 1;     //x set default value of RUN
int PRIME = 1;   //x set default value of PRIME
int MAP = 0;     //x vacuum
int x = 0;       //x counter variable

// ----- fuel table axes -----
const int RPM_BIN_COUNT = 6;
const int MAP_BIN_COUNT = 5;

const int rpmBins[RPM_BIN_COUNT] = {1500, 2000, 2500, 3000, 3500, 4000};
const int mapBins[MAP_BIN_COUNT] = {50, 80, 100, 130, 160};

// Rows = RPM bins
// Cols = MAP bins
const uint16_t fuelTable[RPM_BIN_COUNT][MAP_BIN_COUNT] = {
  // MAP →   50    80    100   130   160
  /*1500*/ {377,  428,  430,  600,  900},
  /*2000*/ {547,  650,  752, 1100, 1800},
  /*2500*/ {606,  967, 1451, 1600, 1749},
  /*3000*/ {700, 1100, 1600, 2000, 2300},
  /*3500*/ {900, 1400, 1900, 2300, 2482},
  /*4000*/ {1100,1600, 2000, 2140, 2200}
};

// ----- adaptive MAP sampling -----
int mapSampleTarget = 1200;
const int MAP_SAMPLES_LOW_RPM = 3500;
const int MAP_SAMPLES_HIGH_RPM = 500;
const int MAP_RPM_SWITCH_LOW = 1700;
const int MAP_RPM_SWITCH_HIGH = 1900;
bool fastMapMode = false;

// ----- automatic RUN enable -----
const int RUN_ENABLE_RPM = 600;
const int RUN_DISABLE_RPM = 400;
bool runLatched = false;

// ------------------------------------------------------------
// Helper function: find nearest table index
// ------------------------------------------------------------
int findClosestIndex(int value, const int *array, int size)
{
  int closestIndex = 0;
  int smallestDiff = abs(value - array[0]);

  for (int i = 1; i < size; i++)
  {
    int diff = abs(value - array[i]);
    if (diff < smallestDiff)
    {
      smallestDiff = diff;
      closestIndex = i;
    }
  }

  return closestIndex;
}

// ------------------------------------------------------------
// Helper function: find lower index for interpolation
// Returns the lower bin index such that:
// array[index] <= value <= array[index + 1]
// Clamps to table edges
// ------------------------------------------------------------
int findLowerIndex(int value, const int *array, int size)
{
  if (value <= array[0])
  {
    return 0;
  }

  if (value >= array[size - 1])
  {
    return size - 2;
  }

  for (int i = 0; i < size - 1; i++)
  {
    if (value >= array[i] && value <= array[i + 1])
    {
      return i;
    }
  }

  return size - 2;
}

// ------------------------------------------------------------
// Helper function: linear interpolation
// ------------------------------------------------------------
float interpolate(float x, float x0, float x1, float y0, float y1)
{
  if (x1 == x0)
  {
    return y0;
  }

  return y0 + ((x - x0) * (y1 - y0) / (x1 - x0));
}

// ------------------------------------------------------------
// Helper function: bilinear interpolation for fuel table
// ------------------------------------------------------------
float getInterpolatedDuty(float rpmValue, int mapValue)
{
  int rpmLowIndex = findLowerIndex((int)rpmValue, rpmBins, RPM_BIN_COUNT);
  int rpmHighIndex = rpmLowIndex + 1;

  int mapLowIndex = findLowerIndex(mapValue, mapBins, MAP_BIN_COUNT);
  int mapHighIndex = mapLowIndex + 1;

  float rpmLow = rpmBins[rpmLowIndex];
  float rpmHigh = rpmBins[rpmHighIndex];
  float mapLow = mapBins[mapLowIndex];
  float mapHigh = mapBins[mapHighIndex];

  float q11 = fuelTable[rpmLowIndex][mapLowIndex];
  float q21 = fuelTable[rpmHighIndex][mapLowIndex];
  float q12 = fuelTable[rpmLowIndex][mapHighIndex];
  float q22 = fuelTable[rpmHighIndex][mapHighIndex];

  float interpAtMapLow = interpolate(rpmValue, rpmLow, rpmHigh, q11, q21);
  float interpAtMapHigh = interpolate(rpmValue, rpmLow, rpmHigh, q12, q22);

  return interpolate((float)mapValue, mapLow, mapHigh, interpAtMapLow, interpAtMapHigh);
}

// ------------------------------------------------------------
// Interrupt Service Routine
// ------------------------------------------------------------
void pulseISR()
{
  unsigned long now = micros();
  unsigned long dt = now - lastPulseTime;

  // Reject impossible pulses (< 2 ms apart)
  if (dt > 4000)  //  range 2000 to 6000. higer for more filtering.  was 2000
  {
    pulsePeriod = dt;
    lastPulseTime = now;
    newPulse = true;
  }
}


// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup()
{
  pinMode(pwmPin, OUTPUT);
  pinMode(freqPin, INPUT);
  //----------------my stuff------------
  pinMode(button, INPUT_PULLUP);//x
  pinMode(RUN_NOW, INPUT_PULLUP);//x
  //--------------------------------------

  Serial.begin(115200);

  // ----- TIMER1 PWM SETUP (Pin 9) -----
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Fast PWM, TOP = ICR1
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10); // prescaler 64

  ICR1 = (16000000UL / (64UL * pwmFrequency)) - 1;

  OCR1A = 0;

  // ----- interrupt setup -----
  attachInterrupt(digitalPinToInterrupt(freqPin), pulseISR, RISING);
}


// ------------------------------------------------------------
// Main Loop
// ------------------------------------------------------------
void loop()
{
  // ----- adaptive MAP sample window -----
  if (fastMapMode)
  {
    if (rpm < MAP_RPM_SWITCH_LOW)
    {
      fastMapMode = false;
    }
  }
  else
  {
    if (rpm > MAP_RPM_SWITCH_HIGH)
    {
      fastMapMode = true;
    }
  }

  if (fastMapMode)
  {
    mapSampleTarget = MAP_SAMPLES_HIGH_RPM;
  }
  else
  {
    mapSampleTarget = MAP_SAMPLES_LOW_RPM;
  }

  //x --------------------brute force MAP data-------------------
  currentReading = analogRead(VAC);           //x
  if (currentReading < lowestReading) {       //x
    lowestReading = currentReading;           //x
  }                                           //x
  x++;                                        //x

  if (x >= mapSampleTarget)
  {
    x = 1200;
  }

  if (x >= 1200)  // Sample count. Was 3000   //x
  { //x
    MAP = lowestReading;                      //x
    lowestReading = 350;                      //x
    x = 0;                                    //x
  }                                           //x
  //------------------------------------------------------------


  // ----- PWM duty control ------------------------
  int potValue = analogRead(potPin);

  int trim = map(potValue, 0, 1023, -35, 35);

  int rpmIndex = findClosestIndex((int)rpm, rpmBins, RPM_BIN_COUNT);
  int mapIndex = findClosestIndex(MAP, mapBins, MAP_BIN_COUNT);

  int selectedRpmBin = rpmBins[rpmIndex];
  int selectedMapBin = mapBins[mapIndex];

  int rpmLowIndex = findLowerIndex((int)rpm, rpmBins, RPM_BIN_COUNT);
  int rpmHighIndex = rpmLowIndex + 1;
  int mapLowIndex = findLowerIndex(MAP, mapBins, MAP_BIN_COUNT);
  int mapHighIndex = mapLowIndex + 1;

  int rpmLowBin = rpmBins[rpmLowIndex];
  int rpmHighBin = rpmBins[rpmHighIndex];
  int mapLowBin = mapBins[mapLowIndex];
  int mapHighBin = mapBins[mapHighIndex];

  float interpolatedDutyFloat = getInterpolatedDuty(rpm, MAP);
  int tableDuty = (int)(interpolatedDutyFloat + 0.5);

  int dutyInt = tableDuty + trim;

  if (dutyInt < 0)
  {
    dutyInt = 0;
  }

  if (dutyInt > ICR1)
  {
    dutyInt = ICR1;
  }

  uint16_t duty = (uint16_t)dutyInt;
  //OCR1A = duty; //x  moved to RUN section.
  //-----------------------------------------------

  //x-------command switch for EFI to start working----------         //x
  RUN = digitalRead(RUN_NOW);                                         //x

  if (RUN != 0)
  {
    runLatched = false;
  }
  else
  {
    if (runLatched)
    {
      if (rpm < RUN_DISABLE_RPM)
      {
        runLatched = false;
      }
    }
    else
    {
      if (rpm > RUN_ENABLE_RPM)
      {
        runLatched = true;
      }
    }
  }

  if (runLatched)
  {
    RUN = 0;
  }
  else
  {
    RUN = 1;
  }

  //x
  if (RUN == 0) {                                                     //x
    OCR1A = duty;   // put duty cycle on digital pin 9 (injector)     //x
  } else {                                                            //x
    OCR1A = 0;      // put ZERO duty cycle on digital pin 9(injector) //x
  }                                                                   //x

  //x -----------------command button to prime engine------------------//x
  PRIME = digitalRead(button);  //x use injector to prime engine with gasoline
  if (PRIME == 0) {                 //x
    for (int i = 0; i < 30000; i++) //x
    { //x
      OCR1A = 5000; //x meh 50% duty cycle//x
    }//x
    OCR1A = 0;//x
  }//x
  //x--------------------------------------------------------------------




  // ----- safely copy interrupt data -----
  unsigned long localPeriod;
  unsigned long localLastPulse;
  bool localNewPulse;

  noInterrupts();
  localPeriod = pulsePeriod;
  localLastPulse = lastPulseTime;
  localNewPulse = newPulse;
  if (newPulse) newPulse = false;
  interrupts();


  // ----- new pulse received -----
  if (localNewPulse && localPeriod > 0)
  {
    periodBuffer[periodIndex] = localPeriod;
    periodIndex++;

    if (periodIndex >= AVG_COUNT)
    {
      periodIndex = 0;
      bufferFilled = true;
    }

    int count = bufferFilled ? AVG_COUNT : periodIndex;

    unsigned long sum = 0;

    for (int i = 0; i < count; i++)
    {
      sum += periodBuffer[i];
    }

    float avgPeriod = (float)sum / count;

    measuredFrequency = 1000000.0 / avgPeriod;
  }


  // ----- zero detection -----
  if (micros() - localLastPulse > 2000000UL)
  {
    measuredFrequency = 0.0;
  }


  // ----- convert to RPM -----
  rpm = measuredFrequency * 60.0;




  // ---- serial output for DEBUG -----

  debugCounter ++; //x
  if (debugCounter > 2000) //x updates data ever 2000 loops of the program
  {//x
    debugCounter = 0;//x

    Serial.print("RPM: ");//x
    Serial.println((int)rpm);//x
    Serial.print("MAP: ");//x
    Serial.println(MAP);//x
    Serial.print("Duty Cycle: ");//x
    Serial.println(OCR1A);//x

    Serial.print("RPM Index: ");
    Serial.println(rpmIndex);
    Serial.print("MAP Index: ");
    Serial.println(mapIndex);
    Serial.print("Selected RPM Bin: ");
    Serial.println(selectedRpmBin);
    Serial.print("Selected MAP Bin: ");
    Serial.println(selectedMapBin);

    Serial.print("RPM Low Bin: ");
    Serial.println(rpmLowBin);
    Serial.print("RPM High Bin: ");
    Serial.println(rpmHighBin);
    Serial.print("MAP Low Bin: ");
    Serial.println(mapLowBin);
    Serial.print("MAP High Bin: ");
    Serial.println(mapHighBin);

    Serial.print("MAP Sample Target: ");
    Serial.println(mapSampleTarget);
    Serial.print("Fast MAP Mode: ");
    Serial.println(fastMapMode);
    Serial.print("Trim: ");
    Serial.println(trim);
    Serial.print("Interpolated Table Duty: ");
    Serial.println(tableDuty);
    Serial.print("Final Duty: ");
    Serial.println(duty);
    Serial.print("RUN Latched: ");
    Serial.println(runLatched);

        
    // }//x
  }//x
}
