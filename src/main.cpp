#include <Arduino.h>
#include <arduinoFFT.h>
#include <math.h>
#include <notes.h>
#include <audioData.h>
#include <Yin.h>

#define mutePin1 PB12 // 1
#define inputA PB14   // 7
#define inputB PB15   // 11

#define SWITCH_PIN_1 PA3 // 9
#define SWITCH_PIN_2 PA4 // 10
#define MUTE_LED PC13    // 8
#define INPUT_A_LED PC14
#define INPUT_B_LED PC15

#define TUNER_IN PA0

#define VERY_LOW PB3
#define TOO_LOW PB4
#define PERFECT PB5
#define TOO_HIGH PB6
#define VERY_HIGH PB7

#define STRING_E1 A1
#define STRING_B A1
#define STRING_G A1
#define STRING_D A1
#define STRING_A A1
#define STRING_E2 A1

int tuningLeds[5] = {VERY_LOW, TOO_LOW, PERFECT, TOO_HIGH, VERY_HIGH};

void startSequence();
double FindDominantFrequency();
void DisplayNoteAndBar(double frequency);
void DetectClosestNote(double frequency, int *arr);
void NoteNumberToString(int note_number);
bool InRange(double frequency, double low_limit, double high_limit);
void ISRA3();
void ISRA4();
void blinkNote(int string);

long mapLong(long x, long in_min, long in_max, long out_min, long out_max);
int mapDouble(double x, double in_min, double in_max, double out_min, double out_max);

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define SAMPLES 2048            // This value MUST ALWAYS be a power of 2
#define SAMPLING_FREQUENCY 8000 // Hz, must be less than 10000 due to ADC
#define MAP_RES 8
#define NR_OF_OCTAVES 1
#define nrOfBims 1
#define SIGNAL_THR 300

unsigned int sampling_period_us;
unsigned long microseconds;

//#define USE_YIN

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[SAMPLES];
double vImag[SAMPLES];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

struct isr_val
{
  bool value = true;
  bool prevValue = true;
  bool isInterrupted = false;
  bool isProcessing = false;
  int buttonDelay = 20;
  int interruptPin;
};

typedef struct isr_val IsrVals;

IsrVals muteIsr;
IsrVals channelIsr;
// FFT variables
int temo = 0;
double tune = 0;
int tempo[100];
int16_t guitarSamples[1500];
double peakHys[2] = {0, 0};

double peaks[nrOfBims];
double bims[20];
int bimsCounter = 0;
double peak;
double highest = 1;
int lastString = -1;

double lowest = 4096;

#ifdef USE_YIN
// Yin variables
float pitch;
int buffer_length = 100;
Yin yin;
#endif

// the setup function runs once when you press reset or power the board

void ISRA3()
{
  if (!muteIsr.isProcessing)
  {
    muteIsr.isProcessing = true;
    muteIsr.isInterrupted = true;
    muteIsr.value = !muteIsr.value;
    if (muteIsr.prevValue != muteIsr.value)
    {

      digitalWrite(mutePin1, muteIsr.value ? LOW : HIGH);
      digitalWrite(MUTE_LED, muteIsr.value ? HIGH : LOW);
    }
  }
}
void ISRA4()
{
  if (!channelIsr.isProcessing)
  {
    channelIsr.isProcessing = true;
    channelIsr.isInterrupted = true;
    channelIsr.value = !channelIsr.value;

    if (channelIsr.prevValue != channelIsr.value)
    {

      digitalWrite(inputA, channelIsr.value ? HIGH : LOW);
      digitalWrite(inputB, channelIsr.value ? LOW : HIGH);
      digitalWrite(INPUT_A_LED, channelIsr.value ? HIGH : LOW);
      digitalWrite(INPUT_B_LED, channelIsr.value ? LOW : HIGH);
    }
  }
}

void setup()
{
  analogReadResolution(12); // Sets ADC resolution to 12 bits intead of 10 bits
  analogWriteResolution(12);
  // set pins for interrupt
  muteIsr.interruptPin = SWITCH_PIN_1;
  channelIsr.interruptPin = SWITCH_PIN_2;

  // Serial.begin(9600); // open the serial port at 9600 bps:
  //  Setup switch interrupt pins
  pinMode(muteIsr.interruptPin, INPUT_PULLUP);
  pinMode(channelIsr.interruptPin, INPUT_PULLUP);
  attachInterrupt(muteIsr.interruptPin, ISRA3, FALLING);
  attachInterrupt(channelIsr.interruptPin, ISRA4, FALLING);

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  pinMode(TUNER_IN, INPUT);

  pinMode(mutePin1, OUTPUT);
  pinMode(inputA, OUTPUT);
  pinMode(inputB, OUTPUT);

  pinMode(VERY_LOW, OUTPUT);
  pinMode(TOO_LOW, OUTPUT);
  pinMode(PERFECT, OUTPUT);
  pinMode(TOO_HIGH, OUTPUT);
  pinMode(VERY_HIGH, OUTPUT);

  pinMode(MUTE_LED, OUTPUT);
  pinMode(INPUT_A_LED, OUTPUT);
  pinMode(INPUT_B_LED, OUTPUT);

  digitalWrite(mutePin1, LOW);
  digitalWrite(MUTE_LED, muteIsr.value);

  digitalWrite(inputA, HIGH);
  digitalWrite(INPUT_A_LED, HIGH);

  digitalWrite(inputB, LOW);
  digitalWrite(INPUT_B_LED, LOW);

  // startSequence();
#ifdef USE_YIN
  Yin_init(&yin, buffer_length, 0.05);
#endif
}

void loop()
{
#ifdef USE_YIN
  for (int i = 0; i < 1500; i++)
  {
    guitarSamples[i] = analogRead(TUNER_IN) - (4096 / 2);
  }
  buffer_length = 100;
  while (pitch < 10 && buffer_length < 1500)
  {

    pitch = Yin_getPitch(&yin, guitarSamples);
    buffer_length++;
  }
  if (pitch != -1)
  {
    pitch = 0;
    buffer_length = 100;
  }

  for (int i = 0; i < 100; i++)
  {
    guitarSamples[i] = analogRead(TUNER_IN);
  }
  guitarSamples[0] = 0;
#endif
  DisplayNoteAndBar(FindDominantFrequency());
  if (muteIsr.isInterrupted && muteIsr.prevValue != muteIsr.value)
  {

    while (digitalRead(muteIsr.interruptPin) == LOW)
      ;
    delay(muteIsr.buttonDelay);
    muteIsr.prevValue = muteIsr.value;
    muteIsr.isInterrupted = false;
    muteIsr.isProcessing = false;
  }

  if (channelIsr.isInterrupted && channelIsr.prevValue != channelIsr.value)
  {

    while (digitalRead(channelIsr.interruptPin) == LOW)
      ;
    delay(channelIsr.buttonDelay);
    channelIsr.prevValue = channelIsr.value;
    channelIsr.isInterrupted = false;
    channelIsr.isProcessing = false;
  }
}

double FindDominantFrequency()
{
  peak = 0;
  highest = 1;

  lowest = 4096;

  for (int j = 0; j < nrOfBims; j++)
  {

    for (int i = 0; i < SAMPLES; i++)
    {
      microseconds = micros(); // Overflows after around 70 minutes!
      vReal[i] = analogRead(TUNER_IN);
      vImag[i] = 0;
      if (vReal[i] > highest)
      {
        highest = vReal[i];
      }
      if (vReal[i] < lowest && vReal[i] < highest)
      {
        lowest = vReal[i];
      }
      while (micros() < (microseconds + sampling_period_us))
        ;
    }
    /*FFT*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    peaks[j] = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  }
  // Serial.println(peak / 2);     //Print out what frequency is the most dominant.
  for (int i = 0; i < nrOfBims; i++)
  {
    peak += peaks[i];
  }
  peak /= nrOfBims;

  bims[bimsCounter++] = peak;
  if (bimsCounter == 10)
  {
    bimsCounter = 0;
  }
  if (peak > 2000 || (highest - lowest) < SIGNAL_THR)
  {
    peak = 0.0;
  }

  return (peak * 0.995);
}

void DisplayNoteAndBar(double frequency)
{

  int ArrayWithNoteAndBarWidth[2];
  ArrayWithNoteAndBarWidth[0] = 0;
  ArrayWithNoteAndBarWidth[1] = 150;

  DetectClosestNote(frequency, ArrayWithNoteAndBarWidth);
  /*if (lastString != ArrayWithNoteAndBarWidth[0])
  {
    lastString = ArrayWithNoteAndBarWidth[0];
    NoteNumberToString(ArrayWithNoteAndBarWidth[0]);
  }
  */
  // Serial.println("hehehe");

  // Serial.println(frequency);

  if (ArrayWithNoteAndBarWidth[1] < MAP_RES / 2)
  {

    if (ArrayWithNoteAndBarWidth[1] < (MAP_RES / 2) * 3 / 4)
    {
      digitalWrite(VERY_LOW, HIGH);
      digitalWrite(TOO_LOW, LOW);
      digitalWrite(PERFECT, LOW);
      digitalWrite(TOO_HIGH, LOW);
      digitalWrite(VERY_HIGH, LOW);
    }
    else
    {
      //  Serial.println("TOO LOW");
      // analogWrite(TOO_LOW, 200); // map(ArrayWithNoteAndBarWidth[1], 0, MAP_RES / 2, 0, 4096));
      digitalWrite(VERY_LOW, LOW);
      digitalWrite(TOO_LOW, HIGH);
      digitalWrite(PERFECT, LOW);
      digitalWrite(TOO_HIGH, LOW);
      digitalWrite(VERY_HIGH, LOW);
    }
  }
  else if (ArrayWithNoteAndBarWidth[1] > MAP_RES / 2 && ArrayWithNoteAndBarWidth[1] <= MAP_RES)
  {
    if (ArrayWithNoteAndBarWidth[1] > (MAP_RES / 2) * 1.25)
    {
      digitalWrite(VERY_LOW, LOW);
      digitalWrite(TOO_LOW, LOW);
      digitalWrite(PERFECT, LOW);
      digitalWrite(TOO_HIGH, LOW);
      digitalWrite(VERY_HIGH, HIGH);
    }
    else
    {
      //  Serial.println("TOO LOW");
      // analogWrite(TOO_LOW, 200); // map(ArrayWithNoteAndBarWidth[1], 0, MAP_RES / 2, 0, 4096));
      digitalWrite(VERY_LOW, LOW);
      digitalWrite(TOO_LOW, LOW);
      digitalWrite(PERFECT, LOW);
      digitalWrite(TOO_HIGH, HIGH);
      digitalWrite(VERY_HIGH, LOW);
    }

    // analogWrite(TOO_HIGH, 800); // map(ArrayWithNoteAndBarWidth[1], MAP_RES/2, MAP_RES, 4096, 0));
  }
  else if (ArrayWithNoteAndBarWidth[1] == MAP_RES / 2)
  {
    //            Serial.println("PERFECT");

    digitalWrite(VERY_LOW, LOW);
    digitalWrite(TOO_LOW, LOW);
    digitalWrite(PERFECT, HIGH);
    digitalWrite(TOO_HIGH, LOW);
    digitalWrite(VERY_HIGH, LOW);
  }
  else if (ArrayWithNoteAndBarWidth[1] == 555)
  {
    digitalWrite(VERY_LOW, LOW);
    digitalWrite(TOO_LOW, LOW);
    digitalWrite(PERFECT, LOW);
    digitalWrite(TOO_HIGH, LOW);
    digitalWrite(VERY_HIGH, LOW);
  }
}
void DetectClosestNote(double frequency, int *arr)
{
  arr[0] = 6;
  arr[1] = 555;
  for (int i = 1; i < sizeof notes / sizeof notes[0]; i++)
  {

    for (int j = 1; j < NR_OF_OCTAVES + 1; j++)
    {
      double note = (notes[i]);
      double lowDiff = 3;  //((notes[i] - notes[i - 1]) / 2);
      double highDiff = 3; //((notes[i + 1] - notes[i]) / 2);

      if (InRange(frequency, note - lowDiff, note + highDiff))
      {
        arr[0] = i;
        note = note;
        double mapping = mapDouble(frequency, note - lowDiff, note + highDiff, 1, MAP_RES);
        arr[1] = mapping;
        break;
      }
    }
  }
}

bool InRange(double frequency, double low_limit, double high_limit)
{
  if (frequency < high_limit && frequency > low_limit)
  {
    return true;
  }
  else
  {
    return false;
  }
}
void NoteNumberToString(int note_number)
{

  switch (note_number)
  {
  case 2:
    blinkNote(0);
    break;
  case 7:
    blinkNote(1);

    break;
  case 12:
    blinkNote(2);

    break;
  case 17:
    blinkNote(3);
    break;
  case 14:
    blinkNote(0);
  case 21:
    blinkNote(4);
  default:
    break;
  }
}
void blinkNote(int string)
{

  bool blink = false;
  for (int j = 0; j < sizeof tuningLeds / sizeof tuningLeds[0]; j++)
  {
    digitalWrite(tuningLeds[j], LOW);
  }
  for (int i = 0; i < 8; i++)
  {

    digitalWrite(tuningLeds[string], blink ? 4096 : 0);

    delay(30);

    blink = !blink;
  }
}
long mapLong(long x, long in_min, long in_max, long out_min, long out_max)
{
  long result;
  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return result;
}

int mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  int result;
  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return result;
}

void startSequence()
{
  for (int i = 0; i < 4096 / 2; i++)
  {
    for (int j = 0; j < 5; j++)
    {
      analogWrite(tuningLeds[j], i);
    }
    delayMicroseconds(900);
  }
  delay(1000);
  bool blink = false;
  for (int i = 0; i < 16; i++)
  {
    for (int j = 0; j < sizeof tuningLeds / sizeof tuningLeds[0]; j++)
    {
      analogWrite(tuningLeds[j], blink ? 4096 : 0);
    }
    delay(30);

    blink = !blink;
  }
  for (int j = 0; j < sizeof tuningLeds / sizeof tuningLeds[0]; j++)
  {
    analogWrite(tuningLeds[j], 0);
  }
}
//#include <stdint.h> /* For standard interger types (int16_t) */
//#include <stdlib.h> /* For call to malloc */
#include "Yin.h"
//#include <Arduino.h>

/* ------------------------------------------------------------------------------------------
--------------------------------------------------------------------------- PRIVATE FUNCTIONS
-------------------------------------------------------------------------------------------*/

/**
 * Step 1: Calculates the squared difference of the signal with a shifted version of itself.
 * @param buffer Buffer of samples to process.
 *
 * This is the Yin algorithms tweak on autocorellation. Read http://audition.ens.fr/adc/pdf/2002_JASA_YIN.pdf
 * for more details on what is in here and why it's done this way.
 */
void Yin_difference(Yin *yin, int16_t *buffer)
{
  int16_t i;
  int16_t tau;
  float delta;

  /* Calculate the difference for difference shift values (tau) for the half of the samples */
  for (tau = 0; tau < yin->halfBufferSize; tau++)
  {

    /* Take the difference of the signal with a shifted version of itself, then square it.
     * (This is the Yin algorithm's tweak on autocorellation) */
    for (i = 0; i < yin->halfBufferSize; i++)
    {
      delta = buffer[i] - buffer[i + tau];
      yin->yinBuffer[tau] += delta * delta;
    }
  }
}

/**
 * Step 2: Calculate the cumulative mean on the normalised difference calculated in step 1
 * @param yin #Yin structure with information about the signal
 *
 * This goes through the Yin autocorellation values and finds out roughly where shift is which
 * produced the smallest difference
 */
void Yin_cumulativeMeanNormalizedDifference(Yin *yin)
{
  int16_t tau;
  float runningSum = 0;
  yin->yinBuffer[0] = 1;

  /* Sum all the values in the autocorellation buffer and nomalise the result, replacing
   * the value in the autocorellation buffer with a cumulative mean of the normalised difference */
  for (tau = 1; tau < yin->halfBufferSize; tau++)
  {
    runningSum += yin->yinBuffer[tau];
    yin->yinBuffer[tau] *= tau / runningSum;
  }
}

/**
 * Step 3: Search through the normalised cumulative mean array and find values that are over the threshold
 * @return Shift (tau) which caused the best approximate autocorellation. -1 if no suitable value is found over the threshold.
 */
int16_t Yin_absoluteThreshold(Yin *yin)
{
  int16_t tau;

  /* Search through the array of cumulative mean values, and look for ones that are over the threshold
   * The first two positions in yinBuffer are always so start at the third (index 2) */
  for (tau = 2; tau < yin->halfBufferSize; tau++)
  {
    if (yin->yinBuffer[tau] < yin->threshold)
    {
      while (tau + 1 < yin->halfBufferSize && yin->yinBuffer[tau + 1] < yin->yinBuffer[tau])
      {
        tau++;
      }
      /* found tau, exit loop and return
       * store the probability
       * From the YIN paper: The yin->threshold determines the list of
       * candidates admitted to the set, and can be interpreted as the
       * proportion of aperiodic power tolerated
       * within a periodic signal.
       *
       * Since we want the periodicity and and not aperiodicity:
       * periodicity = 1 - aperiodicity */
      yin->probability = 1 - yin->yinBuffer[tau];
      break;
    }
  }

  /* if no pitch found, tau => -1 */
  if (tau == yin->halfBufferSize || yin->yinBuffer[tau] >= yin->threshold)
  {
    tau = -1;
    yin->probability = 0;
  }

  return tau;
}

/**
 * Step 5: Interpolate the shift value (tau) to improve the pitch estimate.
 * @param  yin         [description]
 * @param  tauEstimate [description]
 * @return             [description]
 *
 * The 'best' shift value for autocorellation is most likely not an interger shift of the signal.
 * As we only autocorellated using integer shifts we should check that there isn't a better fractional
 * shift value.
 */
float Yin_parabolicInterpolation(Yin *yin, int16_t tauEstimate)
{
  float betterTau;
  int16_t x0;
  int16_t x2;

  /* Calculate the first polynomial coeffcient based on the current estimate of tau */
  if (tauEstimate < 1)
  {
    x0 = tauEstimate;
  }
  else
  {
    x0 = tauEstimate - 1;
  }

  /* Calculate the second polynomial coeffcient based on the current estimate of tau */
  if (tauEstimate + 1 < yin->halfBufferSize)
  {
    x2 = tauEstimate + 1;
  }
  else
  {
    x2 = tauEstimate;
  }

  /* Algorithm to parabolically interpolate the shift value tau to find a better estimate */
  if (x0 == tauEstimate)
  {
    if (yin->yinBuffer[tauEstimate] <= yin->yinBuffer[x2])
    {
      betterTau = tauEstimate;
    }
    else
    {
      betterTau = x2;
    }
  }
  else if (x2 == tauEstimate)
  {
    if (yin->yinBuffer[tauEstimate] <= yin->yinBuffer[x0])
    {
      betterTau = tauEstimate;
    }
    else
    {
      betterTau = x0;
    }
  }
  else
  {
    float s0, s1, s2;
    s0 = yin->yinBuffer[x0];
    s1 = yin->yinBuffer[tauEstimate];
    s2 = yin->yinBuffer[x2];
    // fixed AUBIO implementation, thanks to Karl Helgason:
    // (2.0f * s1 - s2 - s0) was incorrectly multiplied with -1
    betterTau = tauEstimate + (s2 - s0) / (2 * (2 * s1 - s2 - s0));
  }

  return betterTau;
}

/* ------------------------------------------------------------------------------------------
---------------------------------------------------------------------------- PUBLIC FUNCTIONS
-------------------------------------------------------------------------------------------*/

/**
 * Initialise the Yin pitch detection object
 * @param yin        Yin pitch detection object to initialise
 * @param bufferSize Length of the audio buffer to analyse
 * @param threshold  Allowed uncertainty (e.g 0.05 will return a pitch with ~95% probability)
 */
void Yin_init(Yin *yin, int16_t bufferSize, float threshold)
{
  /* Initialise the fields of the Yin structure passed in */
  yin->bufferSize = bufferSize;
  yin->halfBufferSize = bufferSize / 2;
  yin->probability = 0.0;
  yin->threshold = threshold;

  /* Allocate the autocorellation buffer and initialise it to zero */
  yin->yinBuffer = (float *)malloc(sizeof(float) * yin->halfBufferSize);

  int16_t i;
  for (i = 0; i < yin->halfBufferSize; i++)
  {
    yin->yinBuffer[i] = 0;
  }
}

/**
 * Runs the Yin pitch detection algortihm
 * @param  yin    Initialised Yin object
 * @param  buffer Buffer of samples to analyse
 * @return        Fundamental frequency of the signal in Hz. Returns -1 if pitch can't be found
 */
float Yin_getPitch(Yin *yin, int16_t *buffer)
{
  int16_t tauEstimate = -1;
  float pitchInHertz = -1;

  /* Step 1: Calculates the squared difference of the signal with a shifted version of itself. */
  Yin_difference(yin, buffer);

  /* Step 2: Calculate the cumulative mean on the normalised difference calculated in step 1 */
  Yin_cumulativeMeanNormalizedDifference(yin);

  /* Step 3: Search through the normalised cumulative mean array and find values that are over the threshold */
  tauEstimate = Yin_absoluteThreshold(yin);

  /* Step 5: Interpolate the shift value (tau) to improve the pitch estimate. */
  if (tauEstimate != -1)
  {
    pitchInHertz = YIN_SAMPLING_RATE / Yin_parabolicInterpolation(yin, tauEstimate);
  }

  return pitchInHertz;
}

/**
 * Certainty of the pitch found
 * @param  yin Yin object that has been run over a buffer
 * @return     Returns the certainty of the note found as a decimal (i.e 0.3 is 30%)
 */
float Yin_getProbability(Yin *yin)
{
  return yin->probability;
}
