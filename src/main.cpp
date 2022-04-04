#include <Arduino.h>
#include <arduinoFFT.h>

#define mutePin1 PB4 // 1
#define inputA PB5   // 7
#define inputB PB6   // 11

#define SWITCH_PIN_1 PA3 // 9
#define SWITCH_PIN_2 PA4 // 10
#define MUTE_LED PC13    // 8
#define INPUT_A_LED PC14
#define INPUT_B_LED PC15

#define TUNER_IN PA0

#define TOO_LOW PB1  // 4
#define PERFECT PB2  // 5
#define TOO_HIGH PB10 // 6

#define STRING_E1 A1
#define STRING_B A1
#define STRING_G A1
#define STRING_D A1
#define STRING_A A1
#define STRING_E2 A1

double FindDominantFrequency();
void DisplayNoteAndBar(double frequency);
void DetectClosestNote(double frequency, int *arr);
char NoteNumberToString(int note_number);
bool InRange(double frequency, int low_limit, int high_limit);
void ISRA3();
void ISRA4();

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define SAMPLES 512             // This value MUST ALWAYS be a power of 2
#define SAMPLING_FREQUENCY 4000 // Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

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

  pinMode(PA0, INPUT_PULLUP);

  pinMode(mutePin1, OUTPUT);
  pinMode(inputA, OUTPUT);
  pinMode(inputB, OUTPUT);
  pinMode(TUNER_IN, INPUT);
  pinMode(TOO_LOW, OUTPUT);
  pinMode(PERFECT, OUTPUT);
  pinMode(TOO_HIGH, OUTPUT);

  pinMode(MUTE_LED, OUTPUT);
  pinMode(INPUT_A_LED, OUTPUT);
  pinMode(INPUT_B_LED, OUTPUT);

  digitalWrite(mutePin1, LOW);
  digitalWrite(MUTE_LED, muteIsr.value);

  digitalWrite(inputA, HIGH);
  digitalWrite(INPUT_A_LED, HIGH);

  digitalWrite(inputB, LOW);
  digitalWrite(INPUT_B_LED, LOW);
}

void loop()
{


   DisplayNoteAndBar(FindDominantFrequency());
  //FindDominantFrequency();
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
  for (int i = 0; i < SAMPLES; i++)
  {
    microseconds = micros(); // Overflows after around 70 minutes!
    vReal[i] = 65;           // analogRead(TUNER_IN);
    vImag[i] = 0;
    while (micros() < (microseconds + sampling_period_us))
      ;
  }

  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  // Serial.println(peak / 2);     //Print out what frequency is the most dominant.

  peak = (peak * 0.990) / 2;
  return peak;
}

void DisplayNoteAndBar(double frequency)
{

  int ArrayWithNoteAndBarWidth[2];
  DetectClosestNote(frequency, ArrayWithNoteAndBarWidth);

  // char note = NoteNumberToString(ArrayWithNoteAndBarWidth[0]);
  // Serial.println("hehehe");

  // Serial.println(frequency);

  if (ArrayWithNoteAndBarWidth[1] < 64)
  {
    //  Serial.println("TOO LOW");
    analogWrite(TOO_LOW, map(ArrayWithNoteAndBarWidth[1], 0, 64, 0, 255));
    digitalWrite(PERFECT, LOW);
    digitalWrite(TOO_HIGH, LOW);
  }
  else if (ArrayWithNoteAndBarWidth[1] > 64)
  {
    //    Serial.println("TOO HIGH");

    analogWrite(TOO_HIGH, map(ArrayWithNoteAndBarWidth[1], 64, 128, 255, 0));
    digitalWrite(PERFECT, LOW);
    digitalWrite(TOO_LOW, LOW);
  }
  else
  {
    //            Serial.println("PERFECT");

    digitalWrite(PERFECT, HIGH);
    digitalWrite(TOO_LOW, LOW);
    digitalWrite(TOO_HIGH, LOW);
  }
}

void DetectClosestNote(double frequency, int *arr)
{
  if (/*ukuleleMode == */ true)
  {
    /*if(InRange(frequency, 62, 102)){
    arr[0] = 6;
    arr[1] = map(frequency, 62, 102, 1, 128);
  }*/
    // arr[0] = 6;
    // arr[1] = map(frequency, 41.2 - 5, 41.2 + 5, 1, 128);
    if (InRange(frequency, 41.2 - 5, 41.2 + 5))
    { // E 41.2

      arr[0] = 6;
      arr[1] = map(frequency, 41.2 - 5, 41.2 + 5, 1, 128);
    }
    else if (InRange(frequency, 55 - 5, 55 + 5))
    { // A 55

      arr[0] = 6;
      arr[1] = map(frequency, 55 - 5, 55 + 5, 1, 128);
    }
    else if (InRange(frequency, 73.4 - 5, 73.4 + 5))
    { // D 73.4

      arr[0] = 6;
      arr[1] = map(frequency, 73.4 - 5, 73.4 + 5, 1, 128);
    }
    else if (InRange(frequency, 98 - 5, 98 + 5))
    { // G 98

      arr[0] = 6;
      arr[1] = map(frequency, 98 - 5, 98 + 5, 1, 128);
    }
    else
    {
      arr[0] = 6;
      arr[1] = map(0, 94, 102, 1, 128);
    } /*else if(InRange(frequency, 100, 120)){
arr[0] = 5;
arr[1] = map(frequency, 100, 120, 1, 128);
}else if(InRange(frequency, 120, 165)){
arr[0] = 4;
arr[1] =  map(frequency, 127, 167, 1, 128);
}else if(InRange(frequency, 165, 210)){
arr[0] = 3;
arr[1] = map(frequency, 176, 216, 1, 128);
}else if(InRange(frequency, 210, 290)){
arr[0] = 2;
arr[1] = map(frequency, 217, 277, 1, 128);
}else if(InRange(frequency, 290, 380)){
arr[0] = 1;
arr[1] = map(frequency, 290, 370, 1, 128);
}*/
  }
}

bool InRange(double frequency, int low_limit, int high_limit)
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
char NoteNumberToString(int note_number)
{

  switch (note_number)
  {
  case 1:
    digitalWrite(STRING_E1, HIGH);
    digitalWrite(STRING_B, LOW);
    digitalWrite(STRING_G, LOW);
    digitalWrite(STRING_D, LOW);
    digitalWrite(STRING_A, LOW);
    digitalWrite(STRING_E2, LOW);

    return 'E';
    break;
  case 2:
    digitalWrite(STRING_E1, LOW);
    digitalWrite(STRING_B, HIGH);
    digitalWrite(STRING_G, LOW);
    digitalWrite(STRING_D, LOW);
    digitalWrite(STRING_A, LOW);
    digitalWrite(STRING_E2, LOW);
    return 'B';
    break;
  case 3:
    digitalWrite(STRING_E1, LOW);
    digitalWrite(STRING_B, LOW);
    digitalWrite(STRING_G, HIGH);
    digitalWrite(STRING_D, LOW);
    digitalWrite(STRING_A, LOW);
    digitalWrite(STRING_E2, LOW);
    return 'G';
    break;
  case 4:
    digitalWrite(STRING_E1, LOW);
    digitalWrite(STRING_B, LOW);
    digitalWrite(STRING_G, LOW);
    digitalWrite(STRING_D, HIGH);
    digitalWrite(STRING_A, LOW);
    digitalWrite(STRING_E2, LOW);
    return 'D';
    break;
  case 5:
    digitalWrite(STRING_E1, LOW);
    digitalWrite(STRING_B, LOW);
    digitalWrite(STRING_G, LOW);
    digitalWrite(STRING_D, LOW);
    digitalWrite(STRING_A, HIGH);
    digitalWrite(STRING_E2, LOW);
    return 'A';
    break;
  case 6:
    digitalWrite(STRING_E1, LOW);
    digitalWrite(STRING_B, LOW);
    digitalWrite(STRING_G, LOW);
    digitalWrite(STRING_D, LOW);
    digitalWrite(STRING_A, LOW);
    digitalWrite(STRING_E2, HIGH);
    return 'e';
    break;
    default: 
  return '-';
  }
}
