#include <Adafruit_Protomatter.h> 
#include "arduinoFFT.h"

// Screen Defines
#define HEIGHT  32 // Matrix height (pixels)
#define WIDTH   64 // Matrix width (pixels)
#define MAX_FPS 60            // Maximum redraw rate, frames/second
#define ADC_SAMPLERATE 30000  // Freerunning ADC samplerate
#define FFT_SMOOTHINGFACTOR 2 // Number of sample sets to hold and average
#define CONVERSIONS_PER_PIN 1 // Analog Conversions

// Visualization Settings
#define N_COLORS   4     // Number of colors per pallet
#define N_COLORPAL 2     // Number of separate color pallets
#define N_BRIGHTNESS 2   // Number of brightness settings
#define N_FFTBINS 8      // Number of bins for FFT display

// Pins for the switches and the 1 potentiometer
int iVisPin = A4; 
int iADCPin = A3;
int iColorPin = A2;
int iBrightnessPin = A1;

#if defined(_VARIANT_MATRIXPORTAL_M4_) // MatrixPortal M4
uint8_t rgbPins[]  = {7, 8, 9, 10, 11, 12};
uint8_t addrPins[] = {17, 18, 19, 20, 21};
uint8_t clockPin   = 14;
uint8_t latchPin   = 15;
uint8_t oePin      = 16;
#else // MatrixPortal ESP32-S3
uint8_t rgbPins[]  = {42, 40, 41, 38, 37, 39}; 
uint8_t addrPins[] = {45, 36, 48, 35, 21};
uint8_t clockPin   = 2;
uint8_t latchPin   = 47;
uint8_t oePin      = 14;

#endif

#if HEIGHT == 16
#define NUM_ADDR_PINS 3
#elif HEIGHT == 32
#define NUM_ADDR_PINS 4
#elif HEIGHT == 64
#define NUM_ADDR_PINS 5
#endif

// ADC Setup
adc_continuous_result_t * ADCresult = NULL;
uint32_t t; 
uint32_t prevADCTime; 
int iTempADCLocation = 0;
int iADCBufferB[64];
int iADCBufferA[64];
int* pReadADCBuffer =  iADCBufferA;
int* pWriteADCBuffer = iADCBufferB;
volatile bool bADCComplete;

// Visualization Setup
uint16_t colors[N_COLORPAL][N_BRIGHTNESS][N_COLORS];
int iColorPal = 0;
int iBrightness = 1;
int iVisualizationMode = 0;
uint32_t prevFPSTime;


// FFT Vars
double iFFTSamples[N_FFTBINS*2];
double vReal[N_FFTBINS*2];
double vImag[N_FFTBINS*2] {0};
uint16_t iFFTValues[FFT_SMOOTHINGFACTOR][N_FFTBINS];

// Hardcoded equalization curve
float fScalingFactors[N_FFTBINS] {1,1,1,1,1,1,1,1};
int iFFTBinWidth = floor(WIDTH/N_FFTBINS);


// FFT Object 
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, N_FFTBINS*2, ADC_SAMPLERATE);

// Matrix Object
Adafruit_Protomatter matrix(
  WIDTH, 4, 1, rgbPins, NUM_ADDR_PINS, addrPins,
  clockPin, latchPin, oePin, true);

// ISR for analog reads 
void ARDUINO_ISR_ATTR adcComplete() 
  {
    bADCComplete = true;
  }

// Store ADC value when triggered by ISR
void storeSingleADCValue()
  {
  bADCComplete = false;
  if(analogContinuousRead(&ADCresult, 0))
    {
    pWriteADCBuffer[iTempADCLocation++] = ADCresult->avg_read_raw;
    }
  }

// Busy-wait and fill the ADC buffer
void fillADCBuffer()
  {
  analogContinuousStart();
  //We will hang here until the ADC has returned a value
  while (iTempADCLocation < 64)
    {
      if (bADCComplete)
        {
        storeSingleADCValue();
        }
    }

  // Swap the buffers so consumers can use the values
  iTempADCLocation = 0;
  int* pTemp = pReadADCBuffer;
  pReadADCBuffer = pWriteADCBuffer;
  pWriteADCBuffer = pTemp;

  // Close down ADC
  analogContinuousStop();
  }



// Take samples and calculate FFT
void fCalcFFT()
  {
  for (int i = 0; i < N_FFTBINS*2; i++)
    {
    vReal[i] = pReadADCBuffer[i];
    vImag[i] = 0.0;
    }
  FFT.dcRemoval(vReal, N_FFTBINS*2);
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // nFill bins with calculated values
  for (int i = 0; i< N_FFTBINS; i++)
    {
    // Scale according to the EQ factors
    vReal[i] = fScalingFactors[i]*vReal[i];
    float fTempValue = 1+ map(vReal[i],0,8000,0,30);
    iFFTValues[0][N_FFTBINS -1 -i] = floor(fTempValue);
    }
  }  

// Take samples and calculate FFT
void fDrawFFT()
  {
  fillADCBuffer();
  fCalcFFT();
  matrix.fillScreen(0x0);
  int iFullBoxHeight = floor(HEIGHT/N_COLORS);
  // Iterate drawing FFT boxes 
  for(int nCurrentBin=0; nCurrentBin < N_FFTBINS ; nCurrentBin++) 
    { 
    int iXValue = floor((WIDTH/N_FFTBINS)*nCurrentBin); //Box X value
    
    float iYValue = 0.0;


    //Perform Smoothing using last values
    for (int i = 0; i<FFT_SMOOTHINGFACTOR; i++)
      {
        iYValue = iYValue + ((float)iFFTValues[i][nCurrentBin]/(float)FFT_SMOOTHINGFACTOR);
      }

    //Iterate through drawing the different colored boxes
    if(iYValue >= iFullBoxHeight)
      {
      //Full Bottom Box
      matrix.fillRect(iXValue,0,iFFTBinWidth,iFullBoxHeight,colors[iColorPal][iBrightness][0]); 
      if(iYValue >= 2*iFullBoxHeight)
        {
        //Full Second Box
        matrix.fillRect(iXValue, iFullBoxHeight, iFFTBinWidth,iFullBoxHeight,colors[iColorPal][iBrightness][1]); 
        if(iYValue >= 3*iFullBoxHeight)
          {
          //Full Third Box
          matrix.fillRect(iXValue, 2*iFullBoxHeight, iFFTBinWidth,iFullBoxHeight,colors[iColorPal][iBrightness][2]);
          if(iYValue >= 4*iFullBoxHeight)
            {
            //Full Fourth Box
            matrix.fillRect(iXValue, 3*iFullBoxHeight, iFFTBinWidth,iFullBoxHeight,colors[iColorPal][iBrightness][3]);
            }
          else
            {
            //Partial Fourth Box
            matrix.fillRect(iXValue, 3*iFullBoxHeight, iFFTBinWidth, iYValue - 3*iFullBoxHeight, colors[iColorPal][iBrightness][3]);
            }
          }
        else
          {
          //Partial Third Box
          matrix.fillRect(iXValue, 2*iFullBoxHeight,iFFTBinWidth,iYValue - 2*iFullBoxHeight,colors[iColorPal][iBrightness][2]); 
          }
        }
      else
        {
        //Partial Second Box
        matrix.fillRect(iXValue, iFullBoxHeight,iFFTBinWidth,iYValue - iFullBoxHeight,colors[iColorPal][iBrightness][1]); 
        }
      } 
      else
        {
        //Partial bottom box
        matrix.fillRect(iXValue, 0,iFFTBinWidth,iYValue,colors[iColorPal][iBrightness][0]); 
        }
      }

    //Push values forward to make room in iFFTValues[0] for the next set of samples
    for (int i = FFT_SMOOTHINGFACTOR-1; i>0; i--)
      {
      for (int j = 0; j< N_FFTBINS; j++)
        {
        iFFTValues[i][j] = iFFTValues[i-1][j];
        }  
      }
    matrix.show();
  }

// Capture samples and display the waveform
void fDrawWave()
  {
  int iVertAdjust = 0;
  fillADCBuffer();
  matrix.fillScreen(0x0);
  for (int i = 0; i<64; i++)
    {
    pReadADCBuffer[i] = map(pReadADCBuffer[i],0,4095,0,31);
    int iAdjustedVal = abs(pReadADCBuffer[i] -15); // magnitude 0 to 16
    int iSign = ((pReadADCBuffer[i]-15)/abs(pReadADCBuffer[i]-15));
    int iCenterPoint = 15;
    if (pReadADCBuffer[i] == 15)
      {
      iSign = 1;
      iAdjustedVal = 1;
      }

    // Partial first bar
    if (iAdjustedVal <= floor(HEIGHT/8))
      {
      matrix.drawLine(i, iCenterPoint, i, pReadADCBuffer[i], colors[iColorPal][iBrightness][0]); 
      }

    // Partial second bar. Full first bar
    else if (iAdjustedVal <= floor(HEIGHT/6))
      {
      matrix.drawLine(i, iCenterPoint, i, pReadADCBuffer[i], colors[iColorPal][iBrightness][1]); 
      matrix.drawLine(i, iCenterPoint, i, iCenterPoint + iSign*floor(HEIGHT/8), colors[iColorPal][iBrightness][0]);
      }

    // Partial third bar. Full first and second bar
    else if (iAdjustedVal <= floor(HEIGHT/4))
      {  
      matrix.drawLine(i, iCenterPoint, i, pReadADCBuffer[i], colors[iColorPal][iBrightness][2]); 
      matrix.drawLine(i, iCenterPoint, i, iCenterPoint + iSign*floor(HEIGHT/6), colors[iColorPal][iBrightness][1]);  
      matrix.drawLine(i, iCenterPoint, i, iCenterPoint + iSign*floor(HEIGHT/8), colors[iColorPal][iBrightness][0]); 
      }
    else
    // Partial fourth bar. Full all other bars
      {
      matrix.drawLine(i, iCenterPoint, i, pReadADCBuffer[i], colors[iColorPal][iBrightness][3]); 
      matrix.drawLine(i, iCenterPoint, i, iCenterPoint + iSign*floor(HEIGHT/4), colors[iColorPal][iBrightness][2]);
      matrix.drawLine(i, iCenterPoint, i, iCenterPoint + iSign*floor(HEIGHT/6), colors[iColorPal][iBrightness][1]);
      matrix.drawLine(i, iCenterPoint, i, iCenterPoint + iSign*floor(HEIGHT/8), colors[iColorPal][iBrightness][0]);
      }
    }
  matrix.show();
  }

void setup(void) 
  {
  // PinModes
  pinMode(iADCPin, ANALOG);
  pinMode(iVisPin, INPUT_PULLUP);
  pinMode(iBrightnessPin, INPUT_PULLUP);
  pinMode(iColorPin, INPUT_PULLUP);

  ProtomatterStatus status = matrix.begin();

  // Define 2 color pallets: cold and warm 
  // Also 2 brightnesses
  // Cold
  colors[0][0][0] = matrix.color565(42,11,58);
  colors[0][0][1] = matrix.color565(63,28,91);
  colors[0][0][2] = matrix.color565(78,68,127); 
  colors[0][0][3] = matrix.color565(75,102,116);

  colors[0][1][0] = matrix.color565(0x54,0x17,0x75);
  colors[0][1][1] = matrix.color565(0x7E,0x38,0xB7);
  colors[0][1][2] = matrix.color565(0x9C,0x89,0xFE); 
  colors[0][1][3] = matrix.color565(0x99,0xCC,0xED);

  //Warm
  colors[1][0][0] = matrix.color565(91,36,0);
  colors[1][0][1] = matrix.color565(124,61,0);
  colors[1][0][2] = matrix.color565(128,75,0);
  colors[1][0][3] = matrix.color565(128,94,0);

  colors[1][1][0] = matrix.color565(183,72,0);
  colors[1][1][1] = matrix.color565(248,122,0);
  colors[1][1][2] = matrix.color565(255,150,0);
  colors[1][1][3] = matrix.color565(255,188,0);

    //ADC Preparation
  uint8_t adc_pins[1] = {iADCPin};
  analogContinuous(adc_pins,1,CONVERSIONS_PER_PIN, ADC_SAMPLERATE, &adcComplete);
  }

void loop() 
  {
  iVisualizationMode = digitalRead(iVisPin);
  iColorPal = digitalRead(iColorPin);
  iBrightness = digitalRead(iBrightnessPin);
  
  if (iVisualizationMode == 0)
    {
    fDrawWave();
    }
  else
    {
    // Only the FFT display is rate-limited
    while(((t = micros()) - prevFPSTime) < (1000000L / MAX_FPS));
    fDrawFFT();
    }
  prevFPSTime = micros();
  }
