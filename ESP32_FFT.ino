#include <Wire.h> // necessary for I2C (SDA/SCL) communication
#include <Adafruit_Sensor.h> // BNO055 sensor library
#include <Adafruit_BNO055.h> // BNO055 sensor library
#include <utility/imumaths.h> // enables mathematical operations for processing sensor data
#include <arduinoFFT.h> // enables FFT functionality
 
#define SAMPLING_FREQ 1000.0 // BNO055 accelerometer-only mode sampling frequency (Hz)
#define SAMPLES 64 // FFTs will be computed in batches of 64 samples for resolution of 15.6 Hz
 
Adafruit_BNO055 proTechIMU = Adafruit_BNO055(); // Defines proTechIMU "object" from "Adafruit_BNO055" class
 
// variables for storing acceleration readings
double accValues[SAMPLES]; // initialize vector for storing 64 acceleration readings
double imag[SAMPLES]; // initialize imaginary part for FFT
int i = 0; // defines counter index to start at 0

// variables for maintaining 1000 Hz sampling frequency
unsigned long sampleTime = 0; // initialize sampleTime variable for maintaining 1000 Hz sampling rate
const unsigned long sampleInterval = 1000000.0/SAMPLING_FREQ; // 1 s (i.e. 1e6 µs)/SAMPLING_FREQ

// classification variables
double targetFreq = 400.0; // target cutoff frequency for classification
double magThreshold = 0.5; // magnitude cutoff for > targetFreq for trial classification
int targetIndex; // declare global variable target index for classification
int endIndex = 50; //

// define "FFT" object from "ArduinoFFT" class
ArduinoFFT<double> FFT = ArduinoFFT<double>(accValues, imag, (uint_fast16_t)SAMPLES, (double)SAMPLING_FREQ); // defines "FFT" object from "ArduinoFFT" class
 
void setup() {

  Serial.begin(115200); // start the serial monitor with baud 115200
  proTechIMU.setMode(OPERATION_MODE_ACCONLY); // set BNO055 to accelerometer-only mode
  proTechIMU.setExtCrystalUse(true); // our BNO055 uses an external crystal for improved accuracy
  
  // Stop code if IMU does not initialize properly
  if (!proTechIMU.begin()) {
    Serial.println("Failed to initialize BNO055");
    while (1);
  }
 
  delay(1000); // setup delay to ensure proper setup completes before reading data
  Serial.println("IMU properly initialized");

  // define targetIndex variable (used later for trial classification)
  targetIndex = (int) ((targetFreq * SAMPLES) / SAMPLING_FREQ); // calculate the appropriate index for 64 bin FFT at pre-defined target freq
}
 
// Defines fuction to collect & store acceleration values
  void collectAndStore() {
    sensors_event_t event; // defines struct data type to store acceleration data (from Adafruit_Sensor.h)
    proTechIMU.getEvent(&event); // collects acceleration data point and uses & pointer to store it in the variable "event"
 
    // calculate magnitude of acceleration vector and store in appropriate index "i" of accValues vector
    accValues[i] = sqrt(
      event.acceleration.x * event.acceleration.x +
      event.acceleration.y * event.acceleration.y +
      event.acceleration.z * event.acceleration.z
      );
    
    imag[i] = 0.0; // store 0.0 in appropriate index for imaginary vector
 
    // print index and magnitude for verification
    // Serial.print("Index: "); Serial.print(i);
    // Serial.print(" Accel Mag: "); Serial.println(accValues[i]);

    //Serial.print(" "); Serial.print(event.acceleration.x);
    //Serial.print(","); Serial.print(event.acceleration.y);
    //Serial.print(","); Serial.println(event.acceleration.z);

 
    i = i + 1; // move to next index
    if (i>= SAMPLES) i = 0; // reset loop after 64 samples
 
  }

 

void loop() {
  
  // ensure 1000 Hz sampling frequency
  if (micros() - sampleTime >= sampleInterval) { // if the last sample time is greater than the alloted 1000 Hz sampling interval - THEN run the collect&store code
    sampleTime = micros();
    collectAndStore(); // read the acceleration data from BNO055
  }

  
  if (i == 0) { // check if the collectAndStore function finished a previous bin, 
    //run FFT code
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // selects Hamming Window from FFT Window class and Forward FFT from FFTDirection class & applies to the "FFT" object
    FFT.compute(FFTDirection::Forward); // compute Forward Fourier Transform applied to the "FFT" object
    FFT.complexToMagnitude(); // applied to "FFT" object

    // Output frequency spectrum
    for (int j = 0; j < SAMPLES; j++) { // loop runs through all 64 FFT frequency bins
      double freq = (j * SAMPLING_FREQ) / SAMPLES; // for each bin, define the frequency
      //Serial.print(freq); // then print the frequency for the appropriate bin in this iteration of the loop
      //Serial.print(","); // add a comma
      //Serial.println(accValues[j]); // print the magnitude of frequency for the specified bin
      // Note: accValues[] originally holds acceleration values - however, after the FFT, it becomes overwritten and now hold frequency magnitude
    }

    bool motion = false; // set inital condition for boolean variable to false

    for (int j = targetIndex; j < endIndex; j++) { // check through most recent FFT for threshold value
      if (accValues[j] > magThreshold) {
        motion = true; // if the magnitude threshold is found, change the boolean variable to 'true'
        break;
      }
    }
    
    // print statements for motion and no motion
    if (motion) {
      Serial.println("Motion Detected"); 
    }
      else {
      Serial.println(". . .");
    }

    Serial.println();
    delay(5); // allow time for serial monitor to print

  }

  }



