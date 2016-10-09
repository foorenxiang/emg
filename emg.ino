#include <Kalman.h>

#include <Wire.h>

#define address 0x40
double measurement, filteredMeasurement;
Kalman myFilter(0.125,30,1023,0); //suggested initial values for high noise filtering

#define samplesize 4
float sampleemg[samplesize] = {0};

float changeemg = 0;

float changeemg2nd = 0;

float changeemg3rd = 0;

float peakemg = 0;

int xpeakval = 2;
int ypeakval = 2;
int zpeakval = 2;

int t = 0; //tracker
int flag = 0; //first sample set tracker
int peakflag = 0;
int thresholdMillis = 1800;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  delay(50);
}

void loop()
{
  myFilter.setParameters(0.125,30,1023);

  readEMG();
  if(flag == 1 && t == 0){
    firstorder(sampleemg, &changeemg);

    secondorder(sampleemg, &changeemg2nd);

    thirdorder(sampleemg, &changeemg3rd);
  }
  
  Serial.print(changeemg2nd);
  Serial.println("");
  
  t++;
  if(t == samplesize) t = 0;
  flag = 1;

  delay(50);
}

void readEMG(){
  /*set kalman params for EMG sensors*/
  //myFilter.setParameters(0.125,32,1023); //default values
  measurement = (double) analogRead(A0);
  filteredMeasurement = myFilter.getFilteredValue(measurement);
  Serial.print(filteredMeasurement);
  Serial.println(" ");
  sampleemg[t] = filteredMeasurement;
}

void firstorder(float* sample, float* change){
  int i;
  for(i = 0, *change = 0; i<samplesize-1; i++){
    *change +=  (sample[i+1] - sample[i]);
  }
  
    *change /= samplesize-1;
}

void secondorder(float* sample, float* change){
  int i;

  for(i = 0, *change = 0; i<samplesize-2; i++){
    *change += (int) (sample[i+2] - sample[i+1])-(sample[i+1] - sample[i]);
  }
  
  *change /= samplesize-2;
}

void thirdorder(float* sample, float* change){
  int i;

  for(i = 0, *change = 0; i<samplesize-3; i++){
    *change += (int) ((sample[i+3] - sample[i+2])-(sample[i+2] - sample[i+1])) - ((sample[i+2] - sample[i+1])-(sample[i+1] - sample[i]));
  }
  
  *change /= samplesize-3;
}
