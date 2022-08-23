#include <EEPROM.h>
#include "SerialCommand.h"
#include "EEPROMAnything.h"

#define SerialPort Serial1
SerialCommand sCmd(SerialPort);   // The demo SerialCommand object

int channel[] = {A1, A2, A3, A4}; //inputs van de 6 sensoren
int value[6]; //array voor waarden sensoren
float average; //int voor interpolatie
float normalized[6]; //array voor genormaliseerde waarden

int pinAin1 = 9;
int pinAin2 = 11;
int pinBin1 = 10;
int pinBin2 = 5;

int startknop = 2;
int stopknop = 3;

unsigned long lastTime;
double Input, output, Setpoint;
double iTerm, lastInput;
float error;
double timeChange, dInput;
unsigned long now;
float outputP;
float outputI;
float outputD;

struct param_t
{
  bool start;
  int speed;
  float kp;
  float ki;
  float kd;
  unsigned long cycleTime;
  int white[4]; //array voor kalibratie wit
  int black[4]; //array voor kalibratie zwart
} params;


unsigned long time, prevTime, delta, calculationTime;


void setup()
{  
  attachInterrupt(0, onInterrupt, RISING);
    
  SerialPort.begin(115200); 
  sCmd.addCommand("start", onStart);
  sCmd.addCommand("stop", onStop);
  sCmd.addCommand("set", onSet);
  sCmd.addCommand("debug", onDebug);
  sCmd.addCommand("reset", onReset);
  sCmd.addCommand("calibrate", onCalibrate);
  sCmd.setDefaultHandler(onUnknownCommand);

  EEPROM_readAnything(0, params);
  params.start = false;

  attachInterrupt(digitalPinToInterrupt(startknop), onStart, FALLING);
  attachInterrupt(digitalPinToInterrupt(stopknop), onStop, FALLING);
  
  time = micros();
  prevTime = time;
  
  SerialPort.println("ready");
}

void loop()
{  
  sCmd.readSerial();     // We don't do much, just process serial commands
  
  time = micros();
  
  if (time > prevTime) delta = time - prevTime;
  else delta = 4294967295 - prevTime + time + 1;
    
  if (delta > params.cycleTime)
  {
    prevTime = time;

    while (params.start == true)
    {
      sCmd.readSerial();

    //uitlezen en normaliseren
    for (int i = 0; i < 4; i++)
    {
      value[i] = analogRead(channel[i]);
      
      normalized[i] = value[i] - params.white[i]; //volgens de formule x = ((waarde - laagste waarde) / (hoogste waarde - laagste waarde)) * 100
      normalized[i] /= (params.black[i] - params.white[i]);
      normalized[i] *= 100;
    }

    // interpoleren
    average = ((150 * normalized [3]) + (+50 *   normalized[2]) + (-50 * normalized[1]) + (-150 * normalized[0]));
    average /= (normalized[3] + normalized[2] + normalized[1] + normalized[0]);

    #define setpoint 0

    error = setpoint - average;
    iTerm += (params.ki * error);
    dInput = (average - lastInput);

    /*PID Output berekenen: */
    outputP = params.kp * error;
    /* Delen door 10000, anders moet Ki veel te klein gemaakt worden */
    outputI = iTerm/10000;
    /*Vermenigvuldigen met 10, anders moet Kd veel te groot gemaakt worden */ 
    outputD = (params.kd * dInput)*10;
    output = outputP + outputI - outputD;

    /*Variabelen onthouden voor volgende keer: */
    lastTime = now;
    lastInput = average;

    int leftSpeed;
    int rightSpeed;

    rightSpeed = params.speed + output;
    leftSpeed = params.speed - output;

    //motor links
    if (leftSpeed > 0)
    {
    analogWrite(pinAin1, leftSpeed);
    analogWrite(pinAin2, 0);
    }
    else if (leftSpeed < 0)
    {
    analogWrite(pinAin1, 0);
    analogWrite(pinAin2, (leftSpeed * (-1)));
    }
    if (rightSpeed > 0)
    {
    analogWrite(pinBin1, rightSpeed);
    analogWrite(pinBin2, 0);
    }
    else if (rightSpeed < 0)
    {
    analogWrite(pinBin1, 0);
    analogWrite(pinBin2, (rightSpeed * (-1)));
    }
  }
  
  }
  unsigned long difference = micros() - time;
  if (difference > calculationTime) calculationTime = difference; 

}

void onUnknownCommand(char *command)
{
  SerialPort.print("unknown command: \"");
  SerialPort.print(command);
  SerialPort.println("\"");
}

void onSet()
{
  char* parameter = sCmd.next();
  char* value = sCmd.next();
  
  if (strcmp(parameter, "speed") == 0) params.speed = atoi(value);
  else if (strcmp(parameter, "kp") == 0) params.kp = atof(value);
  else if (strcmp(parameter, "ki") == 0) params.ki = atof(value);
  else if (strcmp(parameter, "kd") == 0) params.kd = atof(value);
  else if (strcmp(parameter, "cycle") == 0) params.cycleTime = atol(value);
  
  EEPROM_writeAnything(0, params);
}

void onStart()
{
  EEPROM_readAnything(0, params);
  if(params.start == false){
    params.start = true;
  }
  else if(params.start == true){
    params.start = false;
    analogWrite(pinAin1, 0);
    analogWrite(pinAin2, 0);
    analogWrite(pinBin1, 0);
    analogWrite(pinBin2, 0);
  }
  EEPROM_writeAnything(0, params);
}

void onStop()
{
  params.start = false;
  analogWrite(pinAin1, 0);
  analogWrite(pinAin2, 0);
  analogWrite(pinBin1, 0);
  analogWrite(pinBin2, 0);
}

void onDebug()
{
  //sensoren uitlezen
  SerialPort.print("Sensoren: ");
  for (int i = 0; i < 4; i++)
  {
    SerialPort.print(analogRead(channel[i]));
    SerialPort.print(" ");
  }
  SerialPort.println("");

  //kalibratie wit
  SerialPort.print("kalibratie wit: ");
  for (int i = 0; i < 4; i++)
  {
    SerialPort.print(params.white[i]);
    SerialPort.print(" ");
  }
  SerialPort.println("");
  
  //kalibratie zwart
  SerialPort.print("kalibratie zwart: ");
  for (int i = 0; i < 4; i++)
  {
    SerialPort.print(params.black[i]);
    SerialPort.print(" ");
  }
  SerialPort.println("");  
  
  //parameters
  SerialPort.print("speed: ");
  SerialPort.println(params.speed);
  SerialPort.print("kp: ");
  SerialPort.println(params.kp); 
  SerialPort.print("ki: ");
  SerialPort.println(params.ki); 
  SerialPort.print("kd: ");
  SerialPort.println(params.kd); 
  //SerialPort.println();
  
  //cycle times
  SerialPort.print("cycle time: ");
  SerialPort.println(params.cycleTime);
  SerialPort.print("calculation time: ");
  SerialPort.println(calculationTime);
  //SerialPort.println();
  calculationTime = 0;  //reset calculation time
  
  //running
  SerialPort.print("running: ");
  SerialPort.println(params.start); 
}

void onReset()
{
  SerialPort.print("resetting parameters... ");
  EEPROM_resetAnything(0, params);
  EEPROM_readAnything(0, params);  
  SerialPort.println("done");
}

void onInterrupt()
{
  params.start = not params.start;
}

void onCalibrate()
{
  char *arg = sCmd.next();
  if (strcmp (arg, "white") == 0) //voert dit uit als commando "calibrate white" is
  {
    for (int i = 0; i < 4; i++) //de array wordt volledig op 1023 gezet, anders wordt er zo meteen niet overschreven
    {
      params.white[i] = 1023;
    }
    
    SerialPort.print("Calibrating white... ");
    for (int i = 0; i < 50; i++) //de sensorwaarde wordt 50 keer uitgelezen voor elke sensor
    {
      for (int j = 0; j < 4; j++)
      {
        int value_white = analogRead(channel[j]); //indien de uitgelezen waarde lager ligt dan de reeds gekalibreerde witte waarde wordt die overschreven
        if (value_white > params.white[j])
        {
          params.white[j] = value_white;
        }
      }
    }
    SerialPort.println(""); //resulataten worden getoond
    for (int i = 0; i < 4; i++)
    {
      SerialPort.print(params.white[i]);
      SerialPort.print(" ");
    }
    SerialPort.println("");
}

  if (strcmp (arg, "black") == 0) //zelfde werking als wit
  {
    SerialPort.print("Calibrating black... ");
    for (int i = 0; i < 50 ; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        int value_black = analogRead(channel[j]);
        if (value_black < params.black[j])
        {
          params.black[j] = value_black;
        }
      }
    }
    SerialPort.println("");
    for (int i = 0; i < 4; i++)
    {
      SerialPort.print(params.black[i]);
      SerialPort.print(" ");
    }
    SerialPort.println("");
  }
  EEPROM_writeAnything(0,params);
}
