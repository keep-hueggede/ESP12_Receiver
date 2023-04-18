#ifndef COMSTRUCT_H_
#define COMSTRUCT_H_
/**
* Type definition for esp-now communication (use in sender and receiver)
*/
typedef struct{
  char id[50]; //sensor id
  char key[20]; //val key (e.g. temperature, pressure)
  char sensorType[50]; //which type of sensor
  double dValue; //value in format double
  char sValue[50]; //value in format string
  bool type = 0; //0 => double; 1 => string
} comStruct;

#endif