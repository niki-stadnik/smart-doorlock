#include "WString.h"
#ifndef SudoJSON_h
#define SudoJSON_h
#include <Arduino.h> 

class SudoJSON{
  public:
    SudoJSON();
    SudoJSON(String input);
    void check();
    void addPair(const char name[], const char value[]);
    void addPair(const char name[], float value);
    void addPair(const char name[], int value);
    void addPair(const char name[], boolean value);
    char* retrive();
    void startArrayE(const char value[]);
    void addArrayE(int value);
    char* retriveArray();
    char* getPairS(const char value[]);
    float getPairF(const char value[]);
    int getPairI(const char value[]);
    boolean getPairB(const char value[]);
  private:
    char* find(const char value[]);
    char message[500] = {'{'};
    char set[40][20];
    int chars[40];
    int countAr = 0;
};

#endif