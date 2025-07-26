#include "SudoJSON.h"
#include <Arduino.h> 


SudoJSON::SudoJSON(){ 
}
SudoJSON::SudoJSON(String string){
  int len = string.length() + 1;
  char input[len];
  string.toCharArray(input, len);

  int countCr = 0;
  int wordStart = 0;
  boolean wordFound = false;

  for(int i = 0; i < len; i++){
    if (input[i] == ':'){
      if (input[i+1] != '\\'){
        wordFound = true;
        wordStart = i + 1;
      }
      continue;
    }
    if (wordFound){
      if (input[i] == ',' || input[i] == '}'){
        set[countAr][countCr] = '\0';
        //chars[countAr] = countCr + 2;
        wordFound = false;
        countAr++;
        countCr = 0;
        continue;
      }
    }
    if (input[i] == '\\'){
      if (wordFound){
        set[countAr][countCr] = '\0';
        //chars[countAr] = countCr + 2;
        wordFound = false;
        countAr++;
        countCr = 0;
      } else{
        wordFound = true;
        wordStart = i + 2;
      }
      continue;
    }
    if (wordFound && wordStart <= i){
      set[countAr][countCr] = input[i];
      countCr++;
      chars[countAr] = countCr;
    }
  }
}



char* SudoJSON::find(const char value[]){
  int siz = strlen(value);
  boolean flag = false;
  for (int i = 0; i < countAr; i = i + 2){
    if (siz != chars[i]) continue;
    flag = true;
    for (int j = 0; j < siz; j++){
      if (value[j] != set[i][j]) {
        flag = false;
        break;
      }
    }
    if(flag) return set[i + 1];
  }
  return "nope";
}
char* SudoJSON::getPairS(const char value[]){
  char* str = find(value);
  return str;
}
float SudoJSON::getPairF(const char value[]){
  String str = find(value);
  float ff = str.toFloat();
  return ff;
}
int SudoJSON::getPairI(const char value[]){
  String str = find(value);
  int i = str.toInt();
  return i;
}
boolean SudoJSON::getPairB(const char value[]){
  char * str = find(value);
  boolean state = false;
  if(0 == strcmp(str, "true")) state = true;
  return state;
}

void SudoJSON::check(){
  int length = strlen(message);
  if(length > 5){
    message[length - 1] = ',';
  }
}
void SudoJSON::addPair(const char name[], const char value[]){
  check();
  strcat(message, "\\\"");
  strcat(message, name);
  strcat(message, "\\\":\\\"");
  strcat(message, value);
  strcat(message, "\\\"}");
}
void SudoJSON::addPair(const char name[], int value){
  check();
  char myCharArray[10];
  itoa(value, myCharArray, 10);
  strcat(message, "\\\"");
  strcat(message, name);
  strcat(message, "\\\":\\\"");
  strcat(message, myCharArray);
  strcat(message, "\\\"}");
}
void SudoJSON::addPair(const char name[], float value){
  check();
  char myCharArray[10];
  dtostrf(value, 5, 2, myCharArray);
  strcat(message, "\\\"");
  strcat(message, name);
  strcat(message, "\\\":\\\"");
  strcat(message, myCharArray);
  strcat(message, "\\\"}");
}
void SudoJSON::addPair(const char name[], boolean value){
  check();
  strcat(message, "\\\"");
  strcat(message, name);
  strcat(message, "\\\":\\\"");
  if(value) strcat(message, "true");
  else strcat(message, "false");
  strcat(message, "\\\"}");
}
char* SudoJSON::retrive(){
  return message;
}


void SudoJSON::startArrayE(const char value[]){
  strcat(message, "\\\"");
  strcat(message, value);
  strcat(message, "\\\":[");
}
void SudoJSON::addArrayE(int value){
  char myCharArray[10];
  itoa(value, myCharArray, 10);
  strcat(message, myCharArray);
  strcat(message, ",");
}
char* SudoJSON::retriveArray(){
  int len = strlen(message);
  message[len - 1] = '\0';
  strcat(message, "]}"); 
  return message;
}