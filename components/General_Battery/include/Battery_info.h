#pragma once
#include <stdio.h>
#include <vector>
using namespace std;

typedef struct
{
  vector<float> volt_perc_graph; // single cell voltage disttribut evenly from 0% to 100%, eg. for 10 perc per data, the size = (100/10)+1
  vector<float> volt_perc_graph_load;
  vector<float> volt_perc_graph_charge;
  uint16_t capacity = 1; // mah
  uint8_t serial = 1;
  float Cnum_load_graph;   // C Negative
  float Cnum_charge_graph; // C Positive
} Battery_info;

extern Battery_info General_lipo;
extern Battery_info General_21700;
extern Battery_info Moli_P42A;
extern Battery_info Moli_P45B;
extern Battery_info Samsung_40T;
extern Battery_info Samsung_50S;