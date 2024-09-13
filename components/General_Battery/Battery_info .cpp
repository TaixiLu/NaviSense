#include "Battery_info.h"

static vector<float> General_lipo_graph =
    {2.7, 3.57, 3.66, 3.67, 3.69, 3.72, 3.73, 3.75, 3.77, 3.79,
     3.81, 3.82, 3.85, 3.87, 3.9, 3.94, 4.0, 4.05, 4.09, 4.15};
static vector<float> General_lipo_load_graph =
    {2.78, 3.42, 3.50, 3.53, 3.55, 3.57, 3.59, 3.61, 3.62, 3.64,
     3.66, 3.68, 3.71, 3.74, 3.78, 3.82, 3.87, 3.90, 3.94, 4.03};
static vector<float> General_lipo_charge_graph =
    {2.94, 3.68, 3.78, 3.81, 3.84, 3.87, 3.89, 3.90, 3.91, 3.92,
     3.93, 3.95, 3.97, 4.00, 4.03, 4.05, 4.07, 4.12, 4.17, 4.20};
Battery_info General_lipo{General_lipo_graph, General_lipo_load_graph,
                          General_lipo_charge_graph, 1, 1, -1, 0.5};

static vector<float> General_21700_graph =
    {2.7, 3.26, 3.27, 3.34, 3.39, 3.45, 3.48, 3.51, 3.54, 3.58,
     3.63, 3.7, 3.75, 3.79, 3.82, 3.89, 3.94, 3.95, 4.01, 4.14};
static vector<float> General_21700_load_graph =
    {2.70, 2.96, 3.05, 3.11, 3.15, 3.19, 3.22, 3.25, 3.30, 3.35,
     3.41, 3.46, 3.52, 3.57, 3.62, 3.67, 3.69, 3.71, 3.71, 4.16};
static vector<float> General_21700_charge_graph =
    {3.29, 3.60, 3.64, 3.68, 3.73, 3.76, 3.79, 3.81, 3.84, 3.87,
     3.90, 3.94, 3.98, 4.02, 4.06, 4.09, 4.12, 4.14, 4.18, 4.21};
Battery_info General_21700{General_21700_graph, General_21700_load_graph,
                           General_21700_charge_graph, 1, 1, -1, 0.5};

static vector<float> P42A_graph =
    {2.7, 3.11, 3.31, 3.41, 3.45, 3.51, 3.56, 3.6, 3.63, 3.67,
     3.72, 3.77, 3.81, 3.86, 3.9, 3.96, 4.01, 4.05, 4.07, 4.15};
static vector<float> P42A_load_graph =
    {2.70, 3.02, 3.16, 3.23, 3.29, 3.33, 3.37, 3.40, 3.44, 3.48,
     3.53, 3.57, 3.62, 3.66, 3.70, 3.73, 3.78, 3.84, 3.88, 4.07};
static vector<float> P42A_charge_graph =
    {2.87, 3.49, 3.56, 3.61, 3.65, 3.69, 3.74, 3.76, 3.79, 3.83,
     3.88, 3.92, 3.97, 4.00, 4.04, 4.08, 4.13, 4.17, 4.19, 4.20};
Battery_info Moli_P42A{P42A_graph, P42A_load_graph,
                       P42A_charge_graph, 1, 1, -1, 2.5 / 4.2};

static vector<float> P45B_graph =
    {2.7, 3.02, 3.17, 3.3, 3.4, 3.47, 3.51, 3.57, 3.62, 3.67,
     3.72, 3.77, 3.81, 3.86, 3.91, 3.97, 4.03, 4.05, 4.08, 4.16};
static vector<float> P45B_load_graph =
    {2.7, 3.01, 3.15, 3.24, 3.31, 3.36, 3.4, 3.44, 3.48, 3.52,
     3.58, 3.63, 3.67, 3.71, 3.75, 3.8, 3.85, 3.9, 3.94, 4.1};
static vector<float> P45B_charge_graph =
    {2.88, 3.46, 3.51, 3.55, 3.59, 3.63, 3.68, 3.71, 3.74, 3.79,
     3.84, 3.89, 3.93, 3.97, 4.02, 4.07, 4.12, 4.16, 4.18, 4.21};
Battery_info Moli_P45B{P45B_graph, P45B_load_graph,
                       P45B_charge_graph, 1, 1, -1, 2.5 / 4.5};

static vector<float> Samsung_40T_graph =
    {2.7, 3.12, 3.33, 3.41, 3.46, 3.52, 3.56, 3.6, 3.63, 3.68,
     3.72, 3.77, 3.82, 3.86, 3.91, 3.98, 4.03, 4.05, 4.07, 4.16};
static vector<float> Samsung_40T_load_graph =
    {2.7, 3.04, 3.19, 3.26, 3.31, 3.36, 3.39, 3.43, 3.47, 3.52,
     3.57, 3.62, 3.67, 3.71, 3.75, 3.81, 3.85, 3.88, 3.9, 4.03};
static vector<float> Samsung_40T_charge_graph =
    {2.94, 3.48, 3.55, 3.6, 3.65, 3.69, 3.72, 3.76, 3.8, 3.84,
     3.89, 3.93, 3.97, 4.01, 4.05, 4.1, 4.15, 4.18, 4.19, 4.21};
Battery_info Samsung_40T{Samsung_40T_graph, Samsung_40T_load_graph,
                         Samsung_40T_charge_graph, 1, 1, -1, 2.5 / 4};

static vector<float> Samsung_50S_graph =
    {2.7, 3.01, 3.16, 3.29, 3.4, 3.47, 3.53, 3.59, 3.63, 3.68,
     3.73, 3.78, 3.82, 3.86, 3.91, 3.98, 4.03, 4.05, 4.06, 4.15};
static vector<float> Samsung_50S_load_graph =
    {2.7, 2.9, 3.03, 3.12, 3.19, 3.24, 3.29, 3.33, 3.37, 3.41,
     3.45, 3.5, 3.54, 3.58, 3.62, 3.65, 3.69, 3.74, 3.8, 3.96};
static vector<float> Samsung_50S_charge_graph =
    {2.94, 3.51, 3.57, 3.6, 3.63, 3.67, 3.71, 3.75, 3.79, 3.82,
     3.86, 3.9, 3.95, 3.99, 4.03, 4.06, 4.1, 4.15, 4.19, 4.21};
Battery_info Samsung_50S{Samsung_50S_graph, Samsung_50S_load_graph,
                         Samsung_50S_charge_graph, 1, 1, -1, 0.5};
