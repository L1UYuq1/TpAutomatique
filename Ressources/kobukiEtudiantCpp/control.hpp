/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   control.hpp
 * Author: sygorra
 *
 * Created on 1 février 2024, 08:19
 */

#ifndef CONTROL_HPP
#define CONTROL_HPP
#include <stdlib.h>
#include "tustin.hpp" 
typedef struct {
    float R; // rayon des roues en m
    float Ly; // largeur essieu en m;
    double encToRad; // angle roue (rad ) / (encoder value )
    double measure, vx, wz,ref_vx,ref_wz;
    int32_t left_encoder, right_encoder;
    int left_pwm, right_pwm; // pwm of motors, between -128 and 127
    Filter lowpassExample; // an example to understand how to use the tustin approx of continuous filters
    
} ControlStruct;
void initControl(ControlStruct &control);
void oneStepControl(ControlStruct &control);
void endControl(ControlStruct &control);
void handleKeyboard(ControlStruct &control,char key);
#endif /* CONTROL_HPP */

