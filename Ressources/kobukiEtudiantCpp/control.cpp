/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "tustin.hpp"
#include "control.hpp"
#define PI  3.14159265358979323846 /* pi */

void initControl(ControlStruct &control) {
    // constantes
    control.R = 0.07 / 2; // rayon des roues en m
    control.Ly = 0.23; // largeur essieu en m ( dist entre milieu des roues )
    double reduction = 6545.0 / 132.0; // rapport de reduction
    double tickByTour = 52 * reduction; // nb pas codeur / tour de roue
    control.encToRad = tickByTour * 2 * PI; // angles roues  gauche, droite= control.encToRad *  left_encoder , control.encToRad *  right_encoder
    //valeurs initiales des signaux 
    control.ref_vx=0; // ref_vx from key board
    control.vx = 0;   // vx applied to kobuki
    control.ref_wz;   // ref_wz from keyboard
    control.wz = 0;   // wz applied to kobuki
    control.measure = 0;
    // init lowpass filter example : 1/ (1+0.1.p) = ( n0p+n1p.p)/(d0p +d1p.p)  , with sample time Te=0.02 s
    bool ok = initFilter(control.lowpassExample, 1, 0, 1, 0.1, 0.02);

}

void oneStepControl(ControlStruct &control) {
    // you have to compute control.vx and and control.wz to follow a line, corresponding to control.measure=0;
    // ControlStruct is defined in control.hpp file and can be modified
    double vxFiltered = oneStepFilter(control.lowpassExample, control.ref_vx); // compute the output of the lowpass filter with input ref_vx  
    control.vx=control.ref_vx; // vx applied to kobuki <- ref_vx from keyboard
    control.wz=control.ref_wz; // wz applied to kobuki <- ref_wz from keyboard 
    float thetag = control.left_encoder*control.encToRad;
    control.wz = 0;
}

void endControl(ControlStruct &control) {
    control.vx = 0;
    control.wz = 0;
}
/*------------------------------------------------------------------------------
 Key board reactions : increment or decrement speeds
 ------------------------------------------------------------------------------*/
#define SPEED_MAX 1 /* max speed in m/s */
bool setSpeedRef(ControlStruct &control, double ref_vx, double ref_wz) {
    double left_speed = ref_vx - ref_wz * control.Ly / 2;
    if (left_speed<-SPEED_MAX) return false;
    if (left_speed > SPEED_MAX) return false;
    double right_speed = ref_vx + ref_wz * control.Ly / 2;
    if (right_speed<-SPEED_MAX) return false;
    if (right_speed > SPEED_MAX) return false;
    control.ref_vx =ref_vx;
    control.ref_wz= ref_wz;
    return true;
}
#define STEP_WZ ( 0.1  * SPEED_MAX / control.Ly) /* rotation speed step */
#define STEP_VX ( 0.1  * SPEED_MAX)       /* linear speed step */
void handleKeyboard(ControlStruct &control, char key) {
    bool ok;
    switch (key) {
        case 68: // kobuki_msgs::KeyboardInput::KEYCODE_LEFT:
        {
            ok=setSpeedRef( control,control.ref_vx,control.ref_wz - STEP_WZ);
            break;
        }
        case 67: // kobuki_msgs::KeyboardInput::KEYCODE_RIGHT:
        {
            ok=setSpeedRef( control,control.ref_vx,control.ref_wz + STEP_WZ);
            break;
        }
        case 65: // kobuki_msgs::KeyboardInput::KEYCODE_UP:
        {
            ok=setSpeedRef( control,control.ref_vx+STEP_VX,control.ref_wz);
            break;
        }
        case 66: // kobuki_msgs::KeyboardInput::KEYCODE_DOWN:
        {
            ok=setSpeedRef( control,control.ref_vx-STEP_VX,control.ref_wz);
            break;
        }
        case 32: // kobuki_msgs::KeyboardInput::KEYCODE_SPACE:
        {
            // resetVelocity();
            ok=setSpeedRef( control,0,0);
            control.vx = 0;
            control.wz = 0;
            break;
        }
    }
}