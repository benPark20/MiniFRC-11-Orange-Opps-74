#ifndef CONSTANTS
#define CONSTANTS

constexpr int x = 0; //placeholder

//Buttons
constexpr int BUTTON_BOTTOM = 0; //B
constexpr int BUTTON_RIGHT = 1;  //A
constexpr int BUTTON_LEFT = 2;   //Y
constexpr int BUTTON_TOP = 3;    //X
constexpr int LEFT_BUMPER = 4;
constexpr int RIGHT_BUMPER = 5;
constexpr int LEFT_TRIGGER = 6;
constexpr int RIGHT_TRIGGER = 7;
constexpr int MID_RIGHT = 8;     //+
constexpr int MID_LEFT = 9;      //-
constexpr int L_PRESS = 10;      //L Stick
constexpr int R_PRESS = 11;      //R Stick
constexpr int D_UP = 12;
constexpr int D_DOWN = 13;
constexpr int D_LEFT = 14;
constexpr int D_RIGHT = 15;
//Servo Angles
constexpr int positions[][2] = {
    {45, 0},     // intake angle
    {x, x},      // l1
    {x, x},      // l2
    {x, x},      // l3
    {x, x},      // l4
    {x, x},      // algea l2
    {x, x},      // algae l3
    {x, x},      // score processor
    {x, x},      // score barge
    {45, 135},   // stow angle
};
//claw intake/stow
constexpr float stow = x;
//claw grab Algea
constexpr float grab = x;
//
#endif