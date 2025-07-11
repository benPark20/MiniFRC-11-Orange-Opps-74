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
constexpr int MID_RIGHT = 9;     //+
constexpr int MID_LEFT = 8;      //-
constexpr int L_PRESS = 10;      //L Stick
constexpr int R_PRESS = 11;      //R Stick
constexpr int D_UP = 12;
constexpr int D_DOWN = 13;
constexpr int D_LEFT = 14;
constexpr int D_RIGHT = 15;
//Servo Angles
//max 210
constexpr float positions[][2] = {
    {50.0, 205.0},     // intake angle
    {52.6, 113.0},      // l1
    {31.3, 93.4},      // l2
    {40.1, 63.6},      // l3
    {58.3, 23.5},      // l4
    {46.5, 87.38},      // algea l2
    {54.5, 54.9},      // algae l3
    {99.0, 100.8},      // score processor
    {73.7, 0.0},      // score barge
    {50.0, 85.0},   // stow angle
};
//claw intake/stow
constexpr float stow = 0.0;
//claw grab Algea
constexpr float grab = 85.0;
//
#endif