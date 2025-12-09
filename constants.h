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
    {55.0, 200.0},     // intake angle  Stage 1: plus is more forward towards numbers
    {52.6, 126.0},     // l1            Stage 2: plus is further down towards intaking
    {49.3, 100.4},     // l2
    {55.1, 63.6},      // l3
    {59.0, 22.7},      // l4
    {46.5, 87.38},     // algea l2
    {54.5, 54.9},      // algae l3
    {99.0, 100.8},     // score processor
    {73.7, 0.0},       // score barge
    {40.0, 160.0},     // stow angle
};
#endif