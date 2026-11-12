/*
Все константы
*/
#pragma once

//robot movement
// MAX_VEL и MAX_ACC можно переопределить при сборке (-DMAX_VEL=... -DMAX_ACC=...), например под роботов из логов игр
#ifndef MAX_VEL
#define MAX_VEL 1000
#endif
#ifndef MAX_ACC
#define MAX_ACC 2000
#endif
#define SAFE_DIST 20
//idk
#define MAX_ROBOT_COUNT 16
#define N_SOLVER_PAIRS 8
#define K_INTERSECT 10
//geometry
#define ROBOT_R 100.0
#define BALL_R 40.0
#define FIELD_DX 9000
#define FIELD_DY 6000
#define ZONE_DX 1000
#define ZONE_DY 2000
#define GOAL_Y 1000
#define POLARITY 1
//draw info
#define PIXELS_X 900
#define PIXELS_Y 600
#define VELOCITY_K 0.3
#define LINE_WIDTH 20
//field info
#define ALL_TEAMS 0
#define ALLY 1
#define ENEMY 2
//some math constants
#define EPSILON 1e-8