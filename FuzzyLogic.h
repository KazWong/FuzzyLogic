#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <conio.h>

using namespace std;

//#define equal(a,b)	((a)==(b))? (a):(0)
//#define larger(a,b)	((a)>(b))? (a):(0)
//#define	smaller(a,b)	((a)<(b))?	(a):(0)
#define sqr(a)	pow(a, 2)
#define	FL(a,b)	((a)&&(b))?	(min(a,b)):(0)
#define range(a,b)	((abs(b)>=a)&&(a>=-abs(b)))? (a):(b)

enum { positive=0, zero, negative};

//const float dt = 0.4;

float FuzzyLogic(float error, double error_dot, double error_region, double error_dot_region);