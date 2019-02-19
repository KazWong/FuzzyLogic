#include "FuzzyLogic.h"

#include "ikaLib.h"

#include "cv.h"
#include "highgui.h"

CvRect box;
int cvVideoWidth = 0;
int cvVideoHeight = 0;

void print(void) {
	printf("+=======================================+\n");
	printf("|                                       |\n");
	printf("|  \\(^o^)/     Fuzzy Logic     \\(^o^)/  |\n");
	printf("|                                       |\n");
	printf("+=======================================+\n");
	printf("\n\n");
}

float DOM(float m, float er, float c, float shift, float limit) {
	float x=0.0;

	x = (float)( range((float)( -abs((float)(er+shift)) ),(float)limit));
	return (x==limit)? (0):(abs(x*m+c));
}

float FuzzyLogic(float error, double error_dot, double error_region, double error_dot_region) {
	float FC_error[3], FC_error_dot[3];
	float fl_matrix[9];
	float negative_strength, zero_strength, positive_strength;
	float output;

	int e_r = (int)ceil(error_region);
	int e_d_r = (int)ceil(error_dot_region);

	//print();
	//system("cls");

	/*printf("\t\t   -- Air Control System --\n\n\n");
	printf("================================================================================\n");
	printf("Input:\n");
	printf("\t\tPositive\tZero\t\tNegative\n");
	printf("Error\t\tToo cold\tJust right\tToo hot\n");
	printf("Error_Dot\tGetting hotter\tNot changing\tGetting colder\n");
	printf("Output:\n");
	printf("Positive\tHeating\n");
	printf("negative\tCooling\n");
	printf("================================================================================\n");
	printf("\n");*/

	for (int i=-e_r; i<=e_r; i+=e_r ) {
		FC_error[i/e_r+1] = DOM((float)(1/error_region), (float)error, (float)1, (float)i, error_region);
	}
	for (int i=-e_d_r; i<=e_d_r; i+=e_d_r ) {
		FC_error_dot[i/e_d_r+1] = DOM((float)(1/error_dot_region), (float)error_dot, (float)1, (float)i, error_dot_region);
	}

	/*printf("\t\tError\tError_dot\n");
	printf("positive\t%.2f\t%.2f\n", FC_error[positive], FC_error_dot[positive]);
	printf("zero\t\t%.2f\t%.2f\n", FC_error[zero], FC_error_dot[zero]);
	printf("negative\t%.2f\t%.2f\n", FC_error[negative], FC_error_dot[negative]);
	printf("\n");*/

	for (int i=positive; i<=negative; i++) {
		for (int j=positive; j<=negative; j++) {
			fl_matrix[i*3+j] = FL(FC_error[j], FC_error_dot[i]);
			//printf( "%.2f\t", fl_matrix[i*3+j] );
		}
		//printf("\n");
	}
	//printf("\n");

	negative_strength = (float)sqrt((float)sqr(fl_matrix[1])+sqr(fl_matrix[2])+sqr(fl_matrix[5])+sqr(fl_matrix[8]));
	zero_strength = (float)(fl_matrix[4]);
	positive_strength = (float)(sqrt((float)sqr(fl_matrix[0])+sqr(fl_matrix[3])+sqr(fl_matrix[6])+sqr(fl_matrix[7])));

	if ( (negative_strength == 0) && (zero_strength == 0) && (positive_strength == 0) )
		return 0.0;

	/*printf("Strength:\n");
	printf("positive\t%.3f\n", positive_strength);
	printf("zero\t\t%.3f\n", zero_strength);
	printf("negative\t%.3f\n", negative_strength);
	printf("\n");*/

	output = ( ((-100*negative_strength)+(0*zero_strength)+(100*positive_strength))/(positive_strength+zero_strength+negative_strength) )/100;

	//printf("Fuzzy Output : %.4f%%\n", output*100);
	//printf("\n\n");
	return output;
}

static int smooth_count=0;
static CvPoint2D32f prev_error = cvPoint2D32f(0,0);
static float prev_dis_error = 0.0;
const int FACESIZE = 50;
const int FRANGE = 140;
const float SPEED = 0.25f;
const float LIMIT = 0.4f;
const float FACTOR = 10.0f;

float roll_result = 0;
float pitch_result = 0;
float gaz_result = 0;
float yaw_result = 0;

extern "C" float FuzzyControl(CvRect Rect, CvPoint2D32f stay_point, double dt)  {
	CvPoint2D32f error = cvPoint2D32f(0,0);
	double dis_error = 0, dis_error_dot = 0;
	int hovering = 0;
	float pitch = 0, roll = 0;
	float gaz = 0;
	float yaw = 0;

	if (Rect.x == -1 || Rect.y == -1) {
		if (smooth_count++>1) {
			roll_result = roll_result - roll_result/5;
			pitch_result = 0;
			gaz_result = 0;
			yaw_result = 0;

			smooth_count=0;
		}
	} else {
		double target=0.0;
		double error_dot=0.0;

		hovering = 1;

		target = (float)(Rect.width/2)+(float)(Rect.x);
		error.x = (float)( target - stay_point.x );
		target = (float)(Rect.height/2)+(float)(Rect.y);
		error.y = (float)( target - stay_point.y );

		error_dot = -(double)( (error.x - prev_error.x)*FACTOR/dt );
		roll = FuzzyLogic(error.x, error_dot, (double)(cvVideoWidth*LIMIT), (double)(cvVideoWidth*LIMIT/dt));
		error_dot = -(double)( (error.y - prev_error.y)*FACTOR/dt );
		gaz = FuzzyLogic(error.y, error_dot, (double)(cvVideoHeight*(LIMIT/2)), (double)(cvVideoHeight*(LIMIT/2)/dt));

		prev_error.x = error.x;
		prev_error.y = error.y;


		dis_error = ( Rect.width - FACESIZE );
		dis_error_dot = -(dis_error - prev_dis_error)/dt;

		pitch = FuzzyLogic(dis_error, dis_error_dot*FACTOR, (float)(FRANGE), (float)((FRANGE)/dt) );

		prev_dis_error = dis_error;

			roll_result = (roll*SPEED);
			pitch_result = (pitch*SPEED);
			gaz_result = (-gaz*SPEED);
			yaw_result = (yaw*SPEED);

	}

	if (roll_result>1)
		roll_result=1;
	else if (roll_result<-1)
		roll_result=-1;

	if (pitch_result>1)
		pitch_result=1;
	else if (pitch_result<-1)
		pitch_result=-1;

	if (gaz_result>1)
		gaz_result=1;
	else if (gaz_result<-1)
		gaz_result=-1;

	if (yaw_result>1)
		yaw_result=1;
	else if (yaw_result<-1)
		yaw_result=-1;


	printf("%.2f, %.2f ,%.2f ,%.2f\n", roll_result, pitch_result, gaz_result, yaw_result);
	//ardrone_at_set_progress_cmd( 1, roll_result, pitch_result, gaz_result, 0/*yaw_result*/);

	return 1;
}

double tt=-1;
void OnMouse(int events, int x, int y, int flags, void* param) {
	//printf("%i, %i, %i, %i\n", events, x, y, flags);
	
	
	
	IplImage **img = (IplImage **) param;

	switch(events) {
		case CV_EVENT_MOUSEMOVE : {
				//box.width = x-box.x;
				//box.height = y-box.y;
				box.x = x;
				box.y = y;

				if (tt!=-1) {
					tt = (double)cvGetTickCount() - tt;
					printf("dt : %.4f\n", (double)(tt/(cvGetTickFrequency()*1000*1000)));
					FuzzyControl(box, cvPoint2D32f(cvRound(cvVideoWidth/2), cvRound(cvVideoHeight/2)), (double)(tt/(cvGetTickFrequency()*1000*1000)));
				}
				tt = (double)cvGetTickCount();
			
								  }
								  break;
		/*case CV_EVENT_LBUTTONDOWN : {
			drawing_box = true;
			box = cvRect(x, y, 0, 0);
									}
									break;
		case CV_EVENT_LBUTTONUP : {
			drawing_box = false;
			if (box.width<0) {
				box.x+=box.width;
				box.width *=-1;
			}
			if (box.height<0) {
				box.y+=box.height;
				box.height *=-1;
			}
			if (*img && box.width>0 && box.height>0 && box.x>0 && box.y>0) {
				FuzzyControl(box, cvPoint(cvRound(cvVideoWidth), cvRound(cvVideoHeight)), (double)(tt/(cvGetTickFrequency()*1000*1000)));
			}
								  }
								  break;*/
	}
}

void main(void) {
	IplImage *frameImg = NULL;
	CvCapture *capture;
	CvHaarClassifierCascade *cascade;
	CvRect prevrect = cvRect(0,0,0,0);
	haar_detected *face_detect;

	box = cvRect(-1, -1, 50, 50);

	cvNamedWindow("FuzzyLogic", 1);
	//cvSetMouseCallback("FuzzyLogic", OnMouse, (void *)&frameImg);

	capture = cvCaptureFromCAM(0); 
	cascade = ikaCascadeRead("haarcascade_frontalface_alt2.xml");

	while (true) {
		frameImg = cvQueryFrame(capture);
		//system("cls");
		face_detect = ikaDetectObject(frameImg, prevrect, cascade);

		cvVideoWidth = frameImg->width;
		cvVideoHeight = frameImg->height;
		if (face_detect) {
			if (tt!=-1) {
					tt = (double)cvGetTickCount() - tt;
					FuzzyControl(*face_detect->detectedRect, 
						cvPoint2D32f(cvVideoWidth/2, cvVideoHeight/2), 
						(double)(tt/(cvGetTickFrequency()*1000*1000)));
			}
			tt = (double)cvGetTickCount();
		}

		IplImage *cpimg = cvCloneImage(frameImg);

		if (face_detect)
		cvDrawCircle(cpimg, 
			cvPoint(face_detect->detectedRect->x+cvRound(face_detect->detectedRect->width/2),face_detect->detectedRect->y+cvRound(face_detect->detectedRect->height/2)),  
			cvRound(face_detect->detectedRect->width/2), CV_RGB(255, 0, 0));

		cvShowImage("FuzzyLogic", cpimg); 

		if (cvWaitKey(10)==27)
			break;
	}

	cvWaitKey(0);

	cvDestroyWindow("FuzzyLogic");
}
