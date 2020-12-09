/*
 * pid_controller.c
 *
 *  Created on: Sep 17, 2020
 *      Author: son
 */

#include "stdint.h"
#include "pid_controller.h"

#include "math.h"

float PID_out_max=599.0f;
float PID_out_min=-599.0f;

int dir_= 0x00;

float _PID_Kp[10]={5.0, 5.0,  // > 10.0f
				   10.0, 10.0,  // <=10.0f
				   20.0, 20.0,  // <=7.0f
				   30.0, 30.0,  // <=4.5f
				   35.0, 35.0}; //  <=2.0f
float _PID_Ki[10]={350.0, 350.0,
				   350.0, 350.0,
				   300.0, 300.0,
				   200.0, 200.0,
				   150.0, 150.0};
float _PID_Kd[10]={0.0, 0.0,
			  	   0.0, 0.0,
				   0.0, 0.0,
				   0.0, 0.0,
				   0.0, 0.0};

float PID_T = 0.005f;


//void PID_Init(float *Kp, float *Ki, float *Kd, float Ts) {
//	for (int i = 0; i < 2; i++) {
//		PID_Kp[i] = *(Kp+i);
//		PID_Ki[i] = *(Ki+i);
//		PID_Kd[i] = *(Kd+i);
//	}
//	PID_T = Ts/1000.0f;
//}

/*
 * PID_in is array 2 value (target L, target R)
 * PID_current is array 2 value (velo L, velo R)
 * return *PID_out is velo of 2 motors Left and Right

*/
float * PID_Calculate(float *_PID_in, int PID_dir, volatile int16_t *_PID_current){
//	float error, PID_P, PID_I, PID_D;

//	PID_Test[4] = *(_PID_in+0);
//	PID_Test[5] = *(_PID_in+1);

/*
	if (!(PID_dir&0x01)) PID_in[0]=*(_PID_in+0)*(-1);
	else 				 PID_in[0]=*(_PID_in+0);

	if (!(PID_dir&0x02)) PID_in[1]=*(_PID_in+1)*(-1);
	else 				 PID_in[1]=*(_PID_in+1);
*/


/*
	for (int i=0;i<2; i++){

		PID_Test[i] = *(PID_current+i);

		*(PID_in+i)*=updateVel;

		PID_Test[i+2]=*(PID_in+i);

		error = -(*(PID_in+i)-*(PID_current+i))*enc[i*2+1];
		PID_P[i] = (float)(PID_Kp[i]*(error-PID_pre_err[i]));
		PID_I[i] = (float)(0.5F*PID_Ki[i]*0.005f*(error+PID_pre_err[i]));
		PID_D[i] = (float)(PID_Kd[i]*200.0f*(error-2.0f*PID_pre_err[i]+PID_ppre_err[i]));

		PID_out[i] = (PID_P[i] + PID_I[i] + PID_D[i]) + PID_out[i];

		PID_Test[i+6]=PID_out[i];

		if (PID_out[i]>PID_out_max){
			PID_out[i]=PID_out_max;
		}
		else if (PID_out[i]<PID_out_min){
			PID_out[i]=PID_out_min;
		}

		if (PID_out[i]>=0){
			dir_=dir_|(i+1);
			PID_Test[i+8]=100;
		}
		else dir_=dir_&(2-i);
		PID_out[i]=fabs(PID_out[i]);

		PID1[i] = PID_out[i];
		PID_ppre_err[i]=PID_pre_err[i];
		PID_pre_err[i]=error;


	}
//	dir_=3;
	PID_out[2]=(float)dir_;
*/


  	for (int i=0;i<2; i++){

  		if (!(PID_dir&(i+1))) PID_in[i]=*(_PID_in+i)*(-1);
  		else 				 PID_in[i]=*(_PID_in+i);

  		PID_Test[i+2]=PID_in[i];

  		PID_in[i]*=updateVel;

  		PID_current[i] = *(_PID_current+i);

  		if(fabs(PID_in[i])>10.0f){ // > 10
  			PID_Kp[i]=_PID_Kp[i];
  			PID_Ki[i]=_PID_Ki[i];
  			PID_Kd[i]=_PID_Kd[i];
  		}
  		if (fabs(PID_in[i])<=10.0f){ // 10 - 7
  			PID_Kp[i]=_PID_Kp[i+2];
  			PID_Ki[i]=_PID_Ki[i+2];
  			PID_Kd[i]=_PID_Kd[i+2];
  		}
  		if (fabs(PID_in[i])<=7.0f){ // 7 - 4.5
  			PID_Kp[i]=_PID_Kp[i+4];
  			PID_Ki[i]=_PID_Ki[i+4];
  			PID_Kd[i]=_PID_Kd[i+4];
  		}
  		if (fabs(PID_in[i])<=4.5f){ // 4.5 - 2
  			PID_Kp[i]=_PID_Kp[i+6];
  			PID_Ki[i]=_PID_Ki[i+6];
  			PID_Kd[i]=_PID_Kd[i+6];
  		}
  		if (fabs(PID_in[i])<=2.0f){ // 0 - 2.0
  			PID_Kp[i]=_PID_Kp[i+8];
  			PID_Ki[i]=_PID_Ki[i+8];
  			PID_Kd[i]=_PID_Kd[i+8];
  		}
  		if (fabs(PID_in[i])>20.0f){ // 0 - 2.0
  			PID_Kp[i]=0.0;
  			PID_Ki[i]=0.0;
  			PID_Kd[i]=0.0;
  		}

//  		if(((PID_in_pre[i]==0) & (!PID_in[i]==0)) | (PID_in_pre[i]*PID_in[i] <= 0)){
  		if(PID_in_pre[i]*PID_in[i] <= 0){
  			PID_out[i]=265*PID_in[i]/fabs(PID_in[i]);
  		}

  		PID_in_pre[i] = PID_in[i];


//		PID_Test[i] = PID_current[i];
//		PID_Test[i+2]=PID_in[i];

		E0[i] = (PID_in[i]-PID_current[i]);

		A0[i] = PID_Kp[i] + PID_Ki[i]/400.0F + PID_Kd[i]*200.0F;
		A1[i] = -PID_Kp[i] + PID_Ki[i]/400.0F - 400.0F*PID_Kd[i];
		A2[i] = PID_Kd[i]*200.0F;
		PID_out[i] += A0[i]*E0[i] + A1[i]*E1[i] + A2[i]*E2[i];

		E2[i] = E1[i];
		E1[i] = E0[i];

//		PID_Test[i+6]=PID_out[i];

		if (PID_out[i]>PID_out_max){
			PID_out[i]=PID_out_max;
		}
		else if (PID_out[i]<PID_out_min){
			PID_out[i]=PID_out_min;
		}

		if (PID_out[i]>=0){
			dir_=dir_|(i+1);
		}
		else if (PID_out[i]<0)
			dir_=dir_&(2-i);


		PID_out_[i]=fabs(PID_out[i]);
		if (PID_in[i]==0){
			PID_out[i]=0;
		}
	}
  	PID_out_[2]=(float)dir_;


//	PID_Test[5] += PID_out[0];
	return PID_out_;
}

