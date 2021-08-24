#include "quaternion_utils.h"
#include "math.h"

void getQuaternion(struct Quaternion *q, const int* MPUwQuat,const int* MPUxQuat, const int* MPUyQuat, const int* MPUzQuat){

	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	q -> w = (float)(*MPUwQuat >> 16) / 16384.0f;
	q -> x = (float)(*MPUxQuat >> 16) / 16384.0f;
	q -> y = (float)(*MPUyQuat >> 16) / 16384.0f;
	q -> z = (float)(*MPUzQuat >> 16) / 16384.0f;
}

void getGravity(struct Quaternion* q, struct Gravity* v){
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
}

void getYawPitchRoll(struct Quaternion* q, struct Gravity* gravity, float* MPUYaw, float* MPUPitch, float* MPURoll){
    // yaw: (about Z axis)
    *MPUYaw = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    *MPUPitch = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    *MPURoll = atan2(gravity -> y , gravity -> z);
    if (gravity -> z < 0) {
        if(*MPUPitch > 0) {
            *MPUPitch = M_PI - *MPUPitch; 
        } else { 
            *MPUPitch = -M_PI - *MPUPitch;
        }
    }
}
void filterYawPitchRoll(float* MPUYaw, float* MPUPitch, float* MPURoll, float* averageMPUYaw, float* averageMPUPitch, float* averageMPURoll, 
                        int* counter, float* offsetMPUYaw, float* offsetMPUPitch, float* offsetMPURoll){
    

    if(*counter < 300){

        *offsetMPUYaw = 0;
        *offsetMPUPitch = 0;
        *offsetMPURoll = 0;
        
        *averageMPUYaw = *MPUYaw/300;
        *averageMPUPitch = *MPUPitch/300;
        *averageMPURoll = *MPURoll/300;
        
        *counter += 1;
        
    }else if(*counter<600){

        *averageMPUYaw += *MPUYaw/300;
        *averageMPUPitch += *MPUPitch/300;
        *averageMPURoll += *MPURoll/300;

        *offsetMPUYaw = *averageMPUYaw;
        *offsetMPUPitch = *averageMPUPitch;
        *offsetMPURoll = *averageMPURoll;

        *counter += 1;

    }else{

        *MPUYaw -= *offsetMPUYaw;        
        *MPUPitch -= *offsetMPUPitch;
        *MPURoll -= *offsetMPURoll;

        // *MPUYaw *= 1000;
        // *MPUPitch *= 1000;
        // *MPURoll *= 1000;

    }

}


void getEuler(struct Quaternion *q, float* MPUYaw, float* MPUPitch, float* MPURoll){
    *MPUYaw = atan2(2 * q -> x * q -> y - 2 * q -> w * q -> z, 2 * q -> w * q -> w + 2 * q -> x * q -> x - 1); // psi
	*MPUPitch = -asin(2 * q -> x * q -> z + 2 * q -> w * q -> y);                      // theta
	*MPURoll = atan2(2 * q -> y * q -> z - 2 * q -> w * q -> x, 2 * q -> w * q -> w + 2 * q -> z * q -> z - 1); // phi
}

void castEuler(float* MPUYaw, float* MPUPitch, float* MPURoll, int* MPUYawSendable, int* MPUPitchSendable, int* MPURollSendable){
    *MPUYawSendable = (int)(*MPUYaw*1000);
    *MPUPitchSendable = (int)(*MPUPitch*1000);
    *MPURollSendable = (int)(*MPURoll*1000);
}

void normalizeEuler(float* MPUYaw, float* MPUPitch, float* MPURoll, float* MPUYawNormalized, float* MPUPitchNormalized, 
                    float* MPURollNormalized, float MPUMaxYaw, float MPUMaxPitch, float MPUMaxRoll){
    
    *MPUYawNormalized = *MPUYaw / MPUMaxYaw;
    *MPUPitchNormalized = *MPUPitch / MPUMaxPitch;
    *MPURollNormalized = *MPURoll / MPUMaxRoll;

    if(*MPUYawNormalized >1){
        *MPUYawNormalized = 1;
    }else if(*MPUYawNormalized < -1){
        *MPUYawNormalized = -1;
    }

    if(*MPUPitchNormalized >1){
        *MPUPitchNormalized = 1;
    }else if(*MPUPitchNormalized < -1){
        *MPUPitchNormalized = -1;
    }

    if(*MPURollNormalized >1){
        *MPURollNormalized = 1;
    }else if(*MPURollNormalized < -1){
        *MPURollNormalized = -1;
    }
}