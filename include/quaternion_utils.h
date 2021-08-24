#ifndef __QUATERNION_UTILS_H
#define __QUATERNION_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

struct Gravity{
    float x;
    float y;
    float z;
};
struct Quaternion{
    float w;
    float x;
    float y;
    float z;
};

void getQuaternion(struct Quaternion *q, const int* MPUwQuat,const int* MPUxQuat, const int* MPUyQuat, const int* MPUzQuat);
void getGravity(struct Quaternion* q, struct Gravity* v);
void getYawPitchRoll(struct Quaternion* q, struct Gravity* gravity, float* MPUYaw, float* MPUPitch, float* MPURoll);

void filterYawPitchRoll(float* MPUYaw, float* MPUPitch, float* MPURoll, float* averageMPUYaw, float* averageMPUPitch, float* averageMPURoll, 
                        int* counter, float* offsetMPUYaw, float* offsetMPUPitch, float* offsetMPURoll);

void getEuler(struct Quaternion *q, float* MPUYaw, float* MPUPitch, float* MPURoll);
void castEuler(float* MPUYaw, float* MPUPitch, float* MPURoll, int* MPUYawSendable, int* MPUPitchSendable, int* MPURollSendable);
void normalizeEuler(float* MPUYaw, float* MPUPitch, float* MPURoll, float* MPUYawNormalized, float* MPUPitchNormalized, 
                    float* MPURollNormalized, float MPUMaxYaw, float MPUMaxPitch, float MPUMaxRoll);


#ifdef __cplusplus
}
#endif

#endif