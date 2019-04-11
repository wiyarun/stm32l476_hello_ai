#ifndef __AI_PREDICTATION_H__
#define __AI_PREDICTATION_H__


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
	DOWNSTARIRS = 0,
	JOGGING,
	SITTING,
	STANDING,
	UPSTAIRS,
	WALKING
}predict_activity_t;

	
int aiPredictationInit(void);
int aiPredictationProcess(void);
void aiPredictationDeInit(void);
int32_t decode_activity_type(const float data_output[], uint32_t output_size);

#ifdef __cplusplus
}
#endif



#endif