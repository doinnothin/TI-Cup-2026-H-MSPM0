#ifndef BALL_STATE_ESTIMATOR_H
#define BALL_STATE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float position_cm;
    float velocity_cm_s;
    float innovation_cm;
    uint8_t rejected_count;
    int8_t rejected_direction;
    bool initialized;
} BallStateEstimator;

void BallStateEstimator_Init(BallStateEstimator *estimator);

/* Alpha-beta 观测器。单个不合理的位置跳变会被忽略。同一方向上
 * 连续三次跳变被视为真实运动并重新捕获测量，
 * 因此该门限不会永久丢失球。 */
bool BallStateEstimator_Update(BallStateEstimator *estimator,
                               float measured_position_cm,
                               float dt_s);

#endif
