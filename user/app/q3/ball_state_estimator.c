#include "ball_state_estimator.h"

#include <math.h>
#include <string.h>

#include "ball_control_config.h"

void BallStateEstimator_Init(BallStateEstimator *estimator)
{
    if (estimator != NULL)
    {
        memset(estimator, 0, sizeof(*estimator));
    }
}

bool BallStateEstimator_Update(BallStateEstimator *estimator,
                               float measured_position_cm,
                               float dt_s)
{
    float predicted_position;
    float innovation;
    int8_t direction;

    if ((estimator == NULL) || !isfinite(measured_position_cm))
    {
        return false;
    }

    if (!estimator->initialized)
    {
        estimator->position_cm = measured_position_cm;
        estimator->velocity_cm_s = 0.0f;
        estimator->innovation_cm = 0.0f;
        estimator->initialized = true;
        return true;
    }

    if (!isfinite(dt_s) || (dt_s <= 0.0f) || (dt_s > BALL_CONTROL_SAMPLE_MAX_S))
    {
        return false;
    }

    predicted_position = estimator->position_cm + estimator->velocity_cm_s * dt_s;
    innovation = measured_position_cm - predicted_position;
    estimator->innovation_cm = innovation;

    if (fabsf(innovation) > BALL_ESTIMATOR_INNOVATION_LIMIT_CM)
    {
        direction = (innovation < 0.0f) ? -1 : 1;
        if (direction == estimator->rejected_direction)
        {
            ++estimator->rejected_count;
        }
        else
        {
            estimator->rejected_direction = direction;
            estimator->rejected_count = 1U;
        }

        if (estimator->rejected_count < BALL_ESTIMATOR_REACQUIRE_COUNT)
        {
            return false;
        }

        /* 一致的测量胜于预测：以零速度安全地重新捕获，
         * 而不是制造一个巨大的微分值。 */
        estimator->position_cm = measured_position_cm;
        estimator->velocity_cm_s = 0.0f;
        estimator->rejected_count = 0U;
        estimator->rejected_direction = 0;
        return true;
    }

    estimator->position_cm = predicted_position + BALL_ESTIMATOR_ALPHA * innovation;
    estimator->velocity_cm_s += (BALL_ESTIMATOR_BETA / dt_s) * innovation;
    estimator->rejected_count = 0U;
    estimator->rejected_direction = 0;
    return isfinite(estimator->position_cm) && isfinite(estimator->velocity_cm_s);
}
