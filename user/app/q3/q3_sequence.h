#ifndef Q3_SEQUENCE_H
#define Q3_SEQUENCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    Q3_SEQUENCE_IDLE = 0,
    Q3_SEQUENCE_TO_PLUS5,
    Q3_SEQUENCE_BRAKE_PLUS5,
    Q3_SEQUENCE_TO_MINUS2_5,
    Q3_SEQUENCE_TO_MINUS5,
    Q3_SEQUENCE_RETURN_LEVEL,
    Q3_SEQUENCE_FINAL_HOLD,
    Q3_SEQUENCE_WAIT_CENTER,
    Q3_SEQUENCE_FAULT,
    Q3_SEQUENCE_STOPPED,
    Q3_SEQUENCE_CENTER,
    Q3_SEQUENCE_SINGLE_TARGET,
    Q3_SEQUENCE_RETURN_CRUISE,
    Q3_SEQUENCE_RETURN_BRAKE,
    /* 球到达 -5，X42S 已确认回到 PA31 原点，且
     * 位置会话已被显式暂停。 */
    Q3_SEQUENCE_FINISHED
} Q3SequenceState;

typedef struct
{
    Q3SequenceState state;
    uint32_t state_started_ms;
    uint32_t run_sequence;
    bool fault;
    uint32_t motion_started_ms;
    uint32_t completion_ms;
    bool time_pass;
    /* 首次 PA31 零点标记后为 true。这在遥测中
     * 有意暴露，以便证明后续轮次不会重定义零点。 */
    bool zero_reference_ready;
} Q3SequenceStatus;

void Q3Sequence_Init(void);
bool Q3Sequence_Start(uint32_t now_ms);
/* 第一次按 PA31：把电机当前物理姿态标记为角度零点。 */
bool Q3Sequence_ZeroReference(uint32_t now_ms);
bool Q3Sequence_IsZeroReferenceReady(void);
void Q3Sequence_EnableAutoCycle(bool enabled);
void Q3Sequence_Stop(uint32_t now_ms);
void Q3Sequence_Task(uint32_t now_ms);
bool Q3Sequence_IsActive(void);
bool Q3Sequence_ConsumeResultLog(void);
void Q3Sequence_GetStatus(Q3SequenceStatus *status);
const char *Q3Sequence_StateName(Q3SequenceState state);

#endif
