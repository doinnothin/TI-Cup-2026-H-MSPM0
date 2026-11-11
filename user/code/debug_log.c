#include "debug_log.h"

#include <stdio.h>

#include "ti_msp_dl_config.h"
#include "app/ball_actuator/ball_actuator.h"
#include "app/ball_actuator/pendulum_calibration.h"
#include "app/q3/ball_position_control.h"
#include "app/q3/ball_mechanism_id.h"
#include "app/q3/pendulum_geometry_calibration.h"
#include "app/q3/ball_pulse_id.h"
#include "app/q3/q3_sequence.h"
#include "app/q4/q45_vehicle_control.h"
#include "app/q6/q6_config.h"
#include "app/q3/q3_control_config.h"
#include "uart.h"
#include "vision_link.h"
#include "x42s.h"
#include "bsp/pid.h"
#include "code/encoder.h"

#define DEBUG_LOG_BUFFER_SIZE 2048U
#define DEBUG_LOG_SEND_BUDGET 16U

static uint8_t s_buffer[DEBUG_LOG_BUFFER_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static uint32_t s_dropped_lines;
static uint32_t s_record_sequence;
static uint16_t s_queue_high_water;

static uint16_t DebugLog_Next(uint16_t index)
{
    ++index;
    if (index >= DEBUG_LOG_BUFFER_SIZE)
    {
        index = 0U;
    }
    return index;
}

static void DebugLog_Queue(const char *text)
{
    char suffix[20];
    uint16_t input_len = 0U;
    uint16_t body_len;
    uint16_t suffix_len = 0U;
    uint16_t free_bytes;
    uint16_t used_bytes;
    uint16_t i;
    uint16_t next;

    while (text[input_len] != '\0')
        ++input_len;
    body_len = input_len;
    if ((body_len >= 2U) && (text[body_len - 2U] == '\r') && (text[body_len - 1U] == '\n'))
        body_len -= 2U;
    ++s_record_sequence;
    (void)snprintf(suffix, sizeof(suffix), " logseq=%lu\r\n", (unsigned long)s_record_sequence);
    while (suffix[suffix_len] != '\0')
        ++suffix_len;
    free_bytes = (uint16_t)((s_tail + DEBUG_LOG_BUFFER_SIZE - s_head - 1U) % DEBUG_LOG_BUFFER_SIZE);
    if ((uint32_t)body_len + suffix_len > free_bytes)
    {
        ++s_dropped_lines;
        return;
    }
    for (i = 0U; i < body_len; ++i)
    {
        next = DebugLog_Next(s_head);
        s_buffer[s_head] = (uint8_t)text[i];
        s_head = next;
    }
    for (i = 0U; i < suffix_len; ++i)
    {
        next = DebugLog_Next(s_head);
        s_buffer[s_head] = (uint8_t)suffix[i];
        s_head = next;
    }
    used_bytes = (uint16_t)((s_head + DEBUG_LOG_BUFFER_SIZE - s_tail) % DEBUG_LOG_BUFFER_SIZE);
    if (used_bytes > s_queue_high_water)
        s_queue_high_water = used_bytes;
}

void DebugLog_Init(void)
{
    char line[256];

    s_head = 0U;
    s_tail = 0U;
    s_dropped_lines = 0U;
    s_record_sequence = 0U;
    s_queue_high_water = 0U;
    (void)snprintf(line,
                   sizeof(line),
                   "META t=0 schema=1 baud=115200 pos10=mm vel10=mm_s angle10=0.1deg "
                   "q45_hz=5 q6_run_hz=10 vl_q6_hz=5 act_q6_hz=2 "
                   "q6task=%u q6static=%u q6autostart=%u bootselect=%u target_mm=%d "
                   "ev_pb24=1 ev_state=2 ev_target=3 ev_run=4 ev_stop=5 ev_fault=6\r\n",
                   (unsigned)Q6_TASK_ENABLE,
                   (unsigned)Q6_STATIC_TEST_MODE,
                   (unsigned)Q6_BENCH_AUTO_START,
                   (unsigned)Q6_BOOT_AUTO_SELECT,
                   (int)Q6_BENCH_EXPLICIT_TARGET_MM);
    DebugLog_Queue(line);
}

void DebugLog_PublishQ6Event(uint32_t now_ms, uint8_t code, int32_t value)
{
    Q45VehicleStatus vehicle;
    char line[128];

    Q45Vehicle_GetStatus(&vehicle);
    (void)snprintf(line,
                   sizeof(line),
                   "EV t=%lu run=%lu mode=%u code=%u value=%ld\r\n",
                   (unsigned long)now_ms,
                   (unsigned long)vehicle.run_sequence,
                   (unsigned)vehicle.mode,
                   (unsigned)code,
                   (long)value);
    DebugLog_Queue(line);
}

void DebugLog_PublishVision(uint32_t now_ms)
{
    VisionLinkSample sample;
    VisionLinkStats stats;
    Q45VehicleStatus vehicle;
    char line[512];
    int position_x10 = 0;
    int velocity_x10 = 0;
    uint32_t age_ms = 0xFFFFFFFFUL;

    (void)VisionLink_GetLatest(&sample);
    VisionLink_GetStats(&stats);
    Q45Vehicle_GetStatus(&vehicle);

    if (sample.valid)
    {
        position_x10 = (int)(sample.position_cm * 10.0f);
        velocity_x10 = (int)(sample.velocity_cm_s * 10.0f);
        age_ms = (int32_t)(now_ms - sample.rx_time_ms) < 0 ? 0U : now_ms - sample.rx_time_ms;
    }

    (void)snprintf(line,
                   sizeof(line),
                   "VL t=%lu run=%lu mode=%u valid=%u seq=%u pos10=%d vel10=%d stable=%u "
                   "conf=%u cx=%u cy=%u age=%lu bytes=%lu ok=%lu det=%lu "
                   "crc=%lu len=%lu irq=%lu poll=%lu lost=%lu drop=%lu\r\n",
                   (unsigned long)now_ms,
                   (unsigned long)vehicle.run_sequence,
                   (unsigned)vehicle.mode,
                   sample.valid ? 1U : 0U,
                   (unsigned int)sample.sequence,
                   position_x10,
                   velocity_x10,
                   (unsigned int)sample.stable,
                   (unsigned int)sample.confidence_pct,
                   (unsigned int)sample.pixel_cx,
                   (unsigned int)sample.pixel_cy,
                   (unsigned long)age_ms,
                   (unsigned long)stats.bytes_received,
                   (unsigned long)stats.frames_ok,
                   (unsigned long)stats.detect_frames,
                   (unsigned long)stats.crc_errors,
                   (unsigned long)stats.length_errors,
                   (unsigned long)stats.irq_count,
                   (unsigned long)stats.poll_bytes,
                   (unsigned long)stats.sequence_lost,
                   (unsigned long)s_dropped_lines);
    DebugLog_Queue(line);
}

void DebugLog_PublishActuator(uint32_t now_ms)
{
    BallActuatorStatus actuator;
    const X42S_State *motor = X42S_GetState();
    Q45VehicleStatus vehicle;
    char line[224];
    const char *firmware = "NA";
    const char *control = "NA";

    BallActuator_GetStatus(&actuator);
    Q45Vehicle_GetStatus(&vehicle);
    if (actuator.options_valid)
    {
        firmware = actuator.firmware_x ? "X" : "EMM";
        control = actuator.foc_closed_loop ? "FOC" : "OPEN";
    }

    (void)snprintf(line,
                   sizeof(line),
                   "ACT t=%lu run=%lu mode=%u id=%u online=%u fw=%s ctrl=%s opt=%04X "
                   "req=%lu to=%lu preq=%lu pto=%lu cmd10=%d act10=%d pact=%lu "
                   "rx=%lu ok=%lu err=%lu ovf=%lu fn=%02X rep=%02X st=%02X\r\n",
                   (unsigned long)now_ms,
                   (unsigned long)vehicle.run_sequence,
                   (unsigned)vehicle.mode,
                   (unsigned int)actuator.address,
                   actuator.online ? 1U : 0U,
                   firmware,
                   control,
                   (unsigned int)actuator.option_flags,
                   (unsigned long)actuator.requests_sent,
                   (unsigned long)actuator.request_timeouts,
                   (unsigned long)actuator.position_requests_sent,
                   (unsigned long)actuator.position_request_timeouts,
                   actuator.commanded_angle_valid ? (int)(actuator.commanded_angle_deg * 10.0f) : 0,
                   actuator.actual_angle_valid ? (int)(actuator.actual_angle_deg * 10.0f) : 0,
                   actuator.actual_angle_valid ? (unsigned long)(now_ms - actuator.last_position_ms)
                                               : 0xFFFFFFFFUL,
                   (unsigned long)motor->raw_bytes_received,
                   (unsigned long)motor->frames_received,
                   (unsigned long)motor->frame_errors,
                   (unsigned long)motor->rx_overflows,
                   (unsigned int)actuator.last_function,
                   (unsigned int)actuator.last_reply,
                   (unsigned int)actuator.status_flags);
    DebugLog_Queue(line);
}

void DebugLog_PublishPendulumCalibration(uint32_t now_ms)
{
    PendulumCalibrationStatus calibration;
    char line[128];

    PendulumCalibration_GetStatus(&calibration);
    (void)snprintf(line,
                   sizeof(line),
                   "CAL t=%lu state=%s target10=%d seq=%lu fault=%u age=%lu\r\n",
                   (unsigned long)now_ms,
                   PendulumCalibration_StateName(calibration.state),
                   (int)(calibration.target_angle_deg * 10.0f),
                   (unsigned long)calibration.sequence,
                   (unsigned int)calibration.fault,
                   (unsigned long)(now_ms - calibration.state_started_ms));
    DebugLog_Queue(line);
}

void DebugLog_PublishBallPositionControl(uint32_t now_ms)
{
    BallPositionControlStatus control;
    char line[512];

    BallPositionControl_GetStatus(&control);
    (void)snprintf(
        line,
        sizeof(line),
        "BC t=%lu state=%s target10=%d meas10=%d err10=%d "
        "p10=%d i10=%d d10=%d out10=%d cmd10=%d act10=%d "
        "vcal10=%d vvis10=%d "
        "vseq=%u vage=%lu aage=%lu kpeak10=%d kelapsed=%lu "
        "fresh=%u afresh=%u hit=%u "
        "run=%lu rekick=%u fault=%u brake=%u stop10=%d dt=%lu lag=%lu calc=%lu tx=%lu\r\n",
        (unsigned long)now_ms,
        BallPositionControl_StateName(control.state),
        (int)(control.target_cm * 10.0f),
        (int)(control.measured_cm * 10.0f),
        (int)(control.error_cm * 10.0f),
        (int)(control.pid_p_deg * 10.0f),
        (int)(control.pid_i_deg * 10.0f),
        (int)(control.pid_d_deg * 10.0f),
        (int)(control.control_output_deg * 10.0f),
        (int)(control.target_angle_deg * 10.0f),
        (int)(control.actual_angle_deg * 10.0f),
        (int)(control.measured_velocity_cm_s * 10.0f),
        (int)(control.vision_velocity_cm_s * 10.0f),
        (unsigned int)control.vision_sequence,
        (unsigned long)control.vision_age_ms,
        (unsigned long)control.actual_angle_age_ms,
        (int)(control.kick_peak_angle_deg * 10.0f),
        (unsigned long)control.kick_elapsed_ms,
        control.vision_fresh ? 1U : 0U,
        control.actual_angle_fresh ? 1U : 0U,
        control.target_reached ? 1U : 0U,
        (unsigned long)control.run_sequence,
        (unsigned int)control.rekick_count,
        (unsigned int)control.fault,
        control.braking ? 1U : 0U,
        (int)(control.stopping_distance_cm * 10.0f),
        (unsigned long)control.sample_dt_ms,
        (unsigned long)control.dispatch_age_ms,
        (unsigned long)control.pid_updates,
        (unsigned long)control.command_count);
    DebugLog_Queue(line);
}

void DebugLog_PublishBallMechanismId(uint32_t now_ms)
{
    BallMechanismIdStatus id;
    char line[192];

    BallMechanismId_GetStatus(&id);
    (void)snprintf(line,
                   sizeof(line),
                   "ID t=%lu st=%s d=%d n10=%d a10=%d exp10=%d cur10=%d "
                   "s10=%d e10=%d dx10=%d mv=%u p10=%d m10=%d pt=%lu "
                   "rej=%u age=%lu f=%u\\r\\n",
                   (unsigned long)now_ms,
                   BallMechanismId_StateName(id.state),
                   (int)id.direction,
                   (int)(id.next_angle_deg * 10.0f),
                   (int)(id.target_angle_deg * 10.0f),
                   (int)(id.expected_start_cm * 10.0f),
                   (int)(id.current_cm * 10.0f),
                   (int)(id.start_cm * 10.0f),
                   (int)(id.end_cm * 10.0f),
                   (int)(id.delta_cm * 10.0f),
                   id.moved ? 1U : 0U,
                   (int)(id.positive_safe_deg * 10.0f),
                   (int)(id.negative_safe_deg * 10.0f),
                   (unsigned long)id.point_sequence,
                   (unsigned int)id.trigger_rejects,
                   (unsigned long)id.vision_age_ms,
                   (unsigned int)id.fault);
    DebugLog_Queue(line);
}

void DebugLog_PublishPendulumGeometryCalibration(uint32_t now_ms)
{
    PendulumGeometryCalibrationStatus geometry;
    char line[128];

    PendulumGeometryCalibration_GetStatus(&geometry);
    (void)snprintf(line,
                   sizeof(line),
                   "GEO t=%lu st=%s target10=%d next10=%d pt=%lu fault=%u age=%lu\\r\\n",
                   (unsigned long)now_ms,
                   PendulumGeometryCalibration_StateName(geometry.state),
                   (int)(geometry.target_angle_deg * 10.0f),
                   (int)(geometry.next_angle_deg * 10.0f),
                   (unsigned long)geometry.point_sequence,
                   (unsigned int)geometry.fault,
                   (unsigned long)(now_ms - geometry.state_started_ms));
    DebugLog_Queue(line);
}

void DebugLog_PublishBallPulseId(uint32_t now_ms)
{
    BallPulseIdStatus pulse;
    BallActuatorStatus actuator;
    char line[192];

    BallPulseId_GetStatus(&pulse);
    BallActuator_GetStatus(&actuator);
    (void)snprintf(line,
                   sizeof(line),
                   "PULSE t=%lu st=%s angle10=%d start10=%d peak10=%d end10=%d "
                   "vpk10=%d cmd10=%d act10=%d pact=%lu seq=%u age=%lu "
                   "safe=%u fault=%u\\r\\n",
                   (unsigned long)now_ms,
                   BallPulseId_StateName(pulse.state),
                   (int)(pulse.target_angle_deg * 10.0f),
                   (int)(pulse.start_cm * 10.0f),
                   (int)(pulse.peak_cm * 10.0f),
                   (int)(pulse.end_cm * 10.0f),
                   (int)(pulse.peak_velocity_cm_s * 10.0f),
                   actuator.commanded_angle_valid ? (int)(actuator.commanded_angle_deg * 10.0f) : 0,
                   actuator.actual_angle_valid ? (int)(actuator.actual_angle_deg * 10.0f) : 0,
                   actuator.actual_angle_valid ? (unsigned long)(now_ms - actuator.last_position_ms)
                                               : 0xFFFFFFFFUL,
                   (unsigned int)pulse.vision_sequence,
                   (unsigned long)pulse.vision_age_ms,
                   pulse.safety_return ? 1U : 0U,
                   (unsigned int)pulse.fault);
    DebugLog_Queue(line);
}

void DebugLog_PublishQ3Sequence(uint32_t now_ms)
{
    Q3SequenceStatus sequence;
    BallPositionControlStatus control;
    char line[260];

    Q3Sequence_GetStatus(&sequence);
    BallPositionControl_GetStatus(&control);
    (void)snprintf(line,
                   sizeof(line),
                   "Q3 t=%lu st=%s run=%lu final10=%d ref10=%d meas10=%d vel10=%d err10=%d "
                   "out10=%d cmd10=%d act10=%d seq=%u age=%lu brake=%u zero=%u fault=%u "
                   "stateage=%lu finish_ms=%lu timepass=%u\r\n",
                   (unsigned long)now_ms,
                   Q3Sequence_StateName(sequence.state),
                   (unsigned long)sequence.run_sequence,
                   (int)(10.0f * Q3_RETURN_FINAL_TARGET_CM),
                   (int)(10.0f * control.target_cm),
                   (int)(10.0f * control.measured_cm),
                   (int)(10.0f * control.measured_velocity_cm_s),
                   (int)(10.0f * control.error_cm),
                   (int)(10.0f * control.control_output_deg),
                   (int)(10.0f * control.target_angle_deg),
                   (int)(10.0f * control.actual_angle_deg),
                   (unsigned)control.vision_sequence,
                   (unsigned long)control.vision_age_ms,
                   control.braking ? 1U : 0U,
                   sequence.zero_reference_ready ? 1U : 0U,
                   sequence.fault ? 1U : 0U,
                   (unsigned long)(now_ms - sequence.state_started_ms),
                   (unsigned long)sequence.completion_ms,
                   sequence.time_pass ? 1U : 0U);
    DebugLog_Queue(line);
}

void DebugLog_PublishQ45Vehicle(uint32_t now_ms)
{
    static bool q6_state_valid;
    static Q45VehicleState q6_previous_state;
    static uint32_t q6_previous_run;
    Q45VehicleStatus v;
    Q6TargetStatus target;
    char line[512];

    Q45Vehicle_GetStatus(&v);
    Q6Target_GetStatus(&target);
    if (v.mode == Q45_VEHICLE_MODE_Q6)
    {
        if (!q6_state_valid || (v.state != q6_previous_state) ||
            (v.run_sequence != q6_previous_run))
        {
            DebugLog_PublishQ6Event(now_ms, 2U, (int32_t)v.state);
            if (v.target_locked)
                DebugLog_PublishQ6Event(now_ms, 3U, (int32_t)(v.target_cm * 10.0f));
            if (v.state == Q45_VEHICLE_RUNNING)
                DebugLog_PublishQ6Event(now_ms, 4U, (int32_t)(v.target_cm * 10.0f));
            if (v.state >= Q45_VEHICLE_FINISHED)
            {
                DebugLog_PublishQ6Event(now_ms, 5U, (int32_t)v.stop_reason);
                if (v.fault)
                    DebugLog_PublishQ6Event(now_ms, 6U, (int32_t)v.stop_reason);
            }
            q6_previous_state = v.state;
            q6_previous_run = v.run_sequence;
            q6_state_valid = true;
        }
    }
    else
    {
        q6_state_valid = false;
    }
    (void)snprintf(line,
                   sizeof(line),
                   "Q45 t=%lu mode=%s st=%s run=%lu elapsed=%lu pass=%u passms=%lu "
                   "dist10=%d spd10=%d raw10=%d meas10=%d vel10=%d pred10=%d max10=%d "
                   "cmd10=%d act10=%d prof=%u ff=%u lim=%u in=%u viol=%u "
                   "center=%u zero=%u pause=%u conf=%u seq=%u age=%lu stop=%u why=%u fault=%u "
                   "tgt10=%d dev10=%d tmax10=%d capready=%u cap10=%d capframes=%lu "
                   "capspan=%lu locked=%u logdrop=%lu loghigh=%u\r\n",
                   (unsigned long)now_ms,
                   Q45Vehicle_ModeName(v.mode),
                   Q45Vehicle_StateName(v.state),
                   (unsigned long)v.run_sequence,
                   (unsigned long)v.elapsed_ms,
                   v.route_passed ? 1U : 0U,
                   (unsigned long)v.pass_elapsed_ms,
                   (int)(v.route_distance_cm * 10),
                   (int)(v.base_speed_mm_s * 10),
                   (int)(v.raw_position_cm * 10),
                   (int)(v.measured_cm * 10),
                   (int)(v.measured_velocity_cm_s * 10),
                   (int)(v.predicted_position_cm * 10),
                   (int)(v.max_abs_position_cm * 10),
                   (int)(v.commanded_angle_deg * 10),
                   (int)(v.actual_angle_deg * 10),
                   (unsigned)v.control_profile,
                   v.acceleration_feedforward_enabled ? 1U : 0U,
                   v.control_output_limited ? 1U : 0U,
                   v.in_scoring_band ? 1U : 0U,
                   v.band_violation ? 1U : 0U,
                   v.center_ready ? 1U : 0U,
                   v.terminal_zero_confirmed ? 1U : 0U,
                   v.actuator_paused ? 1U : 0U,
                   (unsigned)v.vision_confidence_pct,
                   (unsigned)v.vision_sequence,
                   (unsigned long)v.vision_age_ms,
                   v.stop_requested ? 1U : 0U,
                   (unsigned)v.stop_reason,
                   v.fault ? 1U : 0U,
                   (int)(v.target_cm * 10),
                   (int)((v.raw_position_cm - v.target_cm) * 10),
                   (int)(v.max_target_dev_cm * 10),
                   target.ready ? 1U : 0U,
                   (int)(target.candidate_cm * 10.0f),
                   (unsigned long)target.independent_frames,
                   (unsigned long)target.span_ms,
                   v.target_locked ? 1U : 0U,
                   (unsigned long)s_dropped_lines,
                   (unsigned)s_queue_high_water);
    DebugLog_Queue(line);
    if ((v.mode != Q45_VEHICLE_MODE_Q5) && (v.mode != Q45_VEHICLE_MODE_Q6))
        return;
    /* 第二条短行使两条记录都能在有限缓冲区内完整保存。
     * 对比车轮目标速度与实际速度，可区分是主动指令停车还是
     * 轮速环跟踪问题。所有速度字段单位均为 mm/s * 10。 */
    (void)snprintf(line,
                   sizeof(line),
                   "Q5D t=%lu run=%lu vsp10=%d ax100=%d ay100=%d ap100=%d "
                   "pd100=%d i100=%d f100=%d beam100=%d "
                   "lt10=%d rt10=%d lv10=%d rv10=%d "
                   "accepted=%lu rejected=%lu goodage=%lu pact=%lu gap=%lu "
                   "logdrop=%lu loghigh=%u\r\n",
                   (unsigned long)now_ms,
                   (unsigned long)v.run_sequence,
                   (int)(v.actual_speed_mm_s * 10),
                   (int)(v.longitudinal_accel_m_s2 * 100),
                   (int)(v.lateral_accel_m_s2 * 100),
                   (int)(v.parallel_accel_m_s2 * 100),
                   (int)(v.p_beam_deg * 100),
                   (int)(v.i_beam_deg * 100),
                   (int)(v.feedforward_beam_deg * 100),
                   (int)(v.requested_beam_deg * 100),
                   (int)(motorA.target * 10),
                   (int)(motorB.target * 10),
                   (int)(E_speed[0] * 10),
                   (int)(E_speed[1] * 10),
                   (unsigned long)v.accepted_frames,
                   (unsigned long)v.rejected_frames,
                   (unsigned long)v.accepted_age_ms,
                   (unsigned long)v.actual_angle_age_ms,
                   (unsigned long)v.max_frame_gap_ms,
                   (unsigned long)s_dropped_lines,
                   (unsigned)s_queue_high_water);
    DebugLog_Queue(line);
}

void DebugLog_Task(void)
{
    uint8_t count = 0U;

    while ((s_tail != s_head) && (count < DEBUG_LOG_SEND_BUDGET))
    {
        if (DL_UART_Main_isTXFIFOFull(DEBUG_INST))
            break;
        DL_UART_Main_transmitData(DEBUG_INST, s_buffer[s_tail]);
        s_tail = DebugLog_Next(s_tail);
        ++count;
    }
}

void DebugLog_PublishControlFrame(uint32_t now_ms)
{
    static uint32_t previous_run, previous_update, model_log_ms;
    BallPositionControlStatus c;
    char line[240];
    BallPositionControl_GetStatus(&c);
    if (c.pid_updates == 0U || (previous_run == c.run_sequence && previous_update == c.pid_updates))
        return;
    previous_run = c.run_sequence;
    previous_update = c.pid_updates;
    /* 整型遥测：位置/速度/角度/距离均放大 10 倍。 */
    (void)snprintf(line,
                   sizeof(line),
                   "C,%lu,%lu,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%lu,%lu,%lu,%u\r\n",
                   (unsigned long)now_ms,
                   (unsigned long)c.pid_updates,
                   (unsigned)c.state,
                   (unsigned)c.vision_sequence,
                   (int)(10 * c.target_cm),
                   (int)(10 * c.measured_cm),
                   (int)(10 * c.measured_velocity_cm_s),
                   (int)(10 * c.target_angle_deg),
                   (int)(10 * c.actual_angle_deg),
                   (int)(10 * c.pid_p_deg),
                   (int)(10 * c.pid_i_deg),
                   (int)(10 * c.pid_d_deg),
                   (int)(10 * c.stopping_distance_cm),
                   (unsigned)c.braking,
                   (unsigned long)c.sample_dt_ms,
                   (unsigned long)c.dispatch_age_ms,
                   (unsigned long)c.actual_angle_age_ms,
                   (unsigned)c.fault);
    DebugLog_Queue(line);
    if (now_ms - model_log_ms < 50U)
        return;
    model_log_ms = now_ms;
    /* PD 控制在 <=20 Hz 输出。束（beam）域各项/速率/高度均放大 100 倍；电机量放大 10 倍。
     * rej 表示 alpha-beta 新息门限是否拒绝了近期数据。 */
    (void)snprintf(line,
                   sizeof(line),
                   "PD,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d,%u\r\n",
                   (unsigned long)now_ms,
                   (int)(100 * c.outer_pd.p_deg),
                   (int)(100 * c.outer_pd.d_deg),
                   (int)(100 * c.outer_pd.raw_beam_deg),
                   (int)(100 * c.outer_pd.limited_beam_deg),
                   (int)(100 * c.outer_pd.shaped_beam_deg),
                   (int)(100 * c.outer_pd.beam_rate_deg_s),
                   (int)(100 * c.outer_pd.end_height_mm),
                   (int)(10 * c.outer_pd.motor_deg),
                   (int)(10 * c.estimator_innovation_cm),
                   (unsigned)c.estimator_reject_count,
                   (unsigned)c.outer_pd.limited,
                   (int)(10 * c.outer_pd.stiction_motor_deg),
                   (unsigned)c.outer_pd.stiction_assist);
    DebugLog_Queue(line);
}
