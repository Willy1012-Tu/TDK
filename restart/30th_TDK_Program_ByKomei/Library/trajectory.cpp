#include "trajectory.h"
#include <Arduino.h> 
#include <IntervalTimer.h>
#include <math.h>

// ==========================================
// 私有變數 (隱藏在 .c/.cpp 內)
// ==========================================

static IntervalTimer _sysTimer;                  // Teensy 硬體 Timer
static TrajGen_t* _instances[MAX_TRAJ_INSTANCES]; // 實例指標陣列
static uint8_t _instance_count = 0;              // 目前註冊數量
static float _sys_dt = 0.0f;                     // 統一的 dt (秒)

// ==========================================
// 私有運算核心 (原本的 Update 邏輯)
// ==========================================
static void _Traj_Compute_Step(TrajGen_t *tg) {
    if (!tg->is_active) return;
    
    float dt = _sys_dt;
    float err = tg->traj_target - tg->traj_pos;

    // 1. 判斷是否已到達目標 (Deadband)
    if (fabsf(err) < 1e-4f && fabsf(tg->traj_vel) < 1e-3f) {
        tg->traj_pos = tg->traj_target;
        tg->traj_vel = 0.0f;
        return;
    }

    // 2. 梯形加減速規劃邏輯
    float dir = (err >= 0.0f) ? 1.0f : -1.0f;
    float v_max_dir = dir * tg->maxVel;
    float d_stop = (tg->traj_vel * tg->traj_vel) / (2.0f * tg->maxAcc);
    float a = 0.0f;

    if (fabsf(err) <= d_stop) {
        // 減速區
        if (fabsf(tg->traj_vel) > 1e-3f) {
            a = -tg->maxAcc * ((tg->traj_vel > 0.0f) ? 1.0f : -1.0f);
        } else {
            a = 0.0f;
            tg->traj_vel = 0.0f;
        }
    } else {
        // 加速或定速區
        a = dir * tg->maxAcc;
        if ((dir > 0.0f && tg->traj_vel >= v_max_dir) ||
            (dir < 0.0f && tg->traj_vel <= v_max_dir)) {
            a = 0.0f;
            tg->traj_vel = v_max_dir;
        }
    }

    // 3. 積分
    tg->traj_vel += a * dt;
    tg->traj_pos += tg->traj_vel * dt;

    // 4. Overshoot 修正
    if (dir > 0.0f) {
        if (tg->traj_pos > tg->traj_target) {
            tg->traj_pos = tg->traj_target;
            tg->traj_vel = 0.0f;
        }
    } else {
        if (tg->traj_pos < tg->traj_target) {
            tg->traj_pos = tg->traj_target;
            tg->traj_vel = 0.0f;
        }
    }
}

// ==========================================
// 中斷服務程式 (ISR)
// ==========================================
static void _Traj_Global_ISR(void) {
    // 遍歷所有已註冊的實例，並執行運算
    for (int i = 0; i < _instance_count; i++) {
        if (_instances[i] != NULL) {
            _Traj_Compute_Step(_instances[i]);
        }
    }
}

// ==========================================
// 公開 API 實作
// ==========================================

bool Traj_Engine_Start(float interval_s) {
    if (interval_s <= 0.0f) return false;
    
    _sys_dt = interval_s;
    unsigned long period_us = (unsigned long)(interval_s * 1000000.0f);
    
    // 啟動 Teensy IntervalTimer
    return _sysTimer.begin(_Traj_Global_ISR, period_us);
}

void Traj_Init(TrajGen_t *tg, float now_pos, float maxVel, float maxAcc) {
    if (tg == NULL) return;
    if (_instance_count >= MAX_TRAJ_INSTANCES) return;

    // 初始化數值
    tg->traj_pos = now_pos;
    tg->traj_vel = 0.0f;
    tg->traj_target = now_pos;
    tg->maxVel = maxVel;
    tg->maxAcc = maxAcc;
    tg->is_active = true;

    // 註冊到全域陣列
    // 注意：這裡簡單處理，沒有做重複註冊檢查
    _instances[_instance_count] = tg;
    _instance_count++;
}

void Traj_SetTarget(TrajGen_t *tg, float target_pos) {
    if (tg == NULL) return;
    tg->traj_target = target_pos;
}

void Traj_SetCurrentPos(TrajGen_t *tg, float now_pos) {
    if (tg == NULL) return;
    // 重置時暫時關閉 Timer 中斷可能更安全，但在 float 操作下通常可接受
    tg->traj_pos = now_pos;
    tg->traj_vel = 0.0f;
    tg->traj_target = now_pos;
}

float Traj_GetPosition(TrajGen_t *tg) {
    if (tg == NULL) return 0.0f;
    return tg->traj_pos;
}

float Traj_GetVelocity(TrajGen_t *tg) {
    if (tg == NULL) return 0.0f;
    return tg->traj_vel;
}
