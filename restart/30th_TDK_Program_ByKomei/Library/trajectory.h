#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>
#include <stdbool.h>

// 定義最大支援的軌跡實例數量 
#define MAX_TRAJ_INSTANCES 2

typedef struct {
    // --- 狀態變數 (由 ISR 更新，使用者唯讀) ---
    volatile float traj_pos;     // 當前規劃位置
    volatile float traj_vel;     // 當前規劃速度
    
    // --- 控制變數 (使用者設定) ---
    volatile float traj_target;  // 目標位置
    float maxVel;                // 最大速度
    float maxAcc;                // 最大加速度
    
    // --- 內部旗標 ---
    bool is_active;              // 是否已啟用
} TrajGen_t;

// ==========================================
// 全域管理 API (控制 Timer 與統一週期)
// ==========================================

/**
 * @brief 設定統一的控制週期並啟動硬體 Timer
 * @param interval_s 執行週期 (秒)，例如 0.001 代表 1ms
 * @return true 成功啟動, false 失敗
 */
bool Traj_Engine_Start(float interval_s);

// ==========================================
// 個別實例 API
// ==========================================

/**
 * @brief 初始化並註冊一個軌跡實例
 * @param tg 指向結構體的指標
 * @param now_pos 初始位置
 * @param maxVel 最大速度
 * @param maxAcc 最大加速度
 */
void Traj_Init(TrajGen_t *tg, float now_pos, float maxVel, float maxAcc);

/**
 * @brief 設定新的目標位置
 */
void Traj_SetTarget(TrajGen_t *tg, float target_pos);

/**
 * @brief 重新設定當前狀態 (通常用於緊急重置或校正)
 */
void Traj_SetCurrentPos(TrajGen_t *tg, float now_pos);

/**
 * @brief 獲取當前規劃的位置 (Thread-safe)
 */
float Traj_GetPosition(TrajGen_t *tg);

/**
 * @brief 獲取當前規劃的速度 (Thread-safe)
 */
float Traj_GetVelocity(TrajGen_t *tg);

#endif