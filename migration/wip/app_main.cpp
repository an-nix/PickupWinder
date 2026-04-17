#include "StepperController.h"
#include "RampPlanner.h"

extern "C" void app_main() {
    StepperConfig cfg = {
        .step_gpio      = GPIO_NUM_18,
        .dir_gpio       = GPIO_NUM_19,
        .resolution_hz  = 1'000'000,
        .task_priority  = 5,
        .task_core      = 0,          // exécution sur core 0
    };
    StepperController ctrl(cfg);
    ctrl.start();

    RampPlanner::MotionProfile profile = {
        .max_freq_hz   = 10'000,
        .start_freq_hz = 500,
        .sample_points = 20,
        .accel_ratio   = 0.2f,
        .decel_ratio   = 0.2f,
    };

    // RampPlanner tourne sur core 1 (ou dans une tâche séparée)
    RampPlanner planner(ctrl, profile);
    planner.moveTo(3200);   // 1 tour (200 pas × 16 micro-steps)
    planner.moveTo(0);      // retour origine
}