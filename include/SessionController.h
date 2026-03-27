#pragma once

#include "WinderApp.h"

/**
 * @brief High-level session lifecycle state.
 *
 * This state is independent from the internal `WinderApp` state machine and is
 * used to arbitrate user intents (start/pause/stop) coming from multiple input
 * sources.
 */
/**
 * @brief Session state: IDLE, ARMED_OR_RUNNING, PAUSED.
 *
 * ARMED_OR_RUNNING means the session is either actively running or armed for motion (e.g. after a start command, even if pot is zero).
 */
enum class SessionState { IDLE, ARMED_OR_RUNNING, PAUSED };

/**
 * @brief Multi-source session arbiter for winding control.
 *
 * Responsibilities:
 * - Merge intents from potentiometer, footswitch and UI/commands.
 * - Apply a strict "last event wins" policy.
 * - Enforce potentiometer re-arm rules when ownership changes.
 * - Drive `WinderApp` through start/pause/stop transitions.
 * - Push real-time speed command (`setControlHz`) based on pot value when running.
 *
 * Design notes:
 * - Input integration is edge-based for physical controls to avoid repeated
 *   re-triggering on unchanged values.
 * - Command processing is bounded by `TickInput::MAX_CMDS` to remain
 *   allocation-free and deterministic in embedded runtime.
 */
class SessionController {
public:
    /**
     * @brief Origin of an intent/event.
     */
    enum class InputSource { None, Pot, IHM, Footswitch };

    /**
     * @brief Resolved control action to apply to the session.
     */
    enum class ControlIntent { None, Start, Pause, Stop };

    /**
     * @brief Construct a session arbiter bound to the winding application.
     * @param winder Non-owning reference to the winding domain controller.
     */
    explicit SessionController(WinderApp& winder);

    /**
     * @brief Request a start transition from UI/command side.
     *
     * The request is not applied immediately; it is recorded as a pending intent
     * and executed during `tick()` according to last-event arbitration.
     */
    void requestStart();

    /**
     * @brief Request a pause transition from UI/command side.
     *
     * The request is deferred and applied in `tick()`.
     */
    void requestPause();

    /**
     * @brief Request a stop transition from UI/command side.
     *
     * Stop follows the same arbitration mechanism as other intents.
     */
    void requestStop();

    /**
     * @brief Runtime input bundle consumed once per control cycle.
     *
     * This struct gathers all data produced by other tasks/controllers and keeps
     * the session update call deterministic and allocation-free.
     */
    struct TickInput {
        /** Latest normalized potentiometer level in [0..1]. */
        float potLevel = 0.0f;   // 0.0 .. 1.0
        /** True when `potLevel` contains a fresh value for this tick. */
        bool hasPot = false;
        /** Latest footswitch state (pressed/released). */
        bool footswitch = false;
        /** True when `footswitch` contains a fresh value for this tick. */
        bool hasFootswitch = false;
        /** Monotonic timestamp provided by caller (`millis()`). */
        uint32_t now = 0;
        /** Encoder delta (detents since last tick, bounded). */
        int32_t encoderDelta = 0;
        /** Max number of commands accepted per tick. */
        static constexpr int MAX_CMDS = 16;
        /** Fixed-size command array (shared command representation). */
        CommandEntry commands[MAX_CMDS];
        /** Number of valid commands in `commands`. */
        int cmdCount = 0;
    };

    /**
     * @brief Handle one textual command if it belongs to session scope.
     * @param cmd Command key.
     * @param value Command value payload.
     * @return true if consumed by `SessionController`, false if caller should
     *         forward to domain-specific handlers (e.g. `WinderApp`).
     */
    bool handleCommand(const String& cmd, const String& value);

    /**
     * @brief Run one session update cycle.
     *
     * Order of operations:
     * 1. Integrate physical inputs (pot/footswitch), using edge detection.
     * 2. Decode and arbitrate queued commands.
     * 3. Apply exactly one pending intent (last-event wins).
     * 4. Apply output power command and tick `WinderApp`.
     *
     * @param in Aggregated inputs for this cycle.
     */
    void tick(const TickInput& in);

private:
    /** Non-owning reference to winding application logic. */
    WinderApp& _winder;
    /** Current high-level session state. */
    SessionState _sessionState = SessionState::IDLE;
    /** Source that last produced an applied transition. */
    InputSource _appliedSource = InputSource::None;
    /** Source of latest observed event, even before application. */
    InputSource _lastEventSource = InputSource::None;

    /** Latest normalized potentiometer value. */
    float _potLevel = 0.0f;
    /** Cached digital state derived from pot threshold crossing. */
    bool _potAboveZero = false;
    /** Latest footswitch sampled value. */
    bool _footswitch = false;

    /** Guards first pot sample to initialize edge detector safely. */
    bool _hasPotSample = false;
    /** Guards first footswitch sample to initialize edge detector safely. */
    bool _hasFootswitchSample = false;

    /**
     * @brief Pot ownership lock.
     *
     * When another source triggers an intent while pot is above threshold,
     * pot start is blocked until it returns below threshold once.
     */
    bool _potNeedsRearm = false;

    /** Pending intent selected by last-event arbitration. */
    ControlIntent _pendingControlIntent = ControlIntent::None;
    /** Source attached to `_pendingControlIntent`. */
    InputSource _pendingIntentSource = InputSource::None;

    enum class RunMode { None, Max, Pot };

    /** Current run mode used by applyPower. */
    RunMode _runMode = RunMode::None;

    /**
     * @brief Record a new intent and override previously pending one.
     * @param intent Intent to stage.
     * @param src Event source that generated the intent.
     */
    void recordIntent(ControlIntent intent, InputSource src);

    /**
     * @brief Apply staged intent to the session state machine and `WinderApp`.
     */
    void applyPendingIntent();

    /**
     * @brief Apply speed output according to current session state and pot value.
     *
     * RUNNING -> set speed proportional to pot.
     * Non-RUNNING -> force speed to zero.
     */
    void applyPower();
};
