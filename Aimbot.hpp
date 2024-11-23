#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <array>
#include <thread>
#include <chrono>
#include <random>
#include <cmath>

#include "HidTable.hpp"
#include "KmboxNet.hpp"
#include "KmboxB.h"

#include "Player.hpp"
#include "LocalPlayer.hpp"
#include "Offsets.hpp"
#include "Camera.hpp"

#include "Vector2D.hpp"
#include "Vector3D.hpp"
#include "QAngle.hpp"
#include "Resolver.hpp"

#include "DMALibrary/Memory/Memory.h"
#include "Conversion.hpp"
#include "HitboxType.hpp"

// Conversion
#define DEG2RAD( x  )  ( (float)(x) * (float)(M_PI / 180.f) )

const float DRAG_COEFFICIENT = 0.47f; // Approximate drag coefficient for a sphere
const float AIR_DENSITY = 1.225f; // kg/m^3, at sea level and 15°C
const float GRAVITY = 9.81f; // m/s^2

namespace AimbotUtils {
    inline float RandomFloat() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<> dis(0, 1);
        return static_cast<float>(dis(gen));
    }
}

struct Aimbot {
    bool PredictMovement = true;    // Enables movement prediction for targets
    bool PredictBulletDrop = true;  // Enables bullet drop compensation

    float FinalDistance = 0;        // Calculated final aiming distance

    bool Sticky = false;            // If true, stays locked on the current target
    float Smooth = 0.1f;            // Smoothing factor for aim movement (0.1 = fast, jerky; 1.0 = slow, smooth)
    const float EXTRA_SMOOTHING = 2.5f;     // Additional smoothing applied (higher = smoother)
    const float SPEED_MULTIPLIER = 5.0f;    // Multiplier for aim speed (higher = faster)
    const float MAX_SMOOTH_INCREASE = 0.15f;// Max increase in smoothing (higher = more variable smoothing)
    float FOV = 12.0f;              // Field of View for target acquisition (smaller = more precise, larger = easier to acquire)
    const float ZOOM_SCALE = 1.25f; // FOV adjustment when zoomed in (higher = more zoom effect)
    float MinDistance = 0;          // Minimum distance to start aiming
    float HipfireDistance = 50;     // Maximum distance for hipfire aiming
    float ZoomDistance = 300;       // Maximum distance for zoomed aiming
    int MinimumDelay = 0.1;           // Minimum delay between aim adjustments (lower = faster updates)
    float RecoilCompensation = 1.2f;// Recoil control strength (higher = stronger compensation)

    int AimBotKey = 0x02;           // Key to activate aimbot (0x02 = right mouse button)
    int AimTriggerKey = 0x05;       // Key to activate aim trigger (0x05 = mouse button 5)
    int AimFlickKey = 0x06;         // Key to activate aim flick (0x06 = mouse button 6)

    bool TriggerBotEnabled = false; // Enables/disables triggerbot functionality
    int TriggerBotKey = 0x05;       // Key to activate triggerbot (0x05 = mouse button 5)
    float TriggerFOV = 5.0f;        // FOV for triggerbot (smaller = more precise)
    float TriggerMagnetStrength = 5.5f; // Strength of triggerbot's magnetic effect (higher = stronger pull)
    float TriggerAccuracyThreshold = 10.0f; // Accuracy threshold for triggerbot (higher = more precise)
    bool TriggerBotUseAllAimbones = false; // If true, considers all bones for triggerbot
    std::vector<HitboxType> TriggerBotSelectedAimbones; // Specific bones for triggerbot to target

    // Advanced magnetic effect settings
    float AdvancedMagnetStrength = 2.0f; // Strength of advanced magnetic effect (0.1 = weak, 1.0 = strong)

    // Constants for advanced magnetic effect
    static constexpr float ADVANCED_MAGNET_CURVATURE = 10.0f; // Curvature of magnetic effect (higher = sharper curve)
    static constexpr float ADVANCED_MAGNET_FALLOFF = 0.1f;   // Falloff rate of magnetic effect (higher = faster falloff)
    static constexpr float ADVANCED_SMOOTHNESS = .0f;       // Smoothness of magnetic effect (0.1 = jerky, 1.0 = very smooth)

    // Constants for Taylor series expansion
    static constexpr int TAYLOR_TERMS = 10;
    static constexpr float TAYLOR_EPSILON = 1e-6f;

    LocalPlayer* Myself;
    std::vector<Player*>* Players;
    Camera* GameCamera;

    Player* CurrentTarget = nullptr;
    Player* LockedTarget = nullptr;
    bool TargetSelected = true;
    bool IsTargetLocked = false;
    std::chrono::milliseconds LastAimTime{ 0 };

    // Add new members for Voronoi Diagram
    std::vector<Vector3D> voronoiSites;
    const int voronoiSiteCount = 5;

    // Add these new members
    const int POSITION_HISTORY_SIZE = 10;
    std::vector<Vector3D> targetPositionHistory;
    std::vector<Vector3D> targetVelocityHistory;
    float lastUpdateTime = 0.0f;

    enum class MovementPattern {
        Unknown,
        Linear,
        Strafing,
        Zigzag,
        Circular,
        Jumping,
        Falling,
        ComplexVertical
    };

    MovementPattern currentPattern = MovementPattern::Unknown;
    MovementPattern previousPattern = MovementPattern::Unknown;
    float patternConfidence = 0.0f;
    Vector3D patternDirection;
    float patternFrequency = 0.0f;
    float patternAmplitude = 0.0f;
    float patternTransitionSmoothing = 0.0f;
    float verticalVelocity = 0.0f;
    bool isJumping = false;
    float jumpStartTime = 0.0f;

    // Constants for fine-tuning (adjust based on your game's mechanics)
    const float STRAFE_THRESHOLD = 0.3f;
    const float ZIGZAG_THRESHOLD = 0.4f;
    const float CIRCULAR_THRESHOLD = 0.6f;
    const float VERTICAL_MOVEMENT_THRESHOLD = 1.0f; // Units per second
    const float JUMP_DETECTION_THRESHOLD = 3.0f; // Units per second
    const float PATTERN_TRANSITION_SPEED = 0.1f;

    bool isInitialized = false;

    std::vector<HitboxType> SelectedAimbones;
    bool UseAllAimbones = false;

    // Static members for trigger lock
    static inline Player* s_TriggerLockedTarget = nullptr;
    static inline bool s_IsTriggerLocked = false;
    static constexpr float TRIGGER_LOCK_DURATION = 5.0f; // Increased to 5 seconds
    static inline float s_TriggerLockStartTime = 0.0f;

    const float CLOSE_RANGE_THRESHOLD = 5.0f; // 5 meters
    const float CLOSE_RANGE_FIRE_FOV = 10.0f; // Smaller FOV for firing at close range

    bool TacticalReloadEnabled = false; // New member to enable/disable tactical reload
    int TacticalReloadKey = 0x52; // Default to 'R' key (0x52 is the virtual key code for 'R')

    Aimbot(LocalPlayer* Myself, std::vector<Player*>* GamePlayers, Camera* Cam) {
        this->Myself = Myself;
        this->Players = GamePlayers;
        this->GameCamera = Cam;
    }

    std::string KmboxType = "Net";
    char KmboxIP[24] = "192.168.2.188";
    char KmboxPort[10] = "8336";
    char KmboxUUID[32] = "24B75054";
    int KmboxComPort = 0;

    _com comPort;

    void Initialize() {
        int choice;
        std::cout << "Choose initialization option:" << std::endl;
        std::cout << "1. Initialize KmNet" << std::endl;
        std::cout << "2. Initialize KmBox" << std::endl;
        std::cout << "3. Skip initialization and continue" << std::endl;
        std::cout << "Enter your choice (1-3): ";
        std::cin >> choice;

        switch (choice) {
        case 1:
            InitializeKmboxNet();
            break;
        case 2:
            InitializeKmboxBPro();
            break;
        case 3:
            std::cout << "Skipping initialization and continuing..." << std::endl;
            isInitialized = true;
            break;
        default:
            std::cout << "Invalid choice. Skipping initialization and continuing..." << std::endl;
            isInitialized = true;
            break;
        }
    }

    void InitializeKmboxNet() {
        std::cout << "Initializing Kmbox Net..." << std::endl;
        MinimumDelay = 1;

        int maxAttempts = 3;
        int attempt = 0;
        int result;

        while (attempt < maxAttempts) {
            result = kmNet_init(KmboxIP, KmboxPort, KmboxUUID);
            if (result == 0) {
                std::cout << "Kmbox Net initialized successfully." << std::endl;
                isInitialized = true;
                return;
            }
            attempt++;
            std::cout << "Kmbox Net initialization failed. Attempt " << attempt << " of " << maxAttempts << std::endl;
            Sleep(1000); // Wait for 1 second before retrying
        }

        std::cout << "Kmbox Net failed to initialize after " << maxAttempts << " attempts." << std::endl;
        std::cout << "Please check your network connection and Kmbox settings." << std::endl;
    }

    void InitializeKmboxBPro() {
        std::cout << "Initializing Kmbox B+ Pro..." << std::endl;
        MinimumDelay = 3;

        int maxAttempts = 3;
        int attempt = 0;

        while (attempt < maxAttempts) {
            if (comPort.open(KmboxComPort, 115200)) {
                std::cout << "Kmbox B+ Pro initialized successfully." << std::endl;
                isInitialized = true;
                return;
            }
            attempt++;
            std::cout << "Kmbox B+ Pro initialization failed. Attempt " << attempt << " of " << maxAttempts << std::endl;
            Sleep(1000); // Wait for 1 second before retrying
        }

        std::cout << "Kmbox B+ Pro failed to initialize after " << maxAttempts << " attempts." << std::endl;
        std::cout << "Please check your COM port settings and Kmbox connection." << std::endl;
    }

    void Move(int x, int y) {
        if (!isInitialized) {
            std::cout << "Aimbot is not initialized. Please initialize before moving." << std::endl;
            return;
        }

        if (KmboxType == "Net") {
            kmNet_mouse_move(x, y);
        }
        else if (KmboxType == "BPro") {
            char cmd[1024] = { 0 };
            sprintf_s(cmd, "km.move(%d, %d, 10)\r\n", x, y);
            comPort.write(cmd);
        }
    }

    void Reload() {
        if (KmboxType == "Net") {
            if (Myself->IsInAttack) {
                kmNet_mask_mouse_left(1);
                kmNet_mouse_left(0);
            }

            kmNet_keydown(KEY_R);
            Sleep((int)Utils::RandomRange(MinimumDelay, 10));
            kmNet_keyup(KEY_R);
            kmNet_mask_mouse_left(0);
        }
        else if (KmboxType == "BPro") {
            char cmd[1024] = { 0 };
            sprintf_s(cmd, "km.press('r')\r\n");
            comPort.write(cmd);
        }
    }

    void LeftClick() {
        if (KmboxType == "Net") {
            kmNet_mouse_left(1);
            Sleep((int)Utils::RandomRange(MinimumDelay, 10));
            kmNet_mouse_left(0);
        }
        else if (KmboxType == "BPro") {
            char cmd[1024] = { 0 };
            sprintf_s(cmd, "km.click(0)\r\n");
            comPort.write(cmd);
        }
    }

    void Update_TacticalReload() {
        if (!TacticalReloadEnabled) return; // Skip if tactical reload is disabled
        if (!Myself->IsCombatReady()) { return; }
        if (!Myself->IsReloading && !Myself->IsHoldingGrenade) {
            if (Myself->Ammo == 1) {
                Reload();
            }
        }
    }

    void Update_Aimbot() {
        if (Myself->IsZooming)
            FinalDistance = ZoomDistance;
        else FinalDistance = HipfireDistance;

        if (mem.GetKeyboard()->IsKeyDown(AimTriggerKey)) {
            return;
        }

        if (!Myself->IsCombatReady()) {
            CurrentTarget = nullptr; LockedTarget = nullptr; IsTargetLocked = false;
            return;
        }

        bool isMainAimKeyActive = mem.GetKeyboard()->IsKeyDown(AimBotKey);
        bool isAimFlickKeyActive = mem.GetKeyboard()->IsKeyDown(AimFlickKey);

        if (!isMainAimKeyActive && !isAimFlickKeyActive) {
            ReleaseTarget();
            return;
        }

        if (Myself->IsHoldingGrenade) {
            ReleaseTarget();
            return;
        }

        Player* Target = nullptr;

        if (IsTargetLocked && IsValidTarget(LockedTarget)) {
            Target = LockedTarget;
        }
        else {
            if (Sticky && IsValidTarget(CurrentTarget)) {
                Target = CurrentTarget;
            }
            else {
                Target = FindBestTarget();
            }

            if (!IsValidTarget(Target)) {
                ReleaseTarget();
                return;
            }

            CurrentTarget = Target;
            CurrentTarget->IsLockedOn = true;
            TargetSelected = true;

            if (isMainAimKeyActive || isAimFlickKeyActive) {
                LockedTarget = Target;
                IsTargetLocked = true;
            }
        }

        if (TargetSelected && Target) {
            std::chrono::milliseconds Now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            if (Now >= LastAimTime + std::chrono::milliseconds(10)) {
                if (Target->DistanceToLocalPlayer <= Conversion::ToGameUnits(CLOSE_RANGE_THRESHOLD)) {
                    StartAimingCloseRange(Target);
                }
                else {
                    StartAiming(Target);
                }
                LastAimTime = Now + std::chrono::milliseconds((int)Utils::RandomRange(MinimumDelay, 0.1));
            }
            return;
        }
    }
    void StartAiming(Player* Target) {
        Vector3D TargetPosition;
        if (IsTargetLocked && LockedTarget != nullptr) {
            TargetPosition = CalculatePredictedPosition(LockedTarget->GetBonePosition(static_cast<HitboxType>(GetBestBone(LockedTarget))), LockedTarget->AbsoluteVelocity, Myself->WeaponProjectileSpeed, Myself->WeaponProjectileScale);
        }
        else {
            TargetPosition = CalculatePredictedPosition(Target->GetBonePosition(static_cast<HitboxType>(GetBestBone(Target))), Target->AbsoluteVelocity, Myself->WeaponProjectileSpeed, Myself->WeaponProjectileScale);
        }



        Vector2D ScreenPosition = { 0, 0 };
        bool isOnScreen = GameCamera->WorldToScreen(TargetPosition, ScreenPosition);

        if (isOnScreen) {
            Vector2D Center = GameCamera->GetCenter();
            Vector2D RelativePosition = { ScreenPosition.x - Center.x, ScreenPosition.y - Center.y };



            // Calculate the magnitude of the relative position
            float relativeMagnitude = std::sqrt(RelativePosition.x * RelativePosition.x + RelativePosition.y * RelativePosition.y);

            // Normalize the relative position
            Vector2D NormalizedRelative = {
                RelativePosition.x / relativeMagnitude,
                RelativePosition.y / relativeMagnitude
            };

            float baseSmoothing = Smooth;
            float baseFOV = FOV;
            float maxPercentageIncrease = MAX_SMOOTH_INCREASE;
            float maxIncrease = baseSmoothing * maxPercentageIncrease;

            baseSmoothing += Utils::RandomRange(-0.05f, 0.05f);
            baseFOV = FOV + Utils::RandomRange(-1.0f, 1.0f);

            float distance = CalculateDistanceFromCrosshair(TargetPosition);

            float dynamicSmoothing = baseSmoothing;
            if (distance <= baseFOV) {
                float scale = 1.0f - (distance / baseFOV);
                dynamicSmoothing += maxIncrease * scale * 0.5f;
            }

            dynamicSmoothing = (std::max)(dynamicSmoothing, baseSmoothing);

            // Calculate step using the normalized relative position
            float stepMagnitude = (relativeMagnitude / (dynamicSmoothing * 0.6f)) * SPEED_MULTIPLIER;
            Vector2D step = {
                NormalizedRelative.x * stepMagnitude,
                NormalizedRelative.y * stepMagnitude * ((Myself->IsInAttack || mem.GetKeyboard()->IsKeyDown(VK_LBUTTON)) ? RecoilCompensation : 1.0f)
            };

            std::cout << "  Base Smoothing: " << baseSmoothing << std::endl;
            std::cout << "  Dynamic Smoothing: " << dynamicSmoothing << std::endl;
            std::cout << "  Distance from Crosshair: " << distance << std::endl;
            std::cout << "  FOV: " << baseFOV << std::endl;
            std::cout << "  Calculated Step: (" << step.x << ", " << step.y << ")" << std::endl;

            // Implement a minimum step threshold
            const float MIN_STEP_THRESHOLD = 0.1f;
            if (std::abs(step.x) < MIN_STEP_THRESHOLD && std::abs(step.y) < MIN_STEP_THRESHOLD) {
                std::cout << "  Step too small, applying minimum step" << std::endl;
                step.x = (step.x < 0) ? -MIN_STEP_THRESHOLD : MIN_STEP_THRESHOLD;
                step.y = (step.y < 0) ? -MIN_STEP_THRESHOLD : MIN_STEP_THRESHOLD;
            }

            std::cout << "  Moving: (" << step.x << ", " << step.y << ")" << std::endl;
            Move(step.x, step.y);
        }
        else {
            std::cout << "  Failed to convert world position to screen position" << std::endl;

            // Calculate the angle to the target
            Vector3D DirectionToTarget = TargetPosition.Subtract(Myself->CameraPosition);
            float DistanceToTarget = DirectionToTarget.Length();
            DirectionToTarget = DirectionToTarget.Normalize();

            QAngle TargetAngle = Resolver::CalculateAngle(Myself->CameraPosition, TargetPosition);
            QAngle CurrentAngle = QAngle(Myself->ViewAngles.x, Myself->ViewAngles.y).NormalizeAngles();
            QAngle DeltaAngle = (TargetAngle - CurrentAngle).NormalizeAngles();

            std::cout << "  Distance to Target: " << DistanceToTarget << std::endl;
            std::cout << "  Angle to Target: (" << DeltaAngle.x << ", " << DeltaAngle.y << ")" << std::endl;

            // Adjust view angles to bring target into view
            float AngleAdjustmentSpeed = 10.0f; // Adjust this value to control turn speed
            Vector2D AngleAdjustment = {
                DeltaAngle.y * AngleAdjustmentSpeed,
                DeltaAngle.x * AngleAdjustmentSpeed
            };

            std::cout << "  Angle Adjustment: (" << AngleAdjustment.x << ", " << AngleAdjustment.y << ")" << std::endl;
            Move(AngleAdjustment.x, AngleAdjustment.y);
        }

        if (!mem.GetKeyboard()->IsKeyDown(AimFlickKey)) {
            return;
        }

        std::vector<int> bones = { 0, 3, 1, 4, 2 };
        for (int boneIndex : bones) {
            Vector3D bonePosition = CalculatePredictedPosition(Target->GetBonePosition(static_cast<HitboxType>(boneIndex)), Target->AbsoluteVelocity, Myself->WeaponProjectileSpeed, Myself->WeaponProjectileScale);

            float boxWidth, boxDepth, boxHeight;

            if (boneIndex == 0) { // Head
                boxWidth = boxDepth = boxHeight = 5.0; // Assuming the head is roughly a cube
            }
            else if (boneIndex == 3) { // Body
                boxWidth = 8.0; // X
                boxDepth = 8.0; // Y
                boxHeight = 12.0; // Z
            }
            else {
                continue;
            }

            // Calculate corner points of the box around the bone in world space
            std::vector<Vector3D> corners = {
                {bonePosition.x + boxWidth, bonePosition.y + boxDepth, bonePosition.z + boxHeight},
                {bonePosition.x - boxWidth, bonePosition.y - boxDepth, bonePosition.z - boxHeight},
                {bonePosition.x + boxWidth, bonePosition.y - boxDepth, bonePosition.z + boxHeight},
                {bonePosition.x - boxWidth, bonePosition.y + boxDepth, bonePosition.z - boxHeight},
            };

            float minX = FLT_MAX, maxX = -FLT_MAX, minY = FLT_MAX, maxY = -FLT_MAX;

            for (const auto& corner : corners) {
                Vector2D screenPos;
                if (GameCamera->WorldToScreen(corner, screenPos)) {
                    minX = (std::min)(minX, screenPos.x);
                    maxX = (std::max)(maxX, screenPos.x);
                    minY = (std::min)(minY, screenPos.y);
                    maxY = (std::max)(maxY, screenPos.y);
                }
            }

            Vector2D Center = GameCamera->GetCenter();
            if ((Center.x >= (minX - 1)) && (Center.x <= (maxX + 1)) &&
                (Center.y >= (minY - 1.5)) && (Center.y <= (maxY + 1.5))) {
                LeftClick();
                break;
            }
        }

        // Check if the aim button is released
        if (IsAimButtonReleased()) {
            ReleaseTarget();
        }
    }

    void StartAimingCloseRange(Player* Target) {
        Vector3D TargetPosition = Target->GetBonePosition(static_cast<HitboxType>(GetBestBone(Target)));

        Vector2D ScreenPosition = { 0, 0 };
        bool isOnScreen = GameCamera->WorldToScreen(TargetPosition, ScreenPosition);

        if (isOnScreen) {
            Vector2D Center = GameCamera->GetCenter();
            Vector2D RelativePosition = { ScreenPosition.x - Center.x, ScreenPosition.y - Center.y };

            // For close range, we'll use a more aggressive aim with less smoothing
            float closeRangeSmoothing = Smooth * 0.4f; // Reduce smoothing for faster aim

            Vector2D step = {
                RelativePosition.x / (closeRangeSmoothing * 0.5f),
                RelativePosition.y / (closeRangeSmoothing * 0.5f)
            };

            // Apply a minimum step threshold for close range
            const float MIN_CLOSE_RANGE_STEP = 0.1f;
            if (std::abs(step.x) < MIN_CLOSE_RANGE_STEP && std::abs(step.y) < MIN_CLOSE_RANGE_STEP) {
                step.x = (step.x < 0) ? -MIN_CLOSE_RANGE_STEP : MIN_CLOSE_RANGE_STEP;
                step.y = (step.y < 0) ? -MIN_CLOSE_RANGE_STEP : MIN_CLOSE_RANGE_STEP;
            }


            Move(step.x, step.y);

            // Check if we should fire
            if (ShouldFireCloseRange(RelativePosition)) {

                LeftClick();
            }
        }
    }

    bool ShouldFireCloseRange(const Vector2D& RelativePosition) {
        float distanceFromCrosshair = RelativePosition.Magnitude();
        return distanceFromCrosshair <= CLOSE_RANGE_FIRE_FOV;
    }


    void Update_Triggerbot() {
        if (!TriggerBotEnabled) return;
        if (!Myself->IsCombatReady()) { return; }
        if (!mem.GetKeyboard()->IsKeyDown(TriggerBotKey)) {
            ReleaseTriggerLock();
            return;
        }
        if (Myself->IsHoldingGrenade) {
            ReleaseTriggerLock();
            return;
        }

        float currentTime = GetCurrentTime();

        // Check if we need to release the lock
        if (s_IsTriggerLocked && (currentTime - s_TriggerLockStartTime > TRIGGER_LOCK_DURATION)) {
            ReleaseTriggerLock();
        }

        Player* Target = s_IsTriggerLocked ? s_TriggerLockedTarget : FindBestTargetForTrigger();

        if (Target != nullptr && Target->IsValid()) {
            if (Target->DistanceToLocalPlayer <= Conversion::ToGameUnits(CLOSE_RANGE_THRESHOLD)) {
                ApplyCloseRangeTriggerbot(Target);
            }
            else {
                // Existing triggerbot logic for longer ranges
                HitboxType bestBone = GetBestBoneForTrigger(Target);
                Vector3D targetBonePosition = Target->GetBonePosition(bestBone);
                Vector3D predictedPos = CalculatePredictedPosition(
                    Target->GetBonePosition(bestBone),
                    Target->AbsoluteVelocity,
                    Myself->WeaponProjectileSpeed,
                    Myself->WeaponProjectileScale
                );

                predictedPos = ApplyVoronoiAdjustment(predictedPos, Target->DistanceToLocalPlayer);

                if (IsTargetInTriggerFOV(predictedPos) || s_IsTriggerLocked) {
                    ApplyMagneticEffect(predictedPos);
                    if (IsAccurateEnoughToShoot(predictedPos)) {
                        LeftClick();
                    }

                    // Lock onto the target
                    if (!s_IsTriggerLocked) {
                        s_TriggerLockedTarget = Target;
                        s_IsTriggerLocked = true;
                        s_TriggerLockStartTime = currentTime;
                    }
                }
            }
        }
        else {
            ReleaseTriggerLock();
        }
    }

    static void ReleaseTriggerLock() {
        s_IsTriggerLocked = false;
        s_TriggerLockedTarget = nullptr;
    }

    bool IsTargetInTriggerFOV(const Vector3D& targetPosition) {
        Vector2D screenPos;
        if (!GameCamera->WorldToScreen(targetPosition, screenPos)) {
            return false;
        }

        Vector2D screenCenter = GameCamera->GetCenter();
        Vector2D aimVector = { screenPos.x - screenCenter.x, screenPos.y - screenCenter.y };
        float distanceFromCrosshair = aimVector.Magnitude();

        // Calculate the maximum distance based on FOV scaling
        float maxDistance = (TriggerFOV / GameCamera->FOV) * screenCenter.x;

        // Check if the target is very close to increase the maxDistance
        float distance = Myself->CameraPosition.Distance(targetPosition);
        if (distance < Conversion::ToGameUnits(10)) {
            maxDistance *= 2.0f; // Increase the max distance for close targets
        }

        // Return true if the target is within the adjusted FOV or if the trigger is locked
        return distanceFromCrosshair <= maxDistance || s_IsTriggerLocked;
    }

    void ApplyMagneticEffect(const Vector3D& targetPosition) {
        ApplyAdvancedMagneticEffect(targetPosition);
    }

    void ApplyAdvancedMagneticEffect(const Vector3D& targetPosition) {
        Vector2D screenPos;
        if (!GameCamera->WorldToScreen(targetPosition, screenPos)) {
            return;
        }

        Vector2D screenCenter = GameCamera->GetCenter();
        Vector2D aimVector = { screenPos.x - screenCenter.x, screenPos.y - screenCenter.y };

        float distanceFromCenter = aimVector.Magnitude();
        float maxDistance = (TriggerFOV / GameCamera->FOV) * screenCenter.x;

        // Normalized distance (0 at center, 1 at edge of FOV)
        float normalizedDistance = distanceFromCenter / maxDistance;

        // Calculate base magnetic force using Taylor series expansion
        float baseMagneticForce = TaylorSeriesExpansion(normalizedDistance);

        // Apply inverse square law falloff with discrete step function
        float falloffFactor = DiscreteInverseSquareLaw(normalizedDistance);

        // Calculate angular factor using Chebyshev polynomials
        float angularFactor = ChebyshevAngularFactor(normalizedDistance);

        // Combine factors for final magnetic force
        float magneticForce = AdvancedMagnetStrength * 1.5f * baseMagneticForce * falloffFactor * angularFactor; // Increased strength

        // Apply dynamic smoothing based on distance
        float smoothFactor = SecureExponentialSmoothing(ADVANCED_SMOOTHNESS * (1.0f - normalizedDistance)); // More smoothing when closer

        // Calculate move vector
        Vector2D moveVector = aimVector.Normalized().Multiply(-magneticForce);

        // Apply smoothing with overflow protection
        static Vector2D lastMoveVector(0, 0);
        moveVector = SecureVectorAddition(
            lastMoveVector.Multiply(1.0f - smoothFactor),
            moveVector.Multiply(smoothFactor)
        );
        lastMoveVector = moveVector;

        // Apply movement with bounds checking
        SecureMove(moveVector.x, moveVector.y);
    }

    float TaylorSeriesExpansion(float x) {
        float result = 1.0f;
        float term = 1.0f;
        for (int n = 1; n <= TAYLOR_TERMS; ++n) {
            term *= -x * ADVANCED_MAGNET_CURVATURE / n;
            if (std::abs(term) < TAYLOR_EPSILON) break;
            result += term;
        }
        return std::clamp(result, 0.0f, 1.0f); // Clamp result between 0 and 1
    }

    float DiscreteInverseSquareLaw(float x) {
        constexpr int STEPS = 10;
        float step = std::floor(x * STEPS) / STEPS;
        return 1.0f / (1.0f + std::pow(step * ADVANCED_MAGNET_FALLOFF, 4));
    }

    float ChebyshevAngularFactor(float x) {
        // Using Chebyshev polynomial of the first kind, degree 2
        return 0.5f * (1.0f - 2 * x * x);
    }

    float SecureExponentialSmoothing(float smoothness) {
        float frameTime = std::clamp(GetFrameTime(), 0.001f, 0.1f); // Bound frame time
        return 1.0f - std::pow(1.0f - smoothness, 60.0f * frameTime);
    }

    Vector2D SecureVectorAddition(const Vector2D& a, const Vector2D& b) {
        float x = std::clamp(a.x + b.x, -1000.0f, 1000.0f); // Arbitrary large bounds
        float y = std::clamp(a.y + b.y, -1000.0f, 1000.0f);
        return Vector2D(x, y);
    }

    void SecureMove(float x, float y) {
        x = std::clamp(x, -100.0f, 100.0f); // Arbitrary bounds for mouse movement
        y = std::clamp(y, -100.0f, 100.0f);
        Move(x, y);
    }

    bool IsAccurateEnoughToShoot(const Vector3D& targetPosition) {
        return IsAdvancedAccurateEnoughToShoot(targetPosition);
    }

    bool IsAdvancedAccurateEnoughToShoot(const Vector3D& targetPosition) {
        Vector2D screenPos;
        if (!GameCamera->WorldToScreen(targetPosition, screenPos)) {
            return false;
        }

        Vector2D screenCenter = GameCamera->GetCenter();
        Vector2D aimVector = { screenPos.x - screenCenter.x, screenPos.y - screenCenter.y };

        float distanceFromCrosshair = aimVector.Magnitude();
        float maxAccurateDistance = (TriggerFOV * TriggerAccuracyThreshold / GameCamera->FOV) * screenCenter.x;

        // Enhanced accuracy check with logistic function (discrete approximation)
        float normalizedDistance = distanceFromCrosshair / maxAccurateDistance;
        float logisticApprox = LogisticApproximation(normalizedDistance);

        return logisticApprox >= 0.1f;
    }

    float LogisticApproximation(float x) {
        // Piecewise linear approximation of logistic function
        const int SEGMENTS = 10;
        float normalizedX = std::clamp(x, 0.0f, 1.0f);
        float segment = std::floor(normalizedX * SEGMENTS) / SEGMENTS;
        return 1.0f - segment;
    }

    void ApplyCloseRangeTriggerbot(Player* Target) {
        HitboxType bestBone = GetBestBoneForTrigger(Target);
        Vector3D targetPosition = Target->GetBonePosition(bestBone);

        Vector2D screenPos;
        if (GameCamera->WorldToScreen(targetPosition, screenPos)) {
            Vector2D screenCenter = GameCamera->GetCenter();
            Vector2D aimVector = { screenPos.x - screenCenter.x, screenPos.y - screenCenter.y };

            float distanceFromCrosshair = aimVector.Magnitude();
            float closeRangeTriggerFOV = TriggerFOV * 3.5f; // Increase FOV for close range

            if (distanceFromCrosshair <= closeRangeTriggerFOV) {
                // Apply a more aggressive magnetic effect for close range
                float closeRangeMagnetStrength = TriggerMagnetStrength * 4.5f;
                Vector2D moveVector = aimVector.Normalized().Multiply(-closeRangeMagnetStrength);

                // Apply movement with reduced smoothing
                SecureMove(moveVector.x, moveVector.y);

                // Check if we're close enough to shoot
                if (distanceFromCrosshair <= closeRangeTriggerFOV * 10.0f) {
                    LeftClick();
                }
            }
        }
    }

    Vector3D CalculatePredictedPosition(const Vector3D& targetPosition, const Vector3D& targetVelocity, float bulletSpeed, float bulletScale) {
        Vector3D playerPosition = Myself->CameraPosition;
        float initialDistance = playerPosition.Distance(targetPosition);

        // Declare predictedPosition here
        Vector3D predictedPosition = targetPosition;

        // Update position and velocity history
        UpdateMovementHistory(targetPosition, targetVelocity);

        // Predict future position based on movement pattern
        Vector3D predictedVelocity = PredictFutureVelocity();
        Vector3D predictedAcceleration = EstimateTargetAcceleration();

        // Iterative prediction with dynamic time step
        const int MAX_ITERATIONS = 15;
        float timeStep = initialDistance / bulletSpeed / MAX_ITERATIONS;

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            Vector3D relativePosition = predictedPosition.Subtract(playerPosition);
            float distance = relativePosition.Length();

            // Calculate air resistance dynamically
            float bulletArea = 3.14159f * 0.00635f * 0.00635f;
            float airResistance = 0.5f * AIR_DENSITY * DRAG_COEFFICIENT * bulletArea * (bulletSpeed * bulletSpeed);
            bulletSpeed -= airResistance / Myself->Mass * timeStep; // Decrease speed due to resistance

            // Calculate bullet drop with air resistance
            float dropTime = timeStep;
            float bulletDrop = 0.5f * GRAVITY * dropTime * dropTime;
            bulletDrop *= (1.0f - (airResistance / (Myself->Mass * GRAVITY)));

            // Update predictedPosition
            predictedPosition = predictedPosition
                .Add(predictedVelocity.Multiply(timeStep))
                .Add(predictedAcceleration.Multiply(0.5f * timeStep * timeStep));
            predictedPosition.z -= bulletDrop;

            // Recalculate time for next iteration
            timeStep = distance / bulletSpeed / (MAX_ITERATIONS - i - 1);
        }

        // Apply Voronoi adjustment for fine-tuning
        return ApplyVoronoiAdjustment(predictedPosition, initialDistance);
    }

    void UpdateMovementHistory(const Vector3D& currentPosition, const Vector3D& currentVelocity) {
        float currentTime = GetCurrentTime(); // Implement this to get the current game time
        float deltaTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        targetPositionHistory.push_back(currentPosition);
        targetVelocityHistory.push_back(currentVelocity);

        if (targetPositionHistory.size() > POSITION_HISTORY_SIZE) {
            targetPositionHistory.erase(targetPositionHistory.begin());
            targetVelocityHistory.erase(targetVelocityHistory.begin());
        }
    }

    Vector3D PredictFutureVelocity() {
        if (targetVelocityHistory.size() < 2) return targetVelocityHistory.back();

        Vector3D averageVelocity;
        Vector3D velocityTrend;

        for (size_t i = 1; i < targetVelocityHistory.size(); i++) {
            averageVelocity = averageVelocity.Add(targetVelocityHistory[i]);
            velocityTrend = velocityTrend.Add(targetVelocityHistory[i].Subtract(targetVelocityHistory[i - 1]));
        }

        averageVelocity = averageVelocity.Divide(targetVelocityHistory.size() - 1);
        velocityTrend = velocityTrend.Divide(targetVelocityHistory.size() - 1);

        // Predict future velocity based on recent trend
        return averageVelocity.Add(velocityTrend.Multiply(4.0f)); // Adjust multiplier as needed
    }

    Vector3D EstimateTargetAcceleration() {
        if (targetVelocityHistory.size() < 3) return Vector3D(0, 0, 0);

        Vector3D averageAcceleration;
        for (size_t i = 2; i < targetVelocityHistory.size(); i++) {
            Vector3D prevVelocityChange = targetVelocityHistory[i - 1].Subtract(targetVelocityHistory[i - 2]);
            Vector3D currentVelocityChange = targetVelocityHistory[i].Subtract(targetVelocityHistory[i - 1]);
            averageAcceleration = averageAcceleration.Add(currentVelocityChange.Subtract(prevVelocityChange));
        }

        return averageAcceleration.Divide(targetVelocityHistory.size() - 2);
    }

    Vector3D ApplyVoronoiAdjustment(const Vector3D& position, float distance) {
        if (voronoiSites.empty()) {
            GenerateVoronoiSites();
        }

        Vector3D nearestSite = voronoiSites[0];
        float nearestDistance = position.Distance(nearestSite);

        for (const auto& site : voronoiSites) {
            float siteDistance = position.Distance(site);
            if (siteDistance < nearestDistance) {
                nearestSite = site;
                nearestDistance = siteDistance;
            }
        }

        // More aggressive adjustment factor calculation
        float distanceFactor = max(0.0f, min(1.0f, 1.0f - (distance / 300.0f))); // Increased the denominator for less falloff with distance
        float adjustmentStrength = 0.4f * distanceFactor; // Doubled the base strength

        Vector3D adjustment = nearestSite.Subtract(position).Multiply(adjustmentStrength);
        return position.Add(adjustment);
    }

    bool IsValidTarget(Player* target) {
        if (target == nullptr ||
            !target->IsValid() ||
            !target->IsCombatReady() ||
            !target->IsVisible ||
            !target->IsHostile ||
            target->Distance2DToLocalPlayer > Conversion::ToGameUnits(FinalDistance))
            return false;
        return true;
    }

    // dma/Aimbot.hpp

    double CalculateDistanceFromCrosshair(Vector3D TargetPosition) {
        Vector3D CameraPosition = Myself->CameraPosition;
        QAngle CurrentAngle = QAngle(Myself->ViewAngles.x, Myself->ViewAngles.y).NormalizeAngles();

        // Use a more precise threshold for distance comparison to handle very close distances better
        const double EPSILON = 1e-5;
        if (CameraPosition.Distance(TargetPosition) <= EPSILON)
            return 0; // Return 0 to indicate no distance instead of -1 which might be interpreted as an error

        QAngle TargetAngle = Resolver::CalculateAngle(CameraPosition, TargetPosition);
        if (!TargetAngle.isValid())
            return -1; // Maintain returning -1 for invalid angles

        // Calculate the angular distance more accurately
        double angularDistance = CurrentAngle.distanceTo(TargetAngle);

        // Optionally, consider adjustments or scaling based on specific game mechanics or requirements
        return angularDistance;
    }

    void ReleaseTarget() {
        if (CurrentTarget != nullptr && CurrentTarget->IsValid())
            CurrentTarget->IsLockedOn = false;

        TargetSelected = false;
        CurrentTarget = nullptr;
        LockedTarget = nullptr;
        IsTargetLocked = false;
    }

    float GetFOVScale() {
        if (Myself->IsValid()) {
            if (Myself->IsZooming) {
                if (Myself->TargetZoomFOV > 1.0 && Myself->TargetZoomFOV < 90.0) {
                    return tanf(DEG2RAD(Myself->TargetZoomFOV) * (0.64285714285)) * ZOOM_SCALE; // Using the constant ZOOM_SCALE
                }
            }
        }
        return 1.0;
    }

    int GetBestBone(Player* Target) {
        if (UseAllAimbones) {
            return GetBestBoneFromAll(Target);
        }
        else if (!SelectedAimbones.empty()) {
            return GetBestBoneFromSelected(Target);
        }
        return 0; // Default to Head if no bones are selected
    }

    int GetBestBoneFromAll(Player* Target) {
        float NearestDistance = 999;
        int NearestBone = 0;
        for (int i = 0; i < static_cast<int>(HitboxType::RightLeg) + 1; i++) {
            HitboxType Bone = static_cast<HitboxType>(i);
            double DistanceFromCrosshair = CalculateDistanceFromCrosshair(Target->GetBonePosition(Bone));
            if (DistanceFromCrosshair < NearestDistance) {
                NearestBone = i;
                NearestDistance = DistanceFromCrosshair;
            }
        }
        return NearestBone;
    }

    int GetBestBoneFromSelected(Player* Target) {
        float NearestDistance = 999;
        int NearestBone = 0;
        for (const auto& Bone : SelectedAimbones) {
            double DistanceFromCrosshair = CalculateDistanceFromCrosshair(Target->GetBonePosition(Bone));
            if (DistanceFromCrosshair < NearestDistance) {
                NearestBone = static_cast<int>(Bone);
                NearestDistance = DistanceFromCrosshair;
            }
        }
        return NearestBone;
    }

    Player* FindBestTarget() {
        Player* NearestTarget = nullptr;
        float BestDistance = 9999;
        float BaseFOV = (std::min)(FOV, FOV * (GetFOVScale() * ZOOM_SCALE)); // Using the constant ZOOM_SCALE
        float CloseQuartersDistance = Conversion::ToGameUnits(10); // 10 meters, adjust as needed
        Vector3D CameraPosition = Myself->CameraPosition;
        for (int i = 0; i < Players->size(); i++) {
            Player* p = Players->at(i);
            if (!IsValidTarget(p)) continue;
            if (p->DistanceToLocalPlayer < Conversion::ToGameUnits(ZoomDistance)) {
                HitboxType BestBone = static_cast<HitboxType>(GetBestBone(p));
                Vector3D TargetPosition = p->GetBonePosition(BestBone);

                float Distance = CameraPosition.Distance(TargetPosition);
                float FOV = CalculateDistanceFromCrosshair(TargetPosition);

                // Adjust FOV for close quarters
                float AdjustedFOV = BaseFOV;
                if (Distance < CloseQuartersDistance) {
                    AdjustedFOV *= 10.0f; // Increase FOV by 20% for close targets
                }



                if (FOV > AdjustedFOV) continue;
                if (Distance > BestDistance) continue;

                NearestTarget = p;
                BestDistance = Distance;
            }
        }
        return NearestTarget;
    }

    Player* FindBestTargetForTrigger() {
        Player* bestTarget = nullptr;
        float bestFOV = TriggerFOV;

        for (int i = 0; i < Players->size(); i++) {
            Player* p = Players->at(i);
            if (!IsValidTarget(p)) continue;

            HitboxType bestBone = GetBestBoneForTrigger(p);
            Vector3D targetPosition = p->GetBonePosition(bestBone);
            Vector2D screenPos;
            if (!GameCamera->WorldToScreen(targetPosition, screenPos)) continue;

            Vector2D screenCenter = GameCamera->GetCenter();
            Vector2D aimVector = { screenPos.x - screenCenter.x, screenPos.y - screenCenter.y };
            float currentFOV = aimVector.Magnitude();

            if (currentFOV < bestFOV) {
                bestTarget = p;
                bestFOV = currentFOV;
            }
        }

        return bestTarget;
    }

    HitboxType GetBestBoneForTrigger(Player* Target) {
        if (TriggerBotUseAllAimbones) {
            return GetBestBoneFromAllForTrigger(Target);
        }
        else if (!TriggerBotSelectedAimbones.empty()) {
            return GetBestBoneFromSelectedForTrigger(Target);
        }
        return HitboxType::Head; // Default to Head if no bones are selected
    }

    HitboxType GetBestBoneFromAllForTrigger(Player* Target) {
        float NearestDistance = 999;
        HitboxType NearestBone = HitboxType::Head;
        for (int i = 0; i < static_cast<int>(HitboxType::RightLeg) + 1; i++) {
            HitboxType Bone = static_cast<HitboxType>(i);
            double DistanceFromCrosshair = CalculateDistanceFromCrosshair(Target->GetBonePosition(Bone));
            if (DistanceFromCrosshair < NearestDistance) {
                NearestBone = Bone;
                NearestDistance = DistanceFromCrosshair;
            }
        }
        return NearestBone;
    }

    HitboxType GetBestBoneFromSelectedForTrigger(Player* Target) {
        float NearestDistance = 999;
        HitboxType NearestBone = HitboxType::Head;
        for (const auto& Bone : TriggerBotSelectedAimbones) {
            double DistanceFromCrosshair = CalculateDistanceFromCrosshair(Target->GetBonePosition(Bone));
            if (DistanceFromCrosshair < NearestDistance) {
                NearestBone = Bone;
                NearestDistance = DistanceFromCrosshair;
            }
        }
        return NearestBone;
    }

    float CalculateBulletSpeedScale(float smoothValue) {
        return 1.00f - (smoothValue - 1) * 0.10f;
    }

    // dma/Aimbot.hpp

  // ... existing code ...

  // Enhanced CalculatePredictedPositionTaylor function with more accurate physics
    Vector3D CalculatePredictedPositionTaylor(Vector3D targetPosition, Vector3D targetVelocity, float bulletSpeed, float bulletScale) {
        Vector3D playerPosition = Myself->CameraPosition;
        Vector3D relativePosition = targetPosition.Subtract(playerPosition);
        Vector3D relativeVelocity = targetVelocity;
        float bulletSpeedScale = CalculateBulletSpeedScale(Smooth);

        float initialDistance = relativePosition.Length();
        float t = initialDistance / (bulletSpeed * bulletSpeedScale);

        // Enhanced Taylor series expansion
        Vector3D predictedPosition = targetPosition;
        float tPower = 1.0f;
        float factorial = 1.0f;

        for (int n = 1; n <= TAYLOR_TERMS; ++n) {
            tPower *= t;
            factorial *= n;

            Vector3D term;
            switch (n) {
            case 1:
                term = relativeVelocity.Multiply(tPower / factorial);
                break;
            case 2:
                term = relativeVelocity.Multiply(relativeVelocity.Dot(relativePosition) / (bulletSpeed * bulletSpeed)).Multiply(tPower / factorial);
                break;
            case 3:
                term = relativeVelocity.Multiply(relativeVelocity.Dot(relativeVelocity) / (bulletSpeed * bulletSpeed)).Multiply(tPower / factorial);
                break;
            case 4:
                term = relativeVelocity.Multiply(relativeVelocity.Dot(relativeVelocity) * relativeVelocity.Dot(relativePosition) / std::pow(bulletSpeed, 4)).Multiply(tPower / factorial);
                break;
            case 5:
                term = relativeVelocity.Multiply(std::pow(relativeVelocity.Dot(relativeVelocity), 2) / std::pow(bulletSpeed, 4)).Multiply(tPower / factorial);
                break;
            case 6:
                term = relativeVelocity.Multiply(std::pow(relativeVelocity.Dot(relativeVelocity), 2) * relativeVelocity.Dot(relativePosition) / std::pow(bulletSpeed, 6)).Multiply(tPower / factorial);
                break;
            case 7:
                term = relativeVelocity.Multiply(std::pow(relativeVelocity.Dot(relativeVelocity), 3) / std::pow(bulletSpeed, 6)).Multiply(tPower / factorial);
                break;
            case 8:
                term = relativeVelocity.Multiply(std::pow(relativeVelocity.Dot(relativeVelocity), 3) * relativeVelocity.Dot(relativePosition) / std::pow(bulletSpeed, 8)).Multiply(tPower / factorial);
                break;
            case 9:
                term = relativeVelocity.Multiply(std::pow(relativeVelocity.Dot(relativeVelocity), 4) / std::pow(bulletSpeed, 8)).Multiply(tPower / factorial);
                break;
            case 10:
                term = relativeVelocity.Multiply(std::pow(relativeVelocity.Dot(relativeVelocity), 4) * relativeVelocity.Dot(relativePosition) / std::pow(bulletSpeed, 10)).Multiply(tPower / factorial);
                break;
            }
            predictedPosition = predictedPosition.Add(term);
        }

        // Bullet drop compensation
        float drop = Resolver::GetBasicBulletDrop(playerPosition, predictedPosition, bulletSpeed, bulletScale);
        predictedPosition.z += drop;

        // Apply Voronoi-based adjustment
        predictedPosition = ApplyVoronoiAdjustment(predictedPosition, initialDistance);

        return predictedPosition;
    }


    void GenerateVoronoiSites() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-100.0, 100.0); // Expanded range for more coverage

        voronoiSites.clear();
        int voronoiSiteCount = 100; // Further increased from 50 to 100 for higher density
        for (int i = 0; i < voronoiSiteCount; ++i) {
            voronoiSites.emplace_back(dis(gen), dis(gen), dis(gen) * 0.2f);
        }
    }

    bool IsAimButtonReleased() {
        return !mem.GetKeyboard()->IsKeyDown(AimBotKey) &&

            !mem.GetKeyboard()->IsKeyDown(AimFlickKey) &&
            !Myself->IsInAttack;
    }

    void AnalyzeMovementPattern() {
        if (targetPositionHistory.size() < POSITION_HISTORY_SIZE) return;

        Vector3D averageDirection;
        float totalDistance = 0.0f;
        float maxDeviation = 0.0f;
        int directionChanges = 0;
        float maxVerticalChange = 0.0f;

        for (size_t i = 1; i < targetPositionHistory.size(); i++) {
            Vector3D movement = targetPositionHistory[i].Subtract(targetPositionHistory[i - 1]);
            float distance = movement.Length();
            totalDistance += distance;

            if (distance > 0.001f) {
                Vector3D direction = movement.Normalize();
                averageDirection = averageDirection.Add(direction);

                // Check for direction changes (for zigzag detection)
                if (i > 1) {
                    float dotProduct = direction.Dot(targetPositionHistory[i - 1].Subtract(targetPositionHistory[i - 2]).Normalize());
                    if (dotProduct < -ZIGZAG_THRESHOLD) {
                        directionChanges++;
                    }
                }

                // Calculate deviation from average direction (for circular motion detection)
                float deviation = std::abs(direction.Dot(averageDirection.Normalize()) - 1.0f);
                maxDeviation = max(maxDeviation, deviation);

                // Track vertical movement
                float verticalChange = std::abs(movement.z);
                maxVerticalChange = max(maxVerticalChange, verticalChange);
            }
        }

        averageDirection = averageDirection.Normalize();
        float straightLineDistance = targetPositionHistory.back().Subtract(targetPositionHistory.front()).Length();
        float straightness = straightLineDistance / totalDistance;

        // Vertical movement analysis
        float averageVerticalVelocity = (targetPositionHistory.back().z - targetPositionHistory.front().z) /
            (POSITION_HISTORY_SIZE * GetFrameTime());
        verticalVelocity = averageVerticalVelocity;

        // Pattern recognition logic
        MovementPattern newPattern;
        float newConfidence;

        if (std::abs(averageVerticalVelocity) > JUMP_DETECTION_THRESHOLD) {
            if (averageVerticalVelocity > 0) {
                newPattern = MovementPattern::Jumping;
                isJumping = true;
                jumpStartTime = GetCurrentTime();
            }
            else {
                newPattern = MovementPattern::Falling;
            }
            newConfidence = min(1.0f, std::abs(averageVerticalVelocity) / JUMP_DETECTION_THRESHOLD);
        }
        else if (maxVerticalChange > VERTICAL_MOVEMENT_THRESHOLD) {
            newPattern = MovementPattern::ComplexVertical;
            newConfidence = min(1.0f, maxVerticalChange / VERTICAL_MOVEMENT_THRESHOLD);
        }
        else if (straightness > 1.0f - STRAFE_THRESHOLD) {
            newPattern = MovementPattern::Linear;
            newConfidence = straightness;
        }
        else if (directionChanges > POSITION_HISTORY_SIZE * ZIGZAG_THRESHOLD) {
            newPattern = MovementPattern::Zigzag;
            newConfidence = min(1.0f, static_cast<float>(directionChanges) / (POSITION_HISTORY_SIZE * ZIGZAG_THRESHOLD));
        }
        else if (maxDeviation > CIRCULAR_THRESHOLD) {
            newPattern = MovementPattern::Circular;
            newConfidence = min(1.0f, maxDeviation / CIRCULAR_THRESHOLD);
        }
        else {
            newPattern = MovementPattern::Strafing;
            newConfidence = 1.0f - straightness;
        }

        // Smooth transition between patterns
        if (newPattern != currentPattern) {
            previousPattern = currentPattern;
            patternTransitionSmoothing = 0.0f;
        }
        currentPattern = newPattern;
        patternConfidence = Lerp(patternConfidence, newConfidence, PATTERN_TRANSITION_SPEED);
        patternTransitionSmoothing = min(1.0f, patternTransitionSmoothing + PATTERN_TRANSITION_SPEED);

        // Update pattern parameters
        patternDirection = averageDirection;
        patternFrequency = static_cast<float>(directionChanges) / POSITION_HISTORY_SIZE;
        patternAmplitude = totalDistance / max(1, directionChanges);
    }

    void AdjustPredictionForPattern(Vector3D& predictedPosition, float time) {
        Vector3D adjustment(0, 0, 0);

        switch (currentPattern) {
        case MovementPattern::Linear:
            // Linear prediction is already handled by base prediction
            break;
        case MovementPattern::Strafing:
        {
            float strafeOffset = sin(time * 2.0f * M_PI) * patternAmplitude;
            adjustment = patternDirection.Cross(Vector3D(0, 0, 1)).Multiply(strafeOffset);
        }
        break;
        case MovementPattern::Zigzag:
        {
            float zigzagOffset = sin(time * patternFrequency * 2.0f * M_PI) * patternAmplitude;
            Vector3D zigzagDirection = patternDirection.Cross(Vector3D(0, 0, 1)).Normalize();
            adjustment = zigzagDirection.Multiply(zigzagOffset);
        }
        break;
        case MovementPattern::Circular:
        {
            float angle = time * 2.0f * static_cast<float>(M_PI) * patternFrequency;
            adjustment = Vector3D(
                cos(angle) * patternAmplitude,
                sin(angle) * patternAmplitude,
                0
            );
        }
        break;
        case MovementPattern::Jumping:
        {
            float jumpTime = GetCurrentTime() - jumpStartTime;
            float jumpHeight = CalculateJumpHeight(jumpTime);
            adjustment = Vector3D(0, 0, jumpHeight);
        }
        break;
        case MovementPattern::Falling:
        {
            adjustment = Vector3D(0, 0, 0.5f * GRAVITY * time * time);
        }
        break;
        case MovementPattern::ComplexVertical:
        {
            adjustment = Vector3D(0, 0, verticalVelocity * time);
        }
        break;
        case MovementPattern::Unknown:
            // No additional adjustment for unknown patterns
            break;
        }

        // Apply smoothing during pattern transitions
        if (patternTransitionSmoothing < 1.0f) {
            Vector3D previousAdjustment(0, 0, 0);
            // Calculate previous adjustment (simplified for brevity)
            // ... (similar switch statement for previousPattern)

            adjustment = Lerp(previousAdjustment, adjustment, patternTransitionSmoothing);
        }

        predictedPosition = predictedPosition.Add(adjustment.Multiply(patternConfidence));
    }

    float CalculateJumpHeight(float time) {
        // Simplified jump height calculation (adjust based on your game's jump mechanics)
        const float MAX_JUMP_HEIGHT = 5.0f;
        const float JUMP_DURATION = 1.0f;
        return MAX_JUMP_HEIGHT * sin((M_PI * time) / JUMP_DURATION);
    }

    Vector3D Lerp(const Vector3D& a, const Vector3D& b, float t) {
        return a.Multiply(1.0f - t).Add(b.Multiply(t));
    }

    float Lerp(float a, float b, float t) {
        return a * (1.0f - t) + b * t;
    }

    float GetFrameTime() {
        static float time = 0.0f;
        time += 0.01f; // Increment time (adjust this value to change oscillation speed)

        // Oscillate between 60 and 240 FPS
        float fps = 150.0f + 90.0f * sin(time);

        // Clamp FPS between 60 and 240
        fps = fmax(60.0f, fmin(240.0f, fps));

        return 1.0f / fps;
    }

    float GetCurrentTime() {
        static float currentTime = 0.0f;
        currentTime += GetFrameTime();
        return currentTime;
    }

    void Update() {
        if (!isInitialized) {
            std::cout << "Aimbot is not initialized. Please initialize before updating." << std::endl;
            return;
        }

        Update_TacticalReload();
        Update_Aimbot();
        if (TriggerBotEnabled) { // Only update triggerbot if it's enabled
            Update_Triggerbot();
        }
    }
};