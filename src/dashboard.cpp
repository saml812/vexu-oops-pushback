#include "dashboard.h"

#include "configure.h"
#include "liblvgl/lvgl.h"
#include "robotActions.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

extern "C"
{
    extern const lv_image_dsc_t bm2_small;
}

namespace
{
    constexpr int UI_UPDATE_MS = 120;
    constexpr int SCREEN_W = 480;
    constexpr int SCREEN_H = 240;
    constexpr int TOP_BAR_H = 30;
    constexpr int PANEL_MARGIN = 8;
    constexpr int PANEL_GAP = 8;
    constexpr int CONTENT_Y = TOP_BAR_H + 4;
    constexpr int PANEL_W = (SCREEN_W - (2 * PANEL_MARGIN) - PANEL_GAP) / 2;
    constexpr int PANEL_H = SCREEN_H - CONTENT_Y - PANEL_MARGIN;

    constexpr int INFO_X = PANEL_MARGIN;
    constexpr int INFO_Y = CONTENT_Y;
    constexpr int INFO_W = PANEL_W;
    constexpr int INFO_H = PANEL_H;

    constexpr int AUTON_X = INFO_X + INFO_W + PANEL_GAP;
    constexpr int AUTON_Y = CONTENT_Y;
    constexpr int AUTON_W = PANEL_W;
    constexpr int AUTON_H = PANEL_H;

    constexpr double MOTOR_TEMP_WARN_C = 55.0;
    constexpr int RUNNING_VOLTAGE_THRESHOLD_MV = 1000;
    constexpr std::uint32_t FLASH_ON_MS = 180;
    constexpr std::uint32_t FLASH_OFF_MS = 140;
    constexpr int FLASH_PULSE_COUNT = 3;
    constexpr int FLASH_REQ_IDLE = 0;
    constexpr int FLASH_REQ_GREEN = 1;
    constexpr int FLASH_REQ_RED = 2;

    constexpr int AUTON_OPTION_COUNT = 3;
    constexpr std::array<AutonRoutine, AUTON_OPTION_COUNT> AUTON_OPTIONS = {
        AutonRoutine::DoNothing,
        AutonRoutine::Left,
        AutonRoutine::Right};

    constexpr double WHEEL_DIAMETER_IN = 2.25;
    constexpr double DRIVE_GEAR_RATIO = 1.0;
    constexpr double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_IN * 0.0254 * 3.14159265358979323846;
    constexpr double RPM_TO_MPS = (WHEEL_CIRCUMFERENCE_M * DRIVE_GEAR_RATIO) / 60.0;

    std::atomic<DashboardMode> requestedMode{DashboardMode::Disabled};
    std::atomic<AutonRoutine> selectedAuton{AutonRoutine::DoNothing};
    std::atomic<DriverProfile> selectedProfile{DriverProfile::White};
    std::atomic<bool> showImageRequested{false};
    std::atomic<int> pendingFlashRequest{FLASH_REQ_IDLE};

    enum class ImuOverlayState
    {
        Hidden,
        Calibrating,
        Success,
        Failed
    };

    struct Telemetry
    {
        bool initialized = false;
        std::uint32_t lastTimeMs = 0;
        double lastSpeedMps = 0.0;
        double speedMps = 0.0;
        double accelMps2 = 0.0;
    };

    Telemetry telemetry;

    struct MotorRunSummary
    {
        int runningCount = 0;
        int direction = 0;
    };

    struct FlashState
    {
        bool active = false;
        bool visible = false;
        int pulsesShown = 0;
        std::uint32_t phaseStartMs = 0;
        lv_color_t color = lv_color_hex(0x155F2A);
    };

    FlashState flashState;

    struct Ui
    {
        lv_obj_t *mainLayer = nullptr;
        lv_obj_t *imageLayer = nullptr;
        lv_obj_t *imageObj = nullptr;
        lv_obj_t *imageHintLabel = nullptr;
        lv_obj_t *imuOverlay = nullptr;
        lv_obj_t *imuOverlayLabel = nullptr;
        lv_obj_t *imuOverlayExitButton = nullptr;
        lv_obj_t *flashOverlay = nullptr;

        lv_obj_t *modeLabel = nullptr;
        lv_obj_t *statusLabel = nullptr;
        lv_obj_t *tempDot = nullptr;
        lv_obj_t *profileWhiteButton = nullptr;
        lv_obj_t *profilePinkButton = nullptr;
        lv_obj_t *showImageButton = nullptr;

        lv_obj_t *infoCard = nullptr;
        lv_obj_t *infoLabel = nullptr;

        lv_obj_t *autonCard = nullptr;
        lv_obj_t *autonSelectedLabel = nullptr;
        lv_obj_t *autonList = nullptr;
        lv_obj_t *autonButtons[AUTON_OPTION_COUNT] = {nullptr, nullptr, nullptr};

        bool imageVisible = false;
        bool imuWasCalibrating = false;
        bool imuOverlayDismissed = false;
        ImuOverlayState imuOverlayState = ImuOverlayState::Hidden;
    };

    Ui ui;

    const char *modeText(DashboardMode mode)
    {
        switch (mode)
        {
        case DashboardMode::Autonomous:
            return "Autonomous";
        case DashboardMode::Driver:
            return "Driver";
        default:
            return "Disabled";
        }
    }

    const char *modeAbbrev(DashboardMode mode)
    {
        switch (mode)
        {
        case DashboardMode::Autonomous:
            return "AUT";
        case DashboardMode::Driver:
            return "DRV";
        default:
            return "DIS";
        }
    }

    const char *driveModeText(DriverProfile profile)
    {
        return profile == DriverProfile::Pink ? "Tank" : "Curvature";
    }

    MotorRunSummary summarizeMotorVoltages(const std::vector<std::int32_t> &voltages)
    {
        int runningCount = 0;
        int positiveCount = 0;
        int negativeCount = 0;

        for (const std::int32_t voltage : voltages)
        {
            if (std::abs(voltage) < RUNNING_VOLTAGE_THRESHOLD_MV)
            {
                continue;
            }

            runningCount++;
            if (voltage > 0)
            {
                positiveCount++;
            }
            else if (voltage < 0)
            {
                negativeCount++;
            }
        }

        int direction = 0;
        if (positiveCount > negativeCount)
        {
            direction = 1;
        }
        else if (negativeCount > positiveCount)
        {
            direction = -1;
        }

        return {runningCount, direction};
    }

    double average(const std::vector<double> &values)
    {
        if (values.empty())
        {
            return 0.0;
        }

        double sum = 0.0;
        for (double value : values)
        {
            sum += value;
        }

        return sum / static_cast<double>(values.size());
    }

    double averageAbs(const std::vector<double> &values)
    {
        if (values.empty())
        {
            return 0.0;
        }

        double sum = 0.0;
        for (double value : values)
        {
            sum += std::abs(value);
        }

        return sum / static_cast<double>(values.size());
    }

    double driveSpeedRpm()
    {
        std::vector<double> values = leftDrive.get_actual_velocity_all();
        const auto rightValues = rightDrive.get_actual_velocity_all();
        values.insert(values.end(), rightValues.begin(), rightValues.end());
        return averageAbs(values);
    }

    double driveSpeedMps()
    {
        return driveSpeedRpm() * RPM_TO_MPS;
    }

    double leftDriveAvgTempC()
    {
        return average(leftDrive.get_temperature_all());
    }

    double rightDriveAvgTempC()
    {
        return average(rightDrive.get_temperature_all());
    }

    double intakeAvgTempC()
    {
        return average({leftIntake.get_temperature(), rightIntake.get_temperature()});
    }

    double maxMotorTempC()
    {
        double maxTemp = leftIntake.get_temperature();
        maxTemp = std::max(maxTemp, rightIntake.get_temperature());

        for (double temp : leftDrive.get_temperature_all())
        {
            maxTemp = std::max(maxTemp, temp);
        }

        for (double temp : rightDrive.get_temperature_all())
        {
            maxTemp = std::max(maxTemp, temp);
        }

        return maxTemp;
    }

    void styleCard(lv_obj_t *card, lv_color_t color)
    {
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_set_style_border_width(card, 0, 0);
        lv_obj_set_style_bg_color(card, color, 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    }

    void stylePillButton(lv_obj_t *button, bool active)
    {
        lv_obj_set_style_radius(button, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(button, 0, 0);
        lv_obj_set_style_bg_color(button, active ? lv_color_hex(0x3B84FF) : lv_color_hex(0x2A3A4D), 0);
        lv_obj_set_style_text_color(button, lv_color_hex(0xECF3FF), 0);
        lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    }

    void styleProfileButton(lv_obj_t *button, DriverProfile profile, bool active)
    {
        if (button == nullptr)
        {
            return;
        }

        lv_obj_set_style_radius(button, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(button, 0, 0);
        lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);

        if (!active)
        {
            lv_obj_set_style_bg_color(button, lv_color_hex(0x2A3A4D), 0);
            lv_obj_set_style_text_color(button, lv_color_hex(0xECF3FF), 0);
            return;
        }

        if (profile == DriverProfile::White)
        {
            lv_obj_set_style_bg_color(button, lv_color_hex(0xF3F6FA), 0);
            lv_obj_set_style_text_color(button, lv_color_hex(0x111827), 0);
            return;
        }

        lv_obj_set_style_bg_color(button, lv_color_hex(0xC93C8F), 0);
        lv_obj_set_style_text_color(button, lv_color_hex(0xFAF1F7), 0);
    }

    void moveObjToFront(lv_obj_t *obj)
    {
        if (obj == nullptr)
        {
            return;
        }

        lv_obj_t *parent = lv_obj_get_parent(obj);
        if (parent == nullptr)
        {
            return;
        }

        const std::uint32_t childCount = lv_obj_get_child_count(parent);
        if (childCount == 0)
        {
            return;
        }

        lv_obj_move_to_index(obj, static_cast<int32_t>(childCount - 1));
    }

    lv_obj_t *createPillButton(lv_obj_t *parent, const char *text, int x, int y, int w, int h)
    {
        lv_obj_t *button = lv_button_create(parent);
        lv_obj_set_pos(button, x, y);
        lv_obj_set_size(button, w, h);
        stylePillButton(button, false);

        lv_obj_t *label = lv_label_create(button);
        lv_label_set_text(label, text);
        lv_obj_center(label);

        return button;
    }

    void splitFixed100(int scaled100, int &whole, int &frac, bool &negative)
    {
        negative = scaled100 < 0;
        const int absValue = std::abs(scaled100);
        whole = absValue / 100;
        frac = absValue % 100;
    }

    void setTopStatus()
    {
        if (ui.statusLabel == nullptr)
        {
            return;
        }
        lv_label_set_text_fmt(ui.statusLabel, "Aut: %s", autonRoutineName(selectedAuton.load()));
    }

    void setTopMode(DashboardMode mode)
    {
        if (ui.modeLabel == nullptr)
        {
            return;
        }
        lv_label_set_text_fmt(ui.modeLabel, "Mode: %s", modeText(mode));
    }

    void setTempDot(double maxTemp)
    {
        if (ui.tempDot == nullptr)
        {
            return;
        }
        lv_obj_set_style_bg_color(ui.tempDot, maxTemp >= MOTOR_TEMP_WARN_C ? lv_color_hex(0xE5484D) : lv_color_hex(0x29C56A), 0);
    }

    void setVoltageParts(int millivolts, int &whole, int &frac)
    {
        whole = millivolts / 1000;
        frac = std::abs((millivolts % 1000) / 10);
    }

    void setScaledParts100(double value, int &whole, int &frac, bool &negative)
    {
        splitFixed100(static_cast<int>(std::lround(value * 100.0)), whole, frac, negative);
    }

    void refreshAutonSelector()
    {
        if (ui.autonSelectedLabel == nullptr)
        {
            return;
        }

        const AutonRoutine active = selectedAuton.load();

        for (int i = 0; i < AUTON_OPTION_COUNT; i++)
        {
            if (ui.autonButtons[i] != nullptr)
            {
                stylePillButton(ui.autonButtons[i], AUTON_OPTIONS[i] == active);
            }
        }

        lv_label_set_text_fmt(ui.autonSelectedLabel, "Act: %s", autonRoutineName(active));
    }

    void refreshProfileSelector()
    {
        const DriverProfile active = selectedProfile.load();
        styleProfileButton(ui.profileWhiteButton, DriverProfile::White, active == DriverProfile::White);
        styleProfileButton(ui.profilePinkButton, DriverProfile::Pink, active == DriverProfile::Pink);
    }

    void onAutonOptionPressed(lv_event_t *event)
    {
        const auto index = static_cast<int>(reinterpret_cast<std::intptr_t>(lv_event_get_user_data(event)));
        if (index < 0 || index >= AUTON_OPTION_COUNT)
        {
            return;
        }

        selectedAuton.store(AUTON_OPTIONS[index]);
    }

    void onWhiteProfilePressed(lv_event_t *)
    {
        selectedProfile.store(DriverProfile::White);
        refreshProfileSelector();
    }

    void onPinkProfilePressed(lv_event_t *)
    {
        selectedProfile.store(DriverProfile::Pink);
        robotactions::setWingUp(true);
        refreshProfileSelector();
    }

    void onShowImagePressed(lv_event_t *)
    {
        showImageRequested.store(true);
    }

    void onHideImagePressed(lv_event_t *)
    {
        showImageRequested.store(false);
    }

    lv_obj_t *createSectionCard(lv_obj_t *screen, int x, int y, int w, int h, lv_color_t color)
    {
        lv_obj_t *card = lv_obj_create(screen);
        lv_obj_set_pos(card, x, y);
        lv_obj_set_size(card, w, h);
        styleCard(card, color);
        lv_obj_set_style_pad_all(card, 6, 0);
        lv_obj_set_scrollbar_mode(card, LV_SCROLLBAR_MODE_AUTO);
        return card;
    }

    void buildTopBar(lv_obj_t *screen)
    {
        lv_obj_t *topBar = lv_obj_create(screen);
        lv_obj_set_pos(topBar, 0, 0);
        lv_obj_set_size(topBar, SCREEN_W, TOP_BAR_H);
        lv_obj_set_style_radius(topBar, 0, 0);
        lv_obj_set_style_border_width(topBar, 0, 0);
        lv_obj_set_style_bg_color(topBar, lv_color_hex(0x141D28), 0);
        lv_obj_set_style_bg_opa(topBar, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(topBar, 0, 0);

        ui.modeLabel = lv_label_create(topBar);
        lv_obj_set_pos(ui.modeLabel, 10, 8);
        lv_label_set_text(ui.modeLabel, "Mode: Disabled");
        lv_obj_set_style_text_font(ui.modeLabel, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(ui.modeLabel, lv_color_hex(0xDCE7F5), 0);

        ui.statusLabel = lv_label_create(topBar);
        lv_obj_set_pos(ui.statusLabel, 120, 8);
        lv_obj_set_width(ui.statusLabel, 150);
        lv_label_set_long_mode(ui.statusLabel, LV_LABEL_LONG_CLIP);
        lv_label_set_text(ui.statusLabel, "Aut: Do Nothing");
        lv_obj_set_style_text_font(ui.statusLabel, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(ui.statusLabel, lv_color_hex(0xDCE7F5), 0);

        ui.profileWhiteButton = createPillButton(topBar, "WHITE", 276, 4, 52, 22);
        if (ui.profileWhiteButton != nullptr)
        {
            lv_obj_set_style_text_font(ui.profileWhiteButton, &lv_font_montserrat_10, 0);
            lv_obj_add_event_cb(ui.profileWhiteButton, onWhiteProfilePressed, LV_EVENT_CLICKED, nullptr);
        }

        ui.profilePinkButton = createPillButton(topBar, "PINK", 332, 4, 42, 22);
        if (ui.profilePinkButton != nullptr)
        {
            lv_obj_set_style_text_font(ui.profilePinkButton, &lv_font_montserrat_10, 0);
            lv_obj_add_event_cb(ui.profilePinkButton, onPinkProfilePressed, LV_EVENT_CLICKED, nullptr);
        }

        ui.showImageButton = createPillButton(topBar, "Image", 378, 4, 72, 22);
        if (ui.showImageButton != nullptr)
        {
            stylePillButton(ui.showImageButton, true);
            lv_obj_set_style_text_font(ui.showImageButton, &lv_font_montserrat_10, 0);
            lv_obj_add_event_cb(ui.showImageButton, onShowImagePressed, LV_EVENT_CLICKED, nullptr);
        }

        ui.tempDot = lv_obj_create(topBar);
        lv_obj_set_pos(ui.tempDot, 456, 10);
        lv_obj_set_size(ui.tempDot, 10, 10);
        lv_obj_set_style_radius(ui.tempDot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(ui.tempDot, 0, 0);
        lv_obj_set_style_bg_color(ui.tempDot, lv_color_hex(0x29C56A), 0);

        refreshProfileSelector();
    }

    void buildSections(lv_obj_t *screen)
    {
        ui.infoCard = createSectionCard(screen, INFO_X, INFO_Y, INFO_W, INFO_H, lv_color_hex(0x1B2837));
        ui.infoLabel = lv_label_create(ui.infoCard);
        lv_obj_set_pos(ui.infoLabel, 4, 4);
        lv_obj_set_width(ui.infoLabel, INFO_W - 20);
        lv_label_set_long_mode(ui.infoLabel, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(ui.infoLabel, &lv_font_montserrat_12, 0);
        lv_label_set_text(ui.infoLabel, "Waiting...");

        ui.autonCard = createSectionCard(screen, AUTON_X, AUTON_Y, AUTON_W, AUTON_H, lv_color_hex(0x1B2837));
        ui.autonSelectedLabel = lv_label_create(ui.autonCard);
        lv_obj_set_pos(ui.autonSelectedLabel, 4, 4);
        lv_obj_set_style_text_font(ui.autonSelectedLabel, &lv_font_montserrat_14, 0);
        lv_label_set_text(ui.autonSelectedLabel, "Act: Do Nothing");

        ui.autonList = lv_obj_create(ui.autonCard);
        lv_obj_set_pos(ui.autonList, 4, 28);
        lv_obj_set_size(ui.autonList, AUTON_W - 20, AUTON_H - 36);
        styleCard(ui.autonList, lv_color_hex(0x223243));
        lv_obj_set_style_pad_all(ui.autonList, 4, 0);
        lv_obj_set_scrollbar_mode(ui.autonList, LV_SCROLLBAR_MODE_AUTO);

        for (int i = 0; i < AUTON_OPTION_COUNT; i++)
        {
            ui.autonButtons[i] = createPillButton(
                ui.autonList,
                autonRoutineName(AUTON_OPTIONS[i]),
                2,
                2 + (i * 36),
                AUTON_W - 36,
                30);

            lv_obj_add_event_cb(
                ui.autonButtons[i],
                onAutonOptionPressed,
                LV_EVENT_CLICKED,
                reinterpret_cast<void *>(static_cast<std::intptr_t>(i)));
        }

        refreshAutonSelector();
    }

    void buildImageScreen(lv_obj_t *screen)
    {
        ui.imageLayer = lv_obj_create(screen);
        lv_obj_set_pos(ui.imageLayer, 0, 0);
        lv_obj_set_size(ui.imageLayer, SCREEN_W, SCREEN_H);
        styleCard(ui.imageLayer, lv_color_hex(0x0F151C));
        lv_obj_set_style_radius(ui.imageLayer, 0, 0);
        lv_obj_set_style_pad_all(ui.imageLayer, 0, 0);
        lv_obj_set_scrollbar_mode(ui.imageLayer, LV_SCROLLBAR_MODE_OFF);
        lv_obj_remove_flag(ui.imageLayer, LV_OBJ_FLAG_SCROLLABLE);

        ui.imageHintLabel = lv_label_create(ui.imageLayer);
        lv_obj_set_pos(ui.imageHintLabel, 8, 6);
        lv_label_set_text(ui.imageHintLabel, "Drive: Curvature\nTap image to return");
        lv_obj_set_style_text_font(ui.imageHintLabel, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(ui.imageHintLabel, lv_color_hex(0xC8D7E8), 0);

        ui.imageObj = lv_image_create(ui.imageLayer);
        if (ui.imageObj != nullptr)
        {
            lv_image_set_src(ui.imageObj, &bm2_small);
            lv_obj_center(ui.imageObj);
            lv_obj_add_flag(ui.imageObj, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(ui.imageObj, onHideImagePressed, LV_EVENT_CLICKED, nullptr);
        }
        else
        {
            lv_label_set_text(ui.imageHintLabel, "Image alloc failed");
        }
    }

    void buildImuOverlay(lv_obj_t *screen)
    {
        ui.imuOverlay = lv_obj_create(screen);
        lv_obj_set_pos(ui.imuOverlay, 0, 0);
        lv_obj_set_size(ui.imuOverlay, SCREEN_W, SCREEN_H);
        lv_obj_set_style_radius(ui.imuOverlay, 0, 0);
        lv_obj_set_style_border_width(ui.imuOverlay, 0, 0);
        lv_obj_set_style_bg_color(ui.imuOverlay, lv_color_hex(0x7A1111), 0);
        lv_obj_set_style_bg_opa(ui.imuOverlay, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(ui.imuOverlay, 0, 0);
        lv_obj_set_scrollbar_mode(ui.imuOverlay, LV_SCROLLBAR_MODE_OFF);
        lv_obj_remove_flag(ui.imuOverlay, LV_OBJ_FLAG_SCROLLABLE);

        ui.imuOverlayLabel = lv_label_create(ui.imuOverlay);
        lv_obj_set_width(ui.imuOverlayLabel, SCREEN_W - 40);
        lv_obj_set_style_text_align(ui.imuOverlayLabel, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(ui.imuOverlayLabel, lv_color_hex(0xFCECEC), 0);
        lv_obj_set_style_text_font(ui.imuOverlayLabel, &lv_font_montserrat_14, 0);
        lv_label_set_long_mode(ui.imuOverlayLabel, LV_LABEL_LONG_WRAP);
        lv_label_set_text(ui.imuOverlayLabel, "IMU Calibrating...\nKeep robot still");
        lv_obj_align(ui.imuOverlayLabel, LV_ALIGN_CENTER, 0, -24);

        ui.imuOverlayExitButton = createPillButton(ui.imuOverlay, "Exit", 192, 190, 96, 30);
        if (ui.imuOverlayExitButton != nullptr)
        {
            stylePillButton(ui.imuOverlayExitButton, true);
            lv_obj_add_event_cb(
                ui.imuOverlayExitButton,
                [](lv_event_t *)
                {
                    ui.imuOverlayDismissed = true;
                    lv_obj_add_flag(ui.imuOverlay, LV_OBJ_FLAG_HIDDEN);
                },
                LV_EVENT_CLICKED,
                nullptr);
        }

        lv_obj_add_flag(ui.imuOverlay, LV_OBJ_FLAG_HIDDEN);
    }

    void buildFlashOverlay(lv_obj_t *screen)
    {
        ui.flashOverlay = lv_obj_create(screen);
        lv_obj_set_pos(ui.flashOverlay, 0, 0);
        lv_obj_set_size(ui.flashOverlay, SCREEN_W, SCREEN_H);
        lv_obj_set_style_radius(ui.flashOverlay, 0, 0);
        lv_obj_set_style_border_width(ui.flashOverlay, 0, 0);
        lv_obj_set_style_bg_color(ui.flashOverlay, lv_color_hex(0x155F2A), 0);
        lv_obj_set_style_bg_opa(ui.flashOverlay, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(ui.flashOverlay, 0, 0);
        lv_obj_set_scrollbar_mode(ui.flashOverlay, LV_SCROLLBAR_MODE_OFF);
        lv_obj_remove_flag(ui.flashOverlay, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(ui.flashOverlay, LV_OBJ_FLAG_HIDDEN);
    }

    void startFlashSequence(bool enabled)
    {
        if (ui.flashOverlay == nullptr)
        {
            return;
        }

        flashState.active = true;
        flashState.visible = true;
        flashState.pulsesShown = 0;
        flashState.phaseStartMs = static_cast<std::uint32_t>(pros::millis());
        flashState.color = enabled ? lv_color_hex(0x1E8F3E) : lv_color_hex(0xB31212);

        lv_obj_set_style_bg_color(ui.flashOverlay, flashState.color, 0);
        moveObjToFront(ui.flashOverlay);
        lv_obj_remove_flag(ui.flashOverlay, LV_OBJ_FLAG_HIDDEN);
    }

    void applyFlashOverlay()
    {
        if (ui.flashOverlay == nullptr)
        {
            return;
        }

        const int request = pendingFlashRequest.exchange(FLASH_REQ_IDLE, std::memory_order_relaxed);
        if (request == FLASH_REQ_GREEN)
        {
            startFlashSequence(true);
        }
        else if (request == FLASH_REQ_RED)
        {
            startFlashSequence(false);
        }

        if (!flashState.active)
        {
            lv_obj_add_flag(ui.flashOverlay, LV_OBJ_FLAG_HIDDEN);
            return;
        }

        const std::uint32_t nowMs = static_cast<std::uint32_t>(pros::millis());
        if (flashState.visible)
        {
            moveObjToFront(ui.flashOverlay);
            lv_obj_remove_flag(ui.flashOverlay, LV_OBJ_FLAG_HIDDEN);

            if ((nowMs - flashState.phaseStartMs) >= FLASH_ON_MS)
            {
                flashState.visible = false;
                flashState.phaseStartMs = nowMs;
                flashState.pulsesShown++;
                lv_obj_add_flag(ui.flashOverlay, LV_OBJ_FLAG_HIDDEN);

                if (flashState.pulsesShown >= FLASH_PULSE_COUNT)
                {
                    flashState.active = false;
                }
            }
            return;
        }

        lv_obj_add_flag(ui.flashOverlay, LV_OBJ_FLAG_HIDDEN);

        if (!flashState.active)
        {
            return;
        }

        if ((nowMs - flashState.phaseStartMs) >= FLASH_OFF_MS && flashState.pulsesShown < FLASH_PULSE_COUNT)
        {
            flashState.visible = true;
            flashState.phaseStartMs = nowMs;
            lv_obj_set_style_bg_color(ui.flashOverlay, flashState.color, 0);
            moveObjToFront(ui.flashOverlay);
            lv_obj_remove_flag(ui.flashOverlay, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void showImuOverlay(ImuOverlayState state)
    {
        if (ui.imuOverlay == nullptr || ui.imuOverlayLabel == nullptr)
        {
            return;
        }

        ui.imuOverlayState = state;
        if (state == ImuOverlayState::Hidden)
        {
            lv_obj_add_flag(ui.imuOverlay, LV_OBJ_FLAG_HIDDEN);
            return;
        }

        switch (state)
        {
        case ImuOverlayState::Calibrating:
            lv_obj_set_style_bg_color(ui.imuOverlay, lv_color_hex(0xB31212), 0);
            lv_label_set_text(ui.imuOverlayLabel, "IMU Calibrating...\nKeep robot still");
            break;
        case ImuOverlayState::Success:
            lv_obj_set_style_bg_color(ui.imuOverlay, lv_color_hex(0x155F2A), 0);
            lv_label_set_text(ui.imuOverlayLabel, "IMU Calibration Complete");
            break;
        case ImuOverlayState::Failed:
            lv_obj_set_style_bg_color(ui.imuOverlay, lv_color_hex(0x8B0B0B), 0);
            lv_label_set_text(ui.imuOverlayLabel, "IMU Calibration Failed\nPress UP to retry");
            break;
        default:
            break;
        }

        lv_obj_align(ui.imuOverlayLabel, LV_ALIGN_CENTER, 0, -24);
        moveObjToFront(ui.imuOverlay);
        lv_obj_remove_flag(ui.imuOverlay, LV_OBJ_FLAG_HIDDEN);
    }

    void applyImuOverlay()
    {
        if (ui.imuOverlay == nullptr || ui.imuOverlayLabel == nullptr)
        {
            return;
        }

        const pros::ImuStatus imuStatus = inertial.get_status();
        const bool calibrating = inertial.is_calibrating() || imuStatus == pros::ImuStatus::calibrating;
        const bool failed = !inertial.is_installed() || imuStatus == pros::ImuStatus::error;

        ImuOverlayState nextState = ImuOverlayState::Hidden;
        if (calibrating)
        {
            nextState = ImuOverlayState::Calibrating;
        }
        else if (ui.imuWasCalibrating)
        {
            nextState = failed ? ImuOverlayState::Failed : ImuOverlayState::Success;
        }
        else if (failed)
        {
            nextState = ImuOverlayState::Failed;
        }

        if (nextState != ui.imuOverlayState)
        {
            ui.imuOverlayDismissed = false;
            showImuOverlay(nextState);
        }
        else if (nextState == ImuOverlayState::Hidden)
        {
            showImuOverlay(ImuOverlayState::Hidden);
        }
        else if (!ui.imuOverlayDismissed)
        {
            showImuOverlay(nextState);
        }

        ui.imuWasCalibrating = calibrating;

        if (!ui.imuWasCalibrating && nextState == ImuOverlayState::Hidden)
        {
            ui.imuOverlayDismissed = false;
        }
    }

    void applyImageVisibility()
    {
        if (ui.mainLayer == nullptr || ui.imageLayer == nullptr)
        {
            return;
        }

        const bool shouldShowImage = showImageRequested.load();
        if (shouldShowImage == ui.imageVisible)
        {
            return;
        }

        ui.imageVisible = shouldShowImage;
        if (shouldShowImage)
        {
            lv_obj_add_flag(ui.mainLayer, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(ui.imageLayer, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(ui.imageLayer, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(ui.mainLayer, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void updateTelemetry(double speedMps)
    {
        const auto nowMs = static_cast<std::uint32_t>(pros::millis());

        if (!telemetry.initialized)
        {
            telemetry.initialized = true;
            telemetry.lastTimeMs = nowMs;
            telemetry.lastSpeedMps = speedMps;
            telemetry.speedMps = speedMps;
            telemetry.accelMps2 = 0.0;
            return;
        }

        const double dt = std::max(0.001, static_cast<double>(nowMs - telemetry.lastTimeMs) / 1000.0);
        telemetry.accelMps2 = (speedMps - telemetry.lastSpeedMps) / dt;
        telemetry.speedMps = speedMps;

        telemetry.lastTimeMs = nowMs;
        telemetry.lastSpeedMps = speedMps;
    }

    void updateTopBar(DashboardMode mode, double maxTemp)
    {
        setTopMode(mode);
        setTopStatus();
        setTempDot(maxTemp);
    }

    void updateDriveModeDisplay(const char *driveMode)
    {
        if (ui.imageHintLabel != nullptr)
        {
            lv_label_set_text_fmt(ui.imageHintLabel, "Drive: %s\nTap image to return", driveMode);
        }
    }

    void updateControllerMotorSummary(DashboardMode mode)
    {
        std::vector<std::int32_t> drivetrainVoltages = leftDrive.get_voltage_all();
        const std::vector<std::int32_t> rightVoltages = rightDrive.get_voltage_all();
        drivetrainVoltages.insert(drivetrainVoltages.end(), rightVoltages.begin(), rightVoltages.end());
        const MotorRunSummary drivetrainSummary = summarizeMotorVoltages(drivetrainVoltages);

        const std::vector<std::int32_t> intakeVoltages = {rightIntake.get_voltage(), leftIntake.get_voltage()};
        const MotorRunSummary intakeSummary = summarizeMotorVoltages(intakeVoltages);

        master.print(0, 0, "DV R%02d D%2d", drivetrainSummary.runningCount, drivetrainSummary.direction);
        master.print(1, 0, "IN R%02d D%2d", intakeSummary.runningCount, intakeSummary.direction);
        master.print(2, 0, "MD %s      ", modeAbbrev(mode));
        // master.print(0, 1, "LT: %f", leftDrive.get_voltage_all());
        // master.print(1, 1, "RT: %f", rightDrive.get_voltage_all());
        // master.print(0, 2, "LT port 20: %f", drivetrainVoltages[4]);
        // master.print(1, 2, "RT port 6: %f", rightVoltages[4]);
        // master.print(2, 1, "Intake running: %s", robotactions::isAnyIntakeRunning());
    }

    void updateInfoSection(
        int batteryPercent,
        int batteryMillivolts,
        double leftDriveTemp,
        double rightDriveTemp,
        double intakeTemp,
        double maxTemp,
        const char *driveMode,
        double imuHeadingDeg,
        double odomXIn,
        double odomYIn)
    {
        int voltageWhole = 0;
        int voltageFrac = 0;
        setVoltageParts(batteryMillivolts, voltageWhole, voltageFrac);

        int speedWhole = 0;
        int speedFrac = 0;
        bool speedNegative = false;
        setScaledParts100(telemetry.speedMps, speedWhole, speedFrac, speedNegative);

        int accelWhole = 0;
        int accelFrac = 0;
        bool accelNegative = false;
        setScaledParts100(telemetry.accelMps2, accelWhole, accelFrac, accelNegative);

        int headingWhole = 0;
        int headingFrac = 0;
        bool headingNegative = false;
        setScaledParts100(imuHeadingDeg, headingWhole, headingFrac, headingNegative);

        int odomXWhole = 0;
        int odomXFrac = 0;
        bool odomXNegative = false;
        setScaledParts100(odomXIn, odomXWhole, odomXFrac, odomXNegative);

        int odomYWhole = 0;
        int odomYFrac = 0;
        bool odomYNegative = false;
        setScaledParts100(odomYIn, odomYWhole, odomYFrac, odomYNegative);

        if (ui.infoLabel == nullptr)
        {
            return;
        }

        lv_label_set_text_fmt(
            ui.infoLabel,
            "Bat %d%% %d.%02dV\n"
            "Drv %s\n"
            "\n"
            "Spd %s%d.%02d m/s\n"
            "Acc %s%d.%02d m/s2\n"
            "\n"
            "Hdg %s%d.%02d deg\n"
            "X %s%d.%02d in\n"
            "Y %s%d.%02d in\n"
            "\n"
            "Tmp L/R %d/%dC\n"
            "Intk %dC Max %dC",
            batteryPercent,
            voltageWhole,
            voltageFrac,
            driveMode,
            speedNegative ? "-" : "",
            speedWhole,
            speedFrac,
            accelNegative ? "-" : "",
            accelWhole,
            accelFrac,
            headingNegative ? "-" : "",
            headingWhole,
            headingFrac,
            odomXNegative ? "-" : "",
            odomXWhole,
            odomXFrac,
            odomYNegative ? "-" : "",
            odomYWhole,
            odomYFrac,
            static_cast<int>(std::lround(leftDriveTemp)),
            static_cast<int>(std::lround(rightDriveTemp)),
            static_cast<int>(std::lround(intakeTemp)),
            static_cast<int>(std::lround(maxTemp)));
    }

    void buildUi()
    {
        lv_obj_t *screen = lv_screen_active();
        lv_obj_clean(screen);
        lv_obj_set_style_bg_color(screen, lv_color_hex(0x0F151C), 0);
        lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

        ui.mainLayer = lv_obj_create(screen);
        lv_obj_set_pos(ui.mainLayer, 0, 0);
        lv_obj_set_size(ui.mainLayer, SCREEN_W, SCREEN_H);
        styleCard(ui.mainLayer, lv_color_hex(0x0F151C));
        lv_obj_set_style_radius(ui.mainLayer, 0, 0);
        lv_obj_set_style_pad_all(ui.mainLayer, 0, 0);
        lv_obj_set_scrollbar_mode(ui.mainLayer, LV_SCROLLBAR_MODE_OFF);
        lv_obj_remove_flag(ui.mainLayer, LV_OBJ_FLAG_SCROLLABLE);

        buildTopBar(ui.mainLayer);
        buildSections(ui.mainLayer);
        buildImageScreen(screen);
        buildImuOverlay(screen);
        buildFlashOverlay(screen);

        const bool startOnImage = requestedMode.load() == DashboardMode::Driver;
        showImageRequested.store(startOnImage);
        ui.imageVisible = startOnImage;
        ui.imuWasCalibrating = false;
        ui.imuOverlayDismissed = false;
        ui.imuOverlayState = ImuOverlayState::Hidden;
        flashState.active = false;
        flashState.visible = false;
        flashState.pulsesShown = 0;
        flashState.phaseStartMs = 0;
        pendingFlashRequest.store(FLASH_REQ_IDLE, std::memory_order_relaxed);
        if (ui.flashOverlay != nullptr)
        {
            lv_obj_add_flag(ui.flashOverlay, LV_OBJ_FLAG_HIDDEN);
        }
        if (startOnImage)
        {
            lv_obj_add_flag(ui.mainLayer, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(ui.imageLayer, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(ui.imageLayer, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void dashboardTask(void *)
    {
        AutonRoutine lastAutonShown = selectedAuton.load();
        DriverProfile lastProfileShown = selectedProfile.load();

        while (true)
        {
            const DashboardMode mode = requestedMode.load();
            const double speedMps = driveSpeedMps();

            const double leftDriveTemp = leftDriveAvgTempC();
            const double rightDriveTemp = rightDriveAvgTempC();
            const double intakeTemp = intakeAvgTempC();
            const double maxTemp = maxMotorTempC();

            const int batteryPercent = static_cast<int>(std::lround(pros::battery::get_capacity()));
            const int batteryMillivolts = pros::battery::get_voltage();
            const lemlib::Pose odomPose = chassis.getPose();
            const double imuHeadingDeg = inertial.get_heading();

            updateTelemetry(speedMps);
            const AutonRoutine currentAuton = selectedAuton.load();
            if (currentAuton != lastAutonShown)
            {
                refreshAutonSelector();
                lastAutonShown = currentAuton;
            }
            const DriverProfile currentProfile = selectedProfile.load();
            if (currentProfile != lastProfileShown)
            {
                refreshProfileSelector();
                lastProfileShown = currentProfile;
            }
            const char *driveMode = driveModeText(currentProfile);

            // applyImageVisibility();
            applyImuOverlay();
            applyFlashOverlay();
            updateTopBar(mode, maxTemp);
            updateDriveModeDisplay(driveMode);
            updateInfoSection(
                batteryPercent,
                batteryMillivolts,
                leftDriveTemp,
                rightDriveTemp,
                intakeTemp,
                maxTemp,
                driveMode,
                imuHeadingDeg,
                odomPose.x,
                odomPose.y);
            updateControllerMotorSummary(mode);

            pros::delay(UI_UPDATE_MS);
        }
    }
} // namespace

void initDashboard()
{
    static bool started = false;
    if (started)
    {
        return;
    }

    started = true;
    buildUi();

    static pros::Task uiTask(dashboardTask, nullptr, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "Dashboard");
    (void)uiTask;
}

void setDashboardMode(DashboardMode mode)
{
    const DashboardMode previousMode = requestedMode.exchange(mode);
    if (mode == DashboardMode::Driver && previousMode != DashboardMode::Driver)
    {
        showImageRequested.store(true);
    }
}

AutonRoutine getSelectedAutonRoutine()
{
    return selectedAuton.load();
}

const char *getSelectedAutonName()
{
    return autonRoutineName(selectedAuton.load());
}

DriverProfile getSelectedDriverProfile()
{
    return selectedProfile.load();
}
