#pragma once
#include <imgui.h>
#include <implot.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <pthread.h>
#include "ScopeStream.h"
#include "sil.h"

#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif

// ======================= GLOBALS =======================
constexpr int SAMPLE_DEPTH = 1000;
constexpr int CHANNELS = 7;

constexpr int CONTROL_PANEL_WIDTH = 300;
constexpr float ELEC_PLOT_HEIGHT = 0.35f;
constexpr float GENERAL_PANEL_WIDTH = 0.25f;

inline float channel_buffer[CHANNELS][SAMPLE_DEPTH];
inline float scope_buffer[CHANNELS * SAMPLE_DEPTH];
inline ScopeStream<float> scope(scope_buffer, CHANNELS, SAMPLE_DEPTH);

// Add mutex for thread safety
inline pthread_mutex_t scope_mutex = PTHREAD_MUTEX_INITIALIZER;
inline bool freeze_scope = false;
inline float sample_time_us = 0.0f;
inline float sil_time_s = 0.0f;

enum class VoltagePlotType
{
    Line_To_Ground,
    Line_To_Line,
    Line_To_Neutral,
    Line_To_Neutral_Filtered,
    PhaseEmf,
    Alpha_beta,
    DQ
};

enum class DisplayMode
{
    Trapezoidal,
    FOC
};

inline const char *voltage_plot_names[] = {
    "Line to Ground",
    "Line to Line",
    "Line to Neutral",
    "Line to Neutral Filtered",
    "Phase EMF",
    "Alpha_beta",
    "DQ"};

inline const char *display_mode_names[] = {
    "Trapezoidal",
    "FOC"};

inline VoltagePlotType voltage_plot_type = VoltagePlotType::Line_To_Neutral;
inline DisplayMode display_mode = DisplayMode::Trapezoidal;

struct DataSummary
{
    double rms_phase_current;
    double rms_phase_voltage;
    double avg_torque;
    double avg_omega;
    double avg_mechanical_power;
    double avg_electrical_power;
    double power_factor;
    double eff;

    double sum_current_sq;
    double sum_voltage_sq;
    double sum_torque;
    double sum_omega;
    double sum_mechanical_power;
    double sum_electrical_power;
    unsigned int sample_count;
};

inline DataSummary summary;

// ======================= PANELS =======================
#include "ImGuiFileDialog.h"
inline ImGuiFileDialog fileDialog;
// ---------- Control ----------
static void drawControlPanel()
{
    ImGui::BeginChild("ControlPanel", ImVec2(CONTROL_PANEL_WIDTH, 0), true);
    ImGui::Text("Control Panel");
    ImGui::Separator();
    if (ImGui::Checkbox("Freeze Scope", &freeze_scope))
    {
    }
    ImGui::Text("Sample Time: %.2f us", sample_time_us);
    ImGui::Text("Sil Time: %.2f s", sil_time_s);
    int temp = scope.decimation;
    if (ImGui::InputInt("Decimation", &temp, 1, 100))
    {
        if (temp < 1)
            temp = 1;
        scope.update_decimation(temp);
    }
    ImGui::Separator();
    ImGui::InputFloat("Trigger", &scope.trigger_level, 0.1, 1, "%.1f");
    ImGui::InputFloat("Hysteresis", &scope.hysteresis_level, 0.1, 1, "%.1f");
    ImGui::Separator();
    if (ImGui::BeginCombo("Display", display_mode_names[(int)display_mode]))
    {
        for (int i = 0; i < IM_ARRAYSIZE(display_mode_names); i++)
        {
            bool is_selected = ((int)display_mode == i);

            if (ImGui::Selectable(display_mode_names[i], is_selected))
            {
                display_mode = (DisplayMode)i;
            }

            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Text("Voltage Plot");
    if (ImGui::BeginCombo("Type", voltage_plot_names[(int)voltage_plot_type]))
    {
        for (int i = 0; i < IM_ARRAYSIZE(voltage_plot_names); i++)
        {
            if (display_mode == DisplayMode::Trapezoidal && (i == (int)VoltagePlotType::Alpha_beta || i == (int)VoltagePlotType::DQ))
                continue;
            bool is_selected = ((int)voltage_plot_type == i);

            if (ImGui::Selectable(voltage_plot_names[i], is_selected))
            {
                voltage_plot_type = (VoltagePlotType)i;
            }

            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    if (!freeze_scope)
    {
        sample_time_us = (sil_input.dt * 1e6 * scope.decimation);
        sil_time_s = sil.state.time;
    }

    ImGui::Separator();
    ImGui::Text("Logger");
    // Display current path
    static char log_path[512] = "./sil_log.h5";
    ImGui::InputText("##path", log_path, sizeof(log_path));
    
    if (ImGui::Button("Save"))
    {
        IGFD::FileDialogConfig config;
        config.path = ".";              // Start directory
        config.fileName = "sil_log"; // Default filename
        config.flags = ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_ConfirmOverwrite;
        config.sidePaneWidth = 600;
        // Set dialog size
        fileDialog.OpenDialog(
            "ChooseFileDlgKey",
            "Choose HDF5 Save Location",
            ".h5", // Filter for .h5 files
            config
        );
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop"))
    {
        silLogger.logStop();
    }
    ImGui::SameLine();
    ImGui::ColorButton("##log_status", 
                   silLogger.isLogOn() ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1),
                   ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(silLogger.isLogOn() ? "Logging active" : "Logging stopped");
    }

    // Display dialog
    if (fileDialog.Display("ChooseFileDlgKey"))
    {
        if (fileDialog.IsOk())
        {
            std::string filePathName = fileDialog.GetFilePathName();
            strcpy(log_path, filePathName.c_str());
            silLogger.logStart(log_path);
            silLogger.log(sil, sil_input, true);
        }
        fileDialog.Close();
    }
    ImGui::EndChild();
}



inline void draw_rotation_indicator(ImDrawList* draw_list, 
                             float omega_rad_s, 
                             const ImVec2& pos,
                             float size = 80.0f) {
    
    static float angle = 0;
    static double last_time = ImGui::GetTime();
    
    // Update angle
    double current_time = ImGui::GetTime();
    float dt = current_time - last_time;
    last_time = current_time;
    
    // 1/100 of actual speed for display
    float rpm = omega_rad_s * 60.0f / (2.0f * M_PI);
    float display_rpm = rpm / 20.0;  // 1/20 speed
    float delta_angle = display_rpm * 360.0f * dt / 60.0f;
    angle = fmod(angle + delta_angle, 360.0f);
    
    ImVec2 center(pos.x + size/2, pos.y + size/2);
    // Draw circle
    draw_list->AddCircle(center, size/2, IM_COL32(150, 150, 150, 255), 0, 2.0f);
    // Draw arrow
    float rad = angle * M_PI / 180.0f;
    float arrow_length = size/2 * 0.9f;
    ImVec2 tip(center.x + arrow_length * cos(rad),
               center.y + arrow_length * sin(rad));
    
    // Arrow line
    draw_list->AddLine(center, tip, IM_COL32(255, 100, 100, 255), 3.0f);
    
    // Center dot
    draw_list->AddCircleFilled(center, 4, IM_COL32(255, 255, 255, 255));
}

// ---------- Summary ----------
static void drawSummary()
{
    ImGui::BeginChild("#Summary", ImVec2(0, 0), true);
    float avail_width = ImGui::GetContentRegionAvail().x;
    float indicator_width = 80;

    ImGui::Text("Phase Current: %.2f A", summary.rms_phase_current);
    ImGui::Text("Phase Voltage: %.2f V", summary.rms_phase_voltage);
    ImGui::Text("Electrical Power: %.2f W", summary.avg_electrical_power);
    ImGui::Text("Torque: %.2f N.m", summary.avg_torque);
    ImGui::Text("Speed: %.2f rpm", summary.avg_omega * 60 / (2 * M_PI));
    ImGui::Text("Mechanical Power: %.2f W", summary.avg_mechanical_power);
    ImGui::Text("Efficiency: %.2f %%", summary.eff * 100);
    ImGui::Separator();
        
    // Get the ImDrawList for current window
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    // Get cursor position (where next element will be placed)
    ImVec2 cursor_pos = ImGui::GetCursorScreenPos();
    cursor_pos.x += 10; cursor_pos.y += 10;
    draw_rotation_indicator(draw_list , sil.state.omega, cursor_pos, indicator_width);

    ImGui::EndChild();
}

static ImPlotRange shared_x_range(0, SAMPLE_DEPTH);
static bool link_x_axis = true;

// ---------- Current Plot ----------
static void drawPhaseCurrentPlot(float height)
{
    if (ImPlot::BeginPlot("Phase Currents", ImVec2(-1, height)))
    {
        ImPlot::SetupAxes("Samples", "Current (A)");
        ImPlot::SetupAxisLinks(ImAxis_X1, &shared_x_range.Min, &shared_x_range.Max);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -20, 20);
        pthread_mutex_lock(&scope_mutex);
        ImPlot::PlotLine("Ia", channel_buffer[0], SAMPLE_DEPTH);
        ImPlot::PlotLine("Ib", channel_buffer[1], SAMPLE_DEPTH);
        ImPlot::PlotLine("Ic", channel_buffer[2], SAMPLE_DEPTH);
        ImPlot::PlotInfLines("Trigger", &scope.trigger_level, 1, {ImPlotProp_Flags, ImPlotInfLinesFlags_Horizontal});
        pthread_mutex_unlock(&scope_mutex);
        ImPlot::EndPlot();
    }
}

// ---------- Voltage Plot ----------
static void drawPhaseVoltagePlot(float height)
{
    if (ImPlot::BeginPlot("Phase Voltages", ImVec2(-1, height)))
    {
        ImPlot::SetupAxes("Samples", "Voltage (V)");
        ImPlot::SetupAxisLinks(ImAxis_X1, &shared_x_range.Min, &shared_x_range.Max);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -20, 20);
        pthread_mutex_lock(&scope_mutex);
        ImPlot::PlotLine("Va", channel_buffer[3], SAMPLE_DEPTH);
        ImPlot::PlotLine("Vb", channel_buffer[4], SAMPLE_DEPTH);
        ImPlot::PlotLine("Vc", channel_buffer[5], SAMPLE_DEPTH);
        pthread_mutex_unlock(&scope_mutex);
        ImPlot::EndPlot();
    }
}

// ---------- Mechanical ----------
static void drawMechanical(float height)
{
    if (ImPlot::BeginPlot("Mechanical", ImVec2(-1, height)))
    {
        ImPlot::SetupAxes("Samples", "Torque (N.m)");
        ImPlot::SetupAxisLinks(ImAxis_X1, &shared_x_range.Min, &shared_x_range.Max);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -0.5, 0.5);
        pthread_mutex_lock(&scope_mutex);
        ImPlot::PlotLine("Te", channel_buffer[6], SAMPLE_DEPTH);
        pthread_mutex_unlock(&scope_mutex);
        ImPlot::EndPlot();
    }
}

// ======================= DASHBOARD =======================
static void renderDashboard()
{
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);

    ImGui::Begin("SIL Dashboard", nullptr,
                 ImGuiWindowFlags_NoDecoration |
                     ImGuiWindowFlags_NoMove);

    // LEFT CONTROL
    drawControlPanel();
    ImGui::SameLine();

    // RIGHT SIDE
    ImGui::BeginChild("RightPane");

    float totalW = ImGui::GetContentRegionAvail().x;

    // LEFT summary (25%)
    ImGui::BeginChild("SummaryColumn", ImVec2(totalW * GENERAL_PANEL_WIDTH, 0), true);
    drawSummary();
    ImGui::EndChild();

    ImGui::SameLine();

    // RIGHT plots (75%)
    ImGui::BeginChild("Plots");

    float totalH = ImGui::GetContentRegionAvail().y;

    float h1 = totalH * ELEC_PLOT_HEIGHT;
    float h2 = totalH * ELEC_PLOT_HEIGHT;
    float h3 = totalH * (1.0f - 2 * ELEC_PLOT_HEIGHT);

    drawPhaseCurrentPlot(h1);
    drawPhaseVoltagePlot(h2);
    drawMechanical(h3);

    ImGui::EndChild();
    ImGui::EndChild();

    ImGui::End();
}

// ======================= MAIN =======================
inline void silgui_render()
{
    renderDashboard();
}

inline void silgui_init()
{
    scope.trigger_level = 2;
    scope.hysteresis_level = 1;
}
inline void silgui_updateData()
{
    float scope_data[CHANNELS];
    static float prev_scope_data[3] = {0, 0, 0};
    scope_data[0] = sil.state.ip[0];
    scope_data[1] = sil.state.ip[1];
    scope_data[2] = sil.state.ip[2];

    if (voltage_plot_type == VoltagePlotType::Line_To_Ground)
    {
        scope_data[3] = sil.state.vp[0];
        scope_data[4] = sil.state.vp[1];
        scope_data[5] = sil.state.vp[2];
    }
    else if (voltage_plot_type == VoltagePlotType::Line_To_Neutral)
    {
        scope_data[3] = sil.state.vp[0] - sil.state.vn;
        scope_data[4] = sil.state.vp[1] - sil.state.vn;
        scope_data[5] = sil.state.vp[2] - sil.state.vn;
    }
    else if (voltage_plot_type == VoltagePlotType::Line_To_Line)
    {
        scope_data[3] = sil.state.vp[0] - sil.state.vp[1];
        scope_data[4] = sil.state.vp[1] - sil.state.vp[2];
        scope_data[5] = sil.state.vp[2] - sil.state.vp[0];
    }
    else if (voltage_plot_type == VoltagePlotType::PhaseEmf)
    {
        scope_data[3] = sil.state.dPsim_dt[0];
        scope_data[4] = sil.state.dPsim_dt[1];
        scope_data[5] = sil.state.dPsim_dt[2];
    }else if (voltage_plot_type == VoltagePlotType::Line_To_Neutral_Filtered)
    {
        scope_data[3] = sil.state.vp_filt[0] - sil.state.vn_filt;
        scope_data[4] = sil.state.vp_filt[1] - sil.state.vn_filt;
        scope_data[5] = sil.state.vp_filt[2] - sil.state.vn_filt;
    }
    scope_data[6] = sil.state.torque;
    scope.write(scope_data);

    // 3-wire two wattmeter method
    summary.sum_mechanical_power += sil.state.torque * sil.state.omega;
    summary.sum_electrical_power += (sil.state.vp[0] - sil.state.vp[1]) * sil.state.ip[0] + (sil.state.vp[2] - sil.state.vp[1]) * sil.state.ip[2];
    summary.sum_current_sq += sil.state.ip[0] * sil.state.ip[0];
    summary.sum_voltage_sq += (sil.state.vp[0] - sil.state.vn) * (sil.state.vp[0] - sil.state.vn);
    summary.sum_torque += sil.state.torque;
    summary.sum_omega += sil.state.omega;
    summary.sample_count++;

    int arrSize = sizeof(channel_buffer[0]) / sizeof(float);
    float filterBuf[arrSize];
    if (scope.isFrozen())
    {
        pthread_mutex_lock(&scope_mutex);
        if (!freeze_scope)
        {
            scope.read(0, (float *)channel_buffer[0], sizeof(channel_buffer));
            scope.read(1, (float *)channel_buffer[1], sizeof(channel_buffer));
            scope.read(2, (float *)channel_buffer[2], sizeof(channel_buffer));
            scope.read(3, (float *)channel_buffer[3], sizeof(channel_buffer));
            scope.read(4, (float *)channel_buffer[4], sizeof(channel_buffer));
            scope.read(5, (float *)channel_buffer[5], sizeof(channel_buffer));
            scope.read(6, (float *)channel_buffer[6], sizeof(channel_buffer));

            // 3-wire two wattmeter method
            summary.rms_phase_current = sqrt(summary.sum_current_sq / summary.sample_count);
            summary.rms_phase_voltage = sqrt(summary.sum_voltage_sq / summary.sample_count);
            summary.avg_mechanical_power = summary.sum_mechanical_power / summary.sample_count;
            summary.avg_electrical_power = summary.sum_electrical_power / summary.sample_count;
            summary.avg_torque = summary.sum_torque / summary.sample_count;
            summary.avg_omega = summary.sum_omega / summary.sample_count;

            summary.eff = summary.avg_mechanical_power / summary.avg_electrical_power;
            summary.power_factor = summary.avg_electrical_power / (summary.rms_phase_current * summary.rms_phase_voltage * 3);

            summary.sum_current_sq = 0;
            summary.sum_voltage_sq = 0;
            summary.sum_torque = 0;
            summary.sum_omega = 0;
            summary.sum_mechanical_power = 0;
            summary.sum_electrical_power = 0;
            summary.sample_count = 0;
        }
        scope.reset();
        pthread_mutex_unlock(&scope_mutex);
    }
}