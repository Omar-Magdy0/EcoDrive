#include "sim_gui.h"
#include "virtual_pmsm.h"

#include <imgui.h>
#include <implot.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <vector>
#include <cmath>
#include <iostream>

#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif

// ======================= DATA STRUCT =======================
struct ModelData {
    float ia, ib, ic;
    float va, vb, vc;
    float speed;
    float torque;
    float vdc;
};

// ======================= GLOBALS =======================
SimHelper simHelper = SimHelper(1000);
float view_buffer[VPMSM_ELEC_CHANNELS * 1000];

constexpr int BUFFER_SIZE = 1000;
constexpr int CHANNELS = 6;

constexpr int CONTROL_PANEL_WIDTH = 200;
constexpr float ELEC_PLOT_HEIGHT = 0.35f;
constexpr float GENERAL_PANEL_WIDTH = 0.25f;


static float channel_buffer[CHANNELS][BUFFER_SIZE];

// ======================= UPDATE =======================
static void update_electric_scope() {
    if (simHelper.elec_scope.read_aligned(view_buffer)) {
        for (int i = 0; i < BUFFER_SIZE; ++i) {
            for (int ch = 0; ch < CHANNELS; ++ch) {
                channel_buffer[ch][i] = view_buffer[i * CHANNELS + ch];
            }
        }
    }
}

// ======================= DATA =======================
ModelData getModelData() {
    ModelData d{};

    d.ia = view_buffer[0];
    d.ib = view_buffer[1];
    d.ic = view_buffer[2];

    d.va = view_buffer[3];
    d.vb = view_buffer[4];
    d.vc = view_buffer[5];

    d.speed  = simHelper.mechanical.speed;
    d.torque = simHelper.mechanical.torque;
    d.vdc    = simHelper.dc_link_voltage;

    return d;
}

// ======================= PANELS =======================

// ---------- Control ----------
static void drawControlPanel() {
    ImGui::BeginChild("ControlPanel", ImVec2(CONTROL_PANEL_WIDTH, 0), true);

    ImGui::Text("Control Panel");
    ImGui::Separator();

    if (ImGui::Button("Start")) simHelper.start();
    if (ImGui::Button("Stop"))  simHelper.stop();

    static bool freeze = false;
    if (ImGui::Checkbox("Freeze Scope", &freeze)) {
        simHelper.elec_scope.frozen = freeze;
    }

    ImGui::EndChild();
}

// ---------- Summary ----------
static void drawSummary(const ModelData& d) {
    ImGui::BeginChild("Summary", ImVec2(0, 0), true);

    ImGui::Text("=== Currents ===");
    ImGui::Text("Ia: %.2f A", d.ia);
    ImGui::Text("Ib: %.2f A", d.ib);
    ImGui::Text("Ic: %.2f A", d.ic);

    ImGui::Separator();

    ImGui::Text("=== Voltages ===");
    ImGui::Text("Va: %.1f V", d.va);
    ImGui::Text("Vb: %.1f V", d.vb);
    ImGui::Text("Vc: %.1f V", d.vc);
    ImGui::Text("Vdc: %.1f V", d.vdc);

    ImGui::Separator();

    ImGui::Text("=== Mechanical ===");
    ImGui::Text("Speed: %.1f rpm", d.speed);
    ImGui::Text("Torque: %.2f Nm", d.torque);

    ImGui::ProgressBar(d.speed / 3000.0f, ImVec2(-1, 10));

    ImGui::EndChild();
}

// ---------- Current Plot ----------
static void drawCurrentPlot(float height) {
    if (ImPlot::BeginPlot("Phase Currents", ImVec2(-1, height))) {

        ImPlot::SetupAxes("Samples", "Current (A)");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, BUFFER_SIZE);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -2, 2);

        ImPlot::PlotLine("Ia", channel_buffer[0], BUFFER_SIZE);
        ImPlot::PlotLine("Ib", channel_buffer[1], BUFFER_SIZE);
        ImPlot::PlotLine("Ic", channel_buffer[2], BUFFER_SIZE);

        ImPlot::EndPlot();
    }
}

// ---------- Voltage Plot ----------
static void drawVoltagePlot(float height) {
    if (ImPlot::BeginPlot("Phase Voltages", ImVec2(-1, height))) {

        ImPlot::SetupAxes("Samples", "Voltage (V)");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, BUFFER_SIZE);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -400, 400);

        ImPlot::PlotLine("Va", channel_buffer[3], BUFFER_SIZE);
        ImPlot::PlotLine("Vb", channel_buffer[4], BUFFER_SIZE);
        ImPlot::PlotLine("Vc", channel_buffer[5], BUFFER_SIZE);

        ImPlot::EndPlot();
    }
}

// ---------- Mechanical ----------
static void drawMechanical(const ModelData& d, float height) {
    ImGui::BeginChild("Mechanical", ImVec2(0, height), true);

    ImGui::Text("Speed");
    ImGui::ProgressBar(d.speed / 3000.0f, ImVec2(-1, 20));

    ImGui::Text("Torque");
    ImGui::ProgressBar(d.torque / 100.0f, ImVec2(-1, 20));

    ImGui::EndChild();
}

// ======================= DASHBOARD =======================
void renderDashboard(const ModelData& data)
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
    drawSummary(data);
    ImGui::EndChild();

    ImGui::SameLine();

    // RIGHT plots (75%)
    ImGui::BeginChild("Plots");

    float totalH = ImGui::GetContentRegionAvail().y;

    float h1 = totalH * ELEC_PLOT_HEIGHT;
    float h2 = totalH * ELEC_PLOT_HEIGHT;
    float h3 = totalH * (1.0f - 2 * ELEC_PLOT_HEIGHT);

    drawCurrentPlot(h1);
    drawVoltagePlot(h2);
    drawMechanical(data, h3);

    ImGui::EndChild();
    ImGui::EndChild();

    ImGui::End();
}

// ======================= MAIN =======================
void virtual_pmsm_gui()
{
    update_electric_scope();
    ModelData data = getModelData();
    renderDashboard(data);
}