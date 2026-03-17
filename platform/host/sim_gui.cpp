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

SimHelper simHelper = SimHelper(1000);
float view_buffer[VPMSM_ELEC_CHANNELS*1000];

constexpr int CONTROL_PANEL_WIDTH = 200;
constexpr float ELEC_PLOT_WIDTH = 0.7;
constexpr float ELEC_PLOT_HEIGHT = 0.4;

constexpr float MECH_PLOT_WIDTH = 0.7;
constexpr float ROTOR_PLOT_WIDTH = 0.3;
constexpr float GENERAL_PLOT_WIDTH = 0.3;

void virtual_pmsm_gui() {
    // 1. Fullscreen Background Window
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    ImGui::Begin("MasterCanvas", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground);

    // --- SECTION A: 200px FIXED CONTROL PANEL ---
    ImGui::BeginChild("ControlPanel", ImVec2(200, 0), true);
    ImGui::Text("Control Panel");
    ImGui::Separator();
    ImGui::EndChild();


    ImGui::SameLine();

    // Calculate remaining width for the 25/75 split
    float remainingWidth = ImGui::GetContentRegionAvail().x;

    // --- SECTION B: 25% PLACEHOLDER ---
    ImGui::BeginChild("Placeholder", ImVec2(remainingWidth * 0.25f, 0), true);

    ImGui::EndChild();

    ImGui::SameLine();

    // --- SECTION C: 75% PLOT AREA ---
    ImGui::BeginChild("PlotArea", ImVec2(0, 0), false); // No border on the container itself looks cleaner
    // Calculate 1/3 of the height, minus a tiny bit for item spacing
    float totalHeight = ImGui::GetContentRegionAvail().y;
    float plotHeight = (totalHeight - (ImGui::GetStyle().ItemSpacing.y * 2.0f)) / 3.0f;

    // --- Plot 1: Phase Currents ---
    if (ImPlot::BeginPlot("##PhaseCurrents", ImVec2(-1, plotHeight))) {
        if(simHelper.elec_scope.frozen){
            simHelper.elec_scope.read_aligned(view_buffer);
        }
        ImPlot::SetupAxes("Sample Index", "Current");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, 1000);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1.5f, 1.5f);

        // --- Phase A (Blue) ---
        ImPlotSpec specA;
        specA.SetProp(ImPlotProp_LineColor, ImVec4(0.0f, 0.5f, 1.0f, 1.0f));
        specA.SetProp(ImPlotProp_LineWeight, 2.0f);
        
        ImPlot::PlotLineG("Va", [](int idx, void* data) {
            float val = ((float*)data)[idx * 3 + 0];
            return ImPlotPoint((double)idx, (double)val);
        }, view_buffer, 1000, specA);

        // --- Phase B (Red) ---
        ImPlotSpec specB;
        specB.SetProp(ImPlotProp_LineColor, ImVec4(1.0f, 0.2f, 0.2f, 1.0f));
        specB.SetProp(ImPlotProp_LineWeight, 2.0f);

        ImPlot::PlotLineG("Vb", [](int idx, void* data) {
            float val = ((float*)data)[idx * 3 + 1];
            return ImPlotPoint((double)idx, (double)val);
        }, view_buffer, 1000, specB);

        // --- Phase C (Yellow) ---
        ImPlotSpec specC;
        specC.SetProp(ImPlotProp_LineColor, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
        specC.SetProp(ImPlotProp_LineWeight, 2.0f);

        ImPlot::PlotLineG("Vc", [](int idx, void* data) {
            float val = ((float*)data)[idx * 3 + 2];
            return ImPlotPoint((double)idx, (double)val);
        }, view_buffer, 1000, specC);
        ImPlot::EndPlot();
    }

    // --- Plot 2: Voltages / Back-EMF ---
    if (ImPlot::BeginPlot("##Voltages", ImVec2(-1, plotHeight))) {
        ImPlot::SetupAxes("Time (s)", "Voltage (V)");
        ImPlot::SetupAxisLimits(ImAxis_Y1, -400, 400);
        // ImPlot::PlotLine("Va", ...);
        ImPlot::EndPlot();
    }

    // --- Plot 3: Mechanical (Speed/Torque) ---
    if (ImPlot::BeginPlot("##Mechanical", ImVec2(-1, plotHeight))) {
        ImPlot::SetupAxes("Time (s)", "Value");
        // ImPlot::PlotLine("Speed", ...);
        ImPlot::EndPlot();
    }

    ImGui::EndChild();
    ImGui::End(); // End MasterCanvas
}





