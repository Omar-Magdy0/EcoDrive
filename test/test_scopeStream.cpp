
#include "imgui.h"
#include "implot.h"
#include "implot_internal.h"

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

static int display_w = 1000;
static int display_h = 1000;


// Assuming your template class is in this header
#include "ScopeStream.h"

static inline void DrawScopeTest() {
    static int ratio_cnt = 0;
    static const int CHANNELS = 3;
    static const int SAMPLE_DEPTH = 500;

    static float storage[SAMPLE_DEPTH * CHANNELS];
    static ScopeStream<float> scope(storage, CHANNELS, SAMPLE_DEPTH);
    static float display_buffer[SAMPLE_DEPTH * CHANNELS];
    static float time_accum = 0.0f;

    // Simulation logic (unchanged)
    float sample_rate = 10000.0f; 
    float freq = 100.0f;           
    for (int i = 0; i < 20; i++) {
        float data[CHANNELS];
        data[0] = sinf(2.0f * M_PI * freq * time_accum);
        data[1] = sinf(2.0f * M_PI * freq * time_accum + (2.0f * M_PI / 3.0f));
        data[2] = sinf(2.0f * M_PI * freq * time_accum + (4.0f * M_PI / 3.0f));
        scope.write(data);
        time_accum += (1.0f / sample_rate);
    }

    if(ratio_cnt++ >= 2)
    {
        ratio_cnt = 0;
    if (scope.frozen) {
        scope.read(display_buffer, sizeof(display_buffer));
    }

    }
    ImGui::Begin("Elcore Scope Test");
    
    // Controls
    ImGui::DragFloat("Trigger Level", &scope.trigger_level, 0.01f, -2.0f, 2.0f);
    ImGui::SliderInt("Decimation", (int*)&scope.decimation, 1, 10);

    // --- ImPlot Visualization ---
    if (ImPlot::BeginPlot("Three Phase Scope", ImVec2(-1, 400))) {
    ImPlot::SetupAxes("Sample Index", "Voltage");
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, SAMPLE_DEPTH);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -1.5f, 1.5f);

    // --- Phase A (Blue) ---
    ImPlotSpec specA;
    specA.SetProp(ImPlotProp_LineColor, ImVec4(0.0f, 0.5f, 1.0f, 1.0f));
    specA.SetProp(ImPlotProp_LineWeight, 2.0f);
    
    ImPlot::PlotLineG("Phase A", [](int idx, void* data) {
        float val = ((float*)data)[idx * 3 + 0];
        return ImPlotPoint((double)idx, (double)val);
    }, display_buffer, SAMPLE_DEPTH, specA);

    // --- Phase B (Red) ---
    ImPlotSpec specB;
    specB.SetProp(ImPlotProp_LineColor, ImVec4(1.0f, 0.2f, 0.2f, 1.0f));
    specB.SetProp(ImPlotProp_LineWeight, 2.0f);

    ImPlot::PlotLineG("Phase B", [](int idx, void* data) {
        float val = ((float*)data)[idx * 3 + 1];
        return ImPlotPoint((double)idx, (double)val);
    }, display_buffer, SAMPLE_DEPTH, specB);

    // --- Phase C (Yellow) ---
    ImPlotSpec specC;
    specC.SetProp(ImPlotProp_LineColor, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
    specC.SetProp(ImPlotProp_LineWeight, 2.0f);

    ImPlot::PlotLineG("Phase C", [](int idx, void* data) {
        float val = ((float*)data)[idx * 3 + 2];
        return ImPlotPoint((double)idx, (double)val);
    }, display_buffer, SAMPLE_DEPTH, specC);

    // --- Trigger Markers ---
    double trig_lvl = (double)scope.trigger_level;

    ImPlotSpec specTrig;
    specTrig.SetProp(ImPlotProp_LineWeight, 1.0f);

    // White Horizontal Line
    specTrig.SetProp(ImPlotProp_LineColor, ImVec4(1, 1, 1, 0.5f));
    specTrig.SetProp(ImPlotProp_Flags, ImPlotInfLinesFlags_Horizontal);
    ImPlot::PlotInfLines("##TrigLvl", &trig_lvl, 1, specTrig);

    ImPlot::EndPlot();
}

    ImGui::End();

}




// Callback to handle GLFW errors
static inline void glfw_error_callback(int error, const char* description) { std::cerr << "GLFW Error " << error << ": " << description << std::endl; }
static inline void window_close_callback(GLFWwindow* window) {
    // Force the window to stay open
    glfwSetWindowShouldClose(window, GLFW_FALSE);
    
    // You could trigger an ImGui popup here saying "Please use Shutdown button"
}

static inline void run_gui() {

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
    }
    // Setup OpenGL version
#if defined(__APPLE__)
    // GL 3.2 + GLSL 150 (MacOS)
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);           // Required on MacOS
#else
    // GL 3.0 + GLSL 130 (Windows and Linux)
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    // Create window
    GLFWwindow* window = glfwCreateWindow(display_w, display_h, "VIRTUAL_PMSM", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    // Setup error callback
    glfwSetErrorCallback(glfw_error_callback);
    glfwSetWindowCloseCallback(window, window_close_callback);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Setup context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
	ImGuiIO& io = ImGui::GetIO();


    // Setup style
    ImGui::StyleColorsDark();

    // Setup backend
    ImGui::StyleColorsDark(); // Or ImGui::StyleColorsLight();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImFont* font = nullptr;
    #ifdef _WIN32
        // Windows standard font path
        font = io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
     #else
        // Common Linux font paths (Ubuntu/Debian/Fedora)
        font = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", 18.0f);
        if (!font) font = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/TTF/DejaVuSans.ttf", 18.0f);
    #endif

    // Fallback if the specific system fonts above aren't found
    if (font == nullptr) {
        fprintf(stderr, "System font not found, using default Dear ImGui font.\n");
    }

    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsDark(); // Dark theme is the modern standard

    // --- Soften the UI ---
    style.WindowRounding    = 6.0f;  // Round window corners
    style.FrameRounding     = 4.0f;  // Round buttons/inputs
    style.ScrollbarRounding = 9.0f;  // Round scrollbars
    style.GrabRounding      = 4.0f;  // Round sliders
    style.PopupRounding     = 4.0f;  // Round right-click menus
    
    // --- Clean up Colors ---
    // Make the background a bit darker and cleaner
    style.Colors[ImGuiCol_WindowBg]         = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);
    style.Colors[ImGuiCol_Header]           = ImVec4(0.20f, 0.25f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_HeaderHovered]    = ImVec4(0.26f, 0.59f, 0.98f, 0.80f);
    
    // --- Anti-aliasing (Crucial for ImPlot) ---
    style.AntiAliasedLines = true;
    style.AntiAliasedFill  = true;

    while (!glfwWindowShouldClose(window)) {
        // Main loop
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glfwPollEvents();

        //TEST CODE
        DrawScopeTest();

        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

}



#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include "ScopeStream.h"

// Mock of your elcore_rstream logic if not linked
// (Replace with your actual includes if available)
extern "C" {
    // If you are testing this standalone, ensure elcore_rstream is linked 
    // or provide the simple implementation of the circular buffer here.
}

void print_buffer(float* buffer, int samples, int channels) {
    std::cout << "\n--- CAPTURED BUFFER ---\n";
    for (int i = 0; i < samples; i++) {
        std::cout << "[" << i << "]\t";
        for (int c = 0; c < channels; c++) {
            std::cout << buffer[i * channels + c] << "\t";
        }
        std::cout << "\n";
    }
    std::cout << "-----------------------\n";
}

void run_cli_test() {
    const int CHANNELS = 2;
    const int DEPTH = 4; // Small depth for easy CLI debugging
    float storage[DEPTH * CHANNELS] = {0};
    float display_out[DEPTH * CHANNELS] = {0};

    ScopeStream<float> scope(storage, CHANNELS, DEPTH);
    
    // Test Configuration
    scope.trigger_level = 5.0f;
    
    std::cout << "=== ElcoreScope CLI Debugger ===" << std::endl;
    std::cout << "Usage:" << std::endl;
    std::cout << "  {val1, val2, val3} -> Input a sample" << std::endl;
    std::cout << "  n                  -> Trigger a GUI 'read'" << std::endl;
    std::cout << "  q                  -> Quit" << std::endl;
    std::cout << "Target: Trigger > " << scope.trigger_level << " on Channel 0" << std::endl;

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line == "q") break;

        if (line == "n") {
            if (scope.frozen) {
                scope.read(display_out, sizeof(display_out));
                print_buffer(display_out, DEPTH, CHANNELS);
                std::cout << "Buffer reset. Status: WAITING" << std::endl;
            } else {
                std::cout << "[!] Scope not frozen yet. To_fill remaining: " << scope.write_idx << std::endl;
            }
            continue;
        }

        // Parse {x, y, z}
        if (line.find('{') != std::string::npos) {
            float vals[CHANNELS] = {0};
            char dummy;
            std::stringstream ss(line);
            ss >> dummy; // eat '{'
            for (int i = 0; i < CHANNELS; i++) {
                ss >> vals[i];
                if (i < CHANNELS - 1) ss >> dummy; // eat ','
            }
            
            scope.write(vals);
            
            // Print immediate feedback
            std::cout << " >> Ch0: " << vals[0] 
                      << " | Trig: " << (scope.triggered ? "YES" : "NO") 
                      << " | Frozen: " << (scope.frozen ? "YES" : "NO") 
                      << " | Head: " << scope.write_idx << std::endl;
        }
    }
}


int main()
{
    //Scope Stream Test
    //run_cli_test();
    run_gui();
    return 0;
}