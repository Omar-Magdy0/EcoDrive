/* ---- Required hooks for the selected FreeRTOS POSIX configuration ---- */
#include "platform.h"
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"


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


void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
    fprintf(stderr, "FreeRTOS assert: %s:%lu\n", pcFile, ulLine);
    fflush(stderr);
    abort();
}

void vApplicationMallocFailedHook(void)
{
    fprintf(stderr, "FreeRTOS malloc failed\n");
    fflush(stderr);
    abort();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    fprintf(stderr, "FreeRTOS stack overflow in task: %s\n", pcTaskName ? pcTaskName : "(unknown)");
    fflush(stderr);
    abort();
}

void vApplicationIdleHook(void)
{
}

void vApplicationTickHook(void)
{
}

void vApplicationDaemonTaskStartupHook(void)
{
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   StackType_t *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    StackType_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#endif


#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include "silgui.h"

volatile uint64_t virtual_tick = 0;
float vtime = 0;
vtimer_manager_t timer_manager;

// get monotonic time in ns
static inline uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

// register a timer (frequency in Hz)
bool register_timer(vtimer_manager_t* mgr, timer_callback_t cb, uint64_t timestep_ns) {
    if (mgr->timer_index >= HOST_TIMERS) return false;

    vtimer_t* t = &mgr->timers[mgr->timer_index++];
    t->periodic_time_ns = (timestep_ns);
    t->last_time_ns = now_ns();
    t->cb = cb;

    if(timestep_ns < mgr->min_timestep_ns)
    {
        mgr->min_timestep_ns = timestep_ns;
    }

    return true;
}

struct SIM{
    uint64_t time_ns;
};
inline SIM sim;

void tick(uint64_t elapsed_ns)
{
    // determine smallest timestep needed by any timer
    uint64_t dt_ns = timer_manager.min_timestep_ns; // example 10 us, can be min of all timers
    uint64_t steps = elapsed_ns / dt_ns;
    for (uint64_t i = 0; i < steps; i++)
    {
        sim.time_ns += dt_ns;
        // unified check of all timers
        for (int t_idx = 0; t_idx < timer_manager.timer_index; t_idx++)
        {
            vtimer_t* t = &timer_manager.timers[t_idx];
            if (!t->cb) continue;
            if (sim.time_ns - t->last_time_ns >= t->periodic_time_ns)
            {
                t->last_time_ns += t->periodic_time_ns; // advance by one period
                t->cb();
            }
        }
    }
}

void* tick_thread(void* arg) {
    while (1)
    {
        const uint64_t sleep_ns = 1000000ULL;   // 1 ms coarse sleep
        tick(sleep_ns);
        // coarse sleep to avoid spinning CPU
        struct timespec req = {0, sleep_ns};
        nanosleep(&req, NULL);
    }
}

void* freertos_thread(void* arg)
{
    vTaskStartScheduler();
    return NULL;
}


static inline int display_w = 1000;
static inline int display_h = 1000;

GLFWwindow* window;

// Callback to handle GLFW errors
static inline void glfw_error_callback(int error, const char* description) { std::cerr << "GLFW Error " << error << ": " << description << std::endl; }
static inline void window_close_callback(GLFWwindow* w) {
    // Force the window to stay open
    glfwSetWindowShouldClose(w, GLFW_FALSE);
    
    // You could trigger an ImGui popup here saying "Please use Shutdown button"
}

void gui_init(){
    // Initialize GLFW
    // Setup error callback
    // Create window
    
    glfwSetErrorCallback(glfw_error_callback);
    
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

    window = glfwCreateWindow(display_w, display_h, "VIRTUAL_PMSM", nullptr, nullptr);
    glfwSetWindowCloseCallback(window, window_close_callback);

    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(3);

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
        auto try_load_font = [&](const char* path) -> ImFont* {
            FILE* file = fopen(path, "rb");
            if (!file) return nullptr;
            fclose(file);
            return io.Fonts->AddFontFromFileTTF(path, 18.0f);
        };

        font = try_load_font("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf");
        if (!font) font = try_load_font("/usr/share/fonts/TTF/DejaVuSans.ttf");
        if (!font) font = try_load_font("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
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
    
    //Initialization silgui
    silgui_init();
}

void gui_deinit()
{
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

void gui_loop() {
    while (!glfwWindowShouldClose(window)) {
        // Main loop
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glfwPollEvents();
        
        silgui_render();

        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
}


pthread_t t;
void freeRtos_init()
{
    pthread_create(&t, NULL, freertos_thread, NULL);
}


void platform_init()
{
    timer_manager.min_timestep_ns = 1000000;
    pthread_create(&t, NULL, tick_thread, NULL);
    gui_init();
}