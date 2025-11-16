#include "interfaceutils.h"

namespace windowVar {
    int monitor_w = 0;
    int monitor_h = 0;
    int display_w = 0;
    int display_h = 0;
    ImVec4 BackgroundColor = ImVec4(0.0, 0.0, 0.0, 1.0); // Black background in RGBA
}

void DrawMultiColor() {
    // Example function to demonstrate additional functionality
    ImVec2 window_pos(0, 0);
    ImVec2 window_size((float)windowVar::display_w, (float)windowVar::display_h);

    // Get draw list for the background
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();

    // Define colors (RGBA)
    ImU32 color_top_left = IM_COL32(0, 0, 200, 255);      
    ImU32 color_top_right = IM_COL32(0, 0, 200, 255);   
    ImU32 color_bottom_left = IM_COL32(0, 0, 60, 255);     
    ImU32 color_bottom_right = IM_COL32(0, 0, 60, 255);

    // Draw a multi-color rectangle (gradient effect)
    draw_list->AddRectFilledMultiColor(
        window_pos,
        ImVec2(window_pos.x + window_size.x, window_pos.y + window_size.y),
        color_top_left,
        color_top_right,
        color_bottom_right,
        color_bottom_left
    );
}

void GetPrimaryMonitorResolution(int& width, int& height) {
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);
    width = mode->width;
    height = mode->height;
    
}
void Setup() {
    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Setup style
    ImGui::StyleColorsDark();
}

void UpdateWindowSize() {
    // Update the display size
    windowVar::display_w = windowVar::monitor_w;
    windowVar::display_h = windowVar::monitor_h;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void Render() {
    // Rendering code here
}

