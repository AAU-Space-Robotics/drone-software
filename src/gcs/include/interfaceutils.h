#pragma once

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>


namespace windowVar {
    extern int monitor_w;
    extern int monitor_h;
    extern int display_w;
    extern int display_h;
    extern ImVec4 BackgroundColor;
}

void GetPrimaryMonitorResolution(int& width, int& height);
void Setup();
void UpdateWindowSize();
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void Render();
void DrawMultiColor();
