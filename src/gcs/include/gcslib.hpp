#ifndef GCSLIB_H
#define GCSLIB_H

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <string>
#include <vector>
#include <utility>
#include <chrono>
#include <array>
#include <climits>
#include "implot.h"
#include <cmath>
#include "tinyxml2.h"
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <imfilebrowser.h>
#include <cctype>
#include <regex>
#include <iostream>
#include <algorithm> // For std::clamp

// Namespace to encapsulate GUI functionality
namespace AppState {
    // Timer class
    class Timer {
    public:
        explicit Timer(float countdown);
        void Start();
        void Pause();
        void Reset();
        void SetTime(float countdown);
        float Update();
        bool IsRunning() const;
        float GetTimeLeft() const;

    private:
        float countdownTime;
        float remainingTime;
        std::chrono::steady_clock::time_point startTime;
        bool isTimerRunning;
    };

    struct windowVar {
        // Window dimensions
        static inline int display_w = 1280;
        static inline int display_h = 720;
        static inline int monitor_w = 0;
        static inline int monitor_h = 0;
        static inline float reference_height = 1080;
        static inline float reference_width = 1920;

        // Colors for the application
        static inline ImVec4 BackgroundColor = ImVec4(16.0f / 255.0f, 52.0f / 255.0f, 159.0f / 255.0f, 1.0f);
        static inline ImVec4 LightBackgroundColor = ImVec4(16.0f / 255.0f, 71.0f / 255.0f, 159.0f / 255.0f, 1.0f);
        static inline ImVec4 HighLightColor = ImVec4(0.12f, 0.56f, 1.0f, 1.0f);

    };

    struct settingsVar {
        // Stupid settings
        static inline float globalScaleFactor = 1.0;
    };

    // Function declarations
    void Setup();
    void GetPrimaryMonitorResolution(int& width, int& height);

    void UpdateWindowSize();
    void DrawLine(const ImVec4& color = ImVec4(16.0f / 255.0f, 71.0f / 255.0f, 159.0f / 255.0f, 1.0f), float thickness = 3.0f, float spacing = 15.0f);
    void CreateTable(const std::string& tableID, const std::vector<std::string>& headers, const std::vector<std::vector<std::string>>& data, Alignment alignment = Alignment::CENTER, bool showBorders = false);    
}

#endif // GCSLIB_H
