// gcslib.cpp

#include "gcslib.h"


namespace AppState {

    // Timer class implementation
    Timer::Timer(float countdown)
        : countdownTime(countdown), remainingTime(countdown), isTimerRunning(false) {}

        void Timer::Start() {
            isTimerRunning = true;
            startTime = std::chrono::steady_clock::now();
        }

        void Timer::Pause() {
            isTimerRunning = false;
        }

        void Timer::Reset() {
            isTimerRunning = false;
            remainingTime = countdownTime; // Reset to initial value
        }

        void Timer::SetTime(float countdown) {
            countdownTime = countdown;
            remainingTime = countdown;
        }

        float Timer::Update() {
            if (isTimerRunning) {
                auto now = std::chrono::steady_clock::now();
                std::chrono::duration<float> elapsed = now - startTime;
                remainingTime = countdownTime - elapsed.count();
                if (remainingTime < 0.0f) {
                    remainingTime = 0.0f;
                    isTimerRunning = false; // Stop when time runs out
                }
            }
            return remainingTime;
        }

        bool Timer::IsRunning() const {
            return isTimerRunning;
        }

        float Timer::GetTimeLeft() const {
            return remainingTime;
        }

    // Function definitions

    void Setup() {
        // Setup Dear ImGui context
        ImPlot::CreateContext();
    
    }

    void GetPrimaryMonitorResolution(int& width, int& height) {
        GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
        if (primaryMonitor) {
            const GLFWvidmode* mode = glfwGetVideoMode(primaryMonitor);
            if (mode) {
                width = mode->width;
                height = mode->height;
            }
        }
    }

    void UpdateWindowSize() {
        ImGuiIO& io = ImGui::GetIO();

        //scale 
        settingsVar::globalScaleFactor = std::min(windowVar::display_w / windowVar::reference_width, windowVar::display_h / windowVar::reference_height);

        // Apply the global scaling factor
        io.FontGlobalScale = settingsVar::globalScaleFactor;
    }

    void DrawLine(const ImVec4& color, float thickness, float spacing) {
        // Get the draw list to draw the custom separator line
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        // Set start and end points for the custom line
        ImVec2 current_pos = ImGui::GetCursorScreenPos();
        ImVec2 start = ImVec2(current_pos.x, current_pos.y + spacing * settingsVar::globalScaleFactor);
        ImVec2 end = ImVec2(start.x + ImGui::GetContentRegionAvail().x, start.y);

        // Draw the line with the desired color and thickness
        draw_list->AddLine(start, end, ImGui::ColorConvertFloat4ToU32(color), thickness);

        ImGui::Dummy(ImVec2(0.0f, spacing * settingsVar::globalScaleFactor));
    }

   
    void CreateTable(const std::string& tableID, const std::vector<std::string>& headers, const std::vector<std::vector<std::string>>& data, Alignment alignment, bool showBorders) {
        int columnCount = static_cast<int>(headers.size());
        if (columnCount == 0) return;
        auto flags = showBorders ? ImGuiTableFlags_Borders : ImGuiTableFlags_None;

        // Begin the table with specified column count
        if (ImGui::BeginTable(tableID.c_str(), columnCount, flags)) {
            // Create header row
            ImGui::TableNextRow();
            for (int i = 0; i < columnCount; i++) {
                ImGui::TableSetColumnIndex(i);
                const std::string& headerText = headers[i];

                // Center align header text
                ImGui::TableHeader(headerText.c_str());
            }

            // Create data rows
            for (const auto& row : data) {
                ImGui::TableNextRow();
                for (int i = 0; i < columnCount; i++) {
                    ImGui::TableSetColumnIndex(i);
                    if (i < static_cast<int>(row.size())) {
                        const std::string& cellText = row[i];
                        ImGui::TextUnformatted(cellText.c_str());
                    }
                }
            }

            ImGui::EndTable();
        }
    }

    
} // namespace GCSState

