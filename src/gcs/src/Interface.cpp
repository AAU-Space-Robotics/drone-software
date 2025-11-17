
#include <iostream>
#include "interfaceutils.h"

WindowInitializer winInit;
Widgets widgets;


int main(int argc, char **argv) {
    float armButton = false;
    ImU32 armColor = IM_COL32(26, 204, 26, 255); // Green color
    const char* armText = "Arm";
    glfwSetErrorCallback([](int error, const char* description) {
        fprintf(stderr, "GLFW Error %d: %s\n", error, description);
    });

    if (!glfwInit())
        return 1;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Find screen resolution
    winInit.GetPrimaryMonitorResolution(windowVar::monitor_w, windowVar::monitor_h);
    windowVar::display_w = windowVar::monitor_w;
    windowVar::display_h = windowVar::monitor_h;

    winInit.Setup();
    GLFWwindow* window = glfwCreateWindow(windowVar::display_w, windowVar::display_h, "Thyra", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    winInit.loadFonts(); // Load fonts once
    


    // Register the callback with GLFW
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        //std::cout << "Test loop running..1" << std::endl;
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        // Draw multi-color background
        winInit.DrawMultiColor();
        // Set size of font and windows - - - - - - - - - - - - - - - - - - - - - - 

        //Update
        winInit.UpdateWindowSize();
       
        // Set the GLFW window size
  
        glfwSetWindowSize(window, windowVar::display_w, windowVar::display_h);
        //std::cout << "Window size set to: " << windowVar::display_w << "x" << windowVar::display_h << std::endl;

        // interface area - - - - - - - - - - - - - - - - - - - - - - - 
        
        ImDrawList* draw_list = ImGui::GetForegroundDrawList();
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2((float)windowVar::display_w, (float)windowVar::display_h));
        ImGui::Begin("GCS Interface", nullptr,
                     ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoBackground |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);   

        // Example content
       
        if (armButton) {
            armColor = IM_COL32(204, 26, 26, 255); // Red color
            armText = "Disarm";
        } else {
            armColor = IM_COL32(26, 204, 26, 255); // Green color
            armText = "Arm";
        }
        if (widgets.costum_square_button(armText, ImVec2(800, 50), ImVec2(150, 50), winInit.getFont(28), 28.0f, armColor)) {

            armButton = !armButton; // Toggle button state
            printf("Arm button clicked. New state: %s\n", armButton ? "Armed" : "Disarmed");
        }
        
        if (widgets.DrawCircleGradientButton(draw_list, winInit.getFont(40), 1.0f, ImVec2(1500, 100), 75.0f, "ESTOP", 40.0f)) {
            std::cout << "ESTOP Button Clicked!" << std::endl;
        }

        //ImGui::PushFont(ImGui::GetFont());
        //ImGui::SetWindowFontScale(3.0f); // 150% text size
        

        ImGui::SetWindowPos(ImVec2(20,20));
        ImGui::PushFont(winInit.getFont(18));
        ImGui::Text("Thyra Ground Control Station");
        ImGui::PopFont(); 
        //ImGui::PopFont(); 

        ImGui::End();


        // Rendering
        ImGui::Render();
        glViewport(0, 0, windowVar::display_w, windowVar::display_h);
       
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

  

    return 0;
}