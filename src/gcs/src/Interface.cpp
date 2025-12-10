
#include <iostream>
#include "interfaceutils.h"
#include <algorithm>

WindowInitializer winInit;
Widgets widgets;


int main(int argc, char **argv) {
    float armButton = false;
    float value = 0.0f;
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
        //winInit.DrawMultiColor();
        // Set size of font and windows - - - - - - - - - - - - - - - - - - - - - - 

        //Update
        
        const int screen_width = 1920; // Reference screen width
        const int screen_height = 1080; // Reference screen height
        int x_sc, y_sc;
        glfwGetWindowSize(window, &x_sc, &y_sc);
        float scale = std::max(static_cast<float>(x_sc) / screen_width, static_cast<float>(y_sc) / screen_height);
        
        ImGui::GetIO().FontGlobalScale = scale;
        // Set the GLFW window size
        winInit.UpdateWindowSize(scale);
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
       
        //if (armButton) {
        //    armColor = IM_COL32(204, 26, 26, 255); // Red color
        //    armText = "Disarm";
        //} else {
        //    armColor = IM_COL32(26, 204, 26, 255); // Green color
        //    armText = "Arm";
        //}
        //if (widgets.costum_square_button(armText, ImVec2(800 * scale, 50*scale), ImVec2(150 * scale, 50 * scale), winInit.getFont(28), 28.0f * scale, armColor)) {
//
        //    armButton = !armButton; // Toggle button state
        //    printf("Arm button clicked. New state: %s\n", armButton ? "Armed" : "Disarmed");
        //}
        //
        //if (widgets.DrawCircleGradientButton(draw_list, winInit.getFont(40), 1.0f, ImVec2(1500 * scale, 100 * scale), 75.0f * scale, "ESTOP", 40.0f * scale)) {
        //    std::cout << "ESTOP Button Clicked!" << std::endl;
        //}

        //ImGui::PushFont(ImGui::GetFont());
        //ImGui::SetWindowFontScale(3.0f); // 150% text size
        

        ImGui::SetWindowPos(ImVec2(20 * scale,20 * scale));
        ImGui::PushFont(winInit.getFont(18));
        ImGui::Text("Thyra Ground Control Station");
        ImGui::PopFont(); 
        //ImGui::PopFont(); 
        ImGui::SetNextWindowPos(ImVec2((screen_width/2- 200) * scale, (screen_height -500) * scale));  
        ImGui::SetNextWindowSize(ImVec2(200 * scale, 200 * scale));  
       
        ImDrawList* dl = ImGui::GetWindowDrawList();  // draw *inside* this window  
        
        ImGui::SliderFloat("Altitude", &value, 0.0f, 1000.0f);
        AltitudeTape(value, 300.0f * scale, 20.0f);
        
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