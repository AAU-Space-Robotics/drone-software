
#include <iostream>
#include "interfaceutils.h"

int main(int argc, char **argv) {
    glfwSetErrorCallback([](int error, const char* description) {
        fprintf(stderr, "GLFW Error %d: %s\n", error, description);
    });

    if (!glfwInit())
        return 1;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Find screen resolution
    GetPrimaryMonitorResolution(windowVar::monitor_w, windowVar::monitor_h);
    windowVar::display_w = windowVar::monitor_w;
    windowVar::display_h = windowVar::monitor_h;

    Setup();

    GLFWwindow* window = glfwCreateWindow(windowVar::display_w, windowVar::display_h, "Thyra", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    


    // Register the callback with GLFW
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        // Draw multi-color background
        DrawMultiColor();
        // Set size of font and windows - - - - - - - - - - - - - - - - - - - - - - 

        //Update
        UpdateWindowSize();

       
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
        ImGui::Text("Ground Control Station Interface");
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