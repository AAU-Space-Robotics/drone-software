// gui.cpp

#include "rclcpp/rclcpp.hpp"
#include "gcslib.h"

using namespace AppState;

// Define the ROS2 Node class
class GCSNode : public rclcpp::Node {
public:
    GCSNode() : Node("gcs_node") {
        RCLCPP_INFO(this->get_logger(), "gcs Node has been started.");

        //make publisher and subscriber
    }

    
    // define in memory
};

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    windowVar::display_w = width;  // Update the stored width
    windowVar::display_h = height; // Update the stored height
    glViewport(0, 0, width, height); // Adjust OpenGL viewport to new window size
}

// Main code
int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    auto node = std::make_shared<GCSNode>();

    // Initialize GLFW
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

    // Setup GCS
    Setup();

    GLFWwindow* window = glfwCreateWindow(windowVar::display_w, windowVar::display_h, "GCS", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Set custom colors for the title bar to always appear highlighted
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, windowVar::BackgroundColor);
    ImGui::PushStyleColor(ImGuiCol_TitleBg, windowVar::BackgroundColor);
    ImGui::PushStyleColor(ImGuiCol_TitleBgCollapsed, windowVar::BackgroundColor);

    // Set custom color for the dropdown menus
    ImGui::PushStyleColor(ImGuiCol_Header, windowVar::BackgroundColor);
    ImGui::PushStyleColor(ImGuiCol_Button, windowVar::BackgroundColor);


    // Register the callback with GLFW
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);



    // Main loop - - 
    while (!glfwWindowShouldClose(window)) {
       
        // Ros 2 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        // Process ROS messages
        rclcpp::spin_some(node);

        // Make Imgui frame
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Set size of font and windows - - - - - - - - - - - - - - - - - - - - - - 

        //Update
        UpdateWindowSize();

       
        // Set the GLFW window size
  
        glfwSetWindowSize(window, windowVar::display_w, windowVar::display_h);


        // interface area - - - - - - - - - - - - - - - - - - - - - - - 
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(windowVar::display_w), static_cast<float>(windowVar::display_h)));


        //write code here!!!
        //https://pthom.github.io/imgui_manual_online/manual/imgui_manual.html


        // Rendering
        ImGui::Render();
        glViewport(0, 0, windowVar::display_w, windowVar::display_h);
        glClearColor(windowVar::BackgroundColor.x, windowVar::BackgroundColor.y, windowVar::BackgroundColor.z, windowVar::BackgroundColor.w);
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

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
