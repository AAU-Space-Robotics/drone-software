// gui.cpp

#include "rclcpp/rclcpp.hpp"
#include "gcslib.h"

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

    GLFWwindow* window = glfwCreateWindow(windowVar::display_w, windowVar::display_h, ApplicationTitle.c_str(), nullptr, nullptr);
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

    // Custom fonts
    InitializeFonts();

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



        node->publish_exerciseInfo();
        if (exerciseVar::sendExerciseRequest){
            std::cout << "Exercise request sent" << std::endl;
            node->send_exercise_request();
            exerciseVar::sendExerciseRequest = false;
        }
        else if ( exerciseVar::softLock){ //soft lock
            std::cout << "stop request sent" << std::endl;
            node->send_interrupt_request(2);
            exerciseVar::softLock = false;
        }
        else if (exerciseVar::removeSoftLock){
            std::cout << "remove soft lock request sent" << std::endl;
            node->send_interrupt_request(3);
            exerciseVar::removeSoftLock = false;
        }
        else if ( exerciseVar::pauseExercise){
            std::cout << "pause request sent" << std::endl;
            node->send_interrupt_request(0);
            exerciseVar::pauseExercise = false;
        }
        else if ( exerciseVar::unPauseExercise){
            std::cout << "Unpause request sent" << std::endl;
            node->send_interrupt_request(1);
            exerciseVar::unPauseExercise = false;
        }
        
        // Process ROS messages
        rclcpp::spin_some(node);

        // Make Imgui frame
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Set size of font and windows - - - - - - - - - - - - - - - - - - - - - - 

        // Set the custom font
        SetFont();

        //Update
        UpdateWindowSize();

       
        // Set the GLFW window size
        if (windowVar::updateScreenResolution) {
            glfwSetWindowSize(window, windowVar::display_w, windowVar::display_h);
            windowVar::updateScreenResolution = false;
        }


        // Tabs area - - - - - - - - - - - - - - - - - - - - - - - 
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(windowVar::tabs_w), static_cast<float>(windowVar::tabs_h)));

        ImGui::Begin("Explorer", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        // ImGui UI code for displaying exercises, settings and handling input
        CreateTabsButtons("Tools", AppState::tools, AppState::exerciseVar::selectedTool, TabCategory::Tools, AppState::activeToolsFunction, AppState::windowVar::HighLightColor , ImVec4(0, 0, 0, 0), true);
        
        CreateTabsButtons("Settings", AppState::settings, AppState::exerciseVar::selectedSetting, TabCategory::Settings, AppState::activeSettingsFunction, AppState::windowVar::HighLightColor, ImVec4(0, 0, 0, 0),false);

        ImGui::End(); // End Tabs window

        // Application area - - - - - - - - - - - - - - - - - - - - - - - 

        // Right Side Application Area
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(windowVar::tabs_w), 0));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(windowVar::application_w), static_cast<float>(windowVar::application_h)));

        ImGui::Begin(ApplicationSpaceTitle.c_str(), nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        
        // Call the active exercise function if selected
        if (activeCategory == TabCategory::Tools && AppState::activeToolsFunction != nullptr) {
            exerciseVar::selectedSetting = -1;
            AppState::activeToolsFunction();

        } else if (activeCategory == TabCategory::Settings && AppState::activeSettingsFunction != nullptr) {
            exerciseVar::selectedTool = -1;
            AppState::activeSettingsFunction();
        }

        ImGui::End(); // End Application Space

        // Logging of data - - - - - - - - - - - - - - - - - - - - - - - 
        if (loggingVar::loggingAllowed){
            LogExercise(exerciseVar::jointAngles, exerciseVar::torques, exerciseVar::fullPath);
        }

        // Rendering pipeline - - - - - - - - - - - - - - - - - - - - - - - 

        ImGui::PopFont();

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
