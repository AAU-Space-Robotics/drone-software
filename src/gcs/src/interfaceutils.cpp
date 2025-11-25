#include "interfaceutils.h"

namespace windowVar {
    int monitor_w = 0;
    int monitor_h = 0;
    int display_w = 0;
    int display_h = 0;
    ImVec4 BackgroundColor = ImVec4(0.0, 0.0, 0.0, 0.0); // Transparent background in RGBA
}

void WindowInitializer::DrawMultiColor() {
    // Example function to demonstrate additional functionality
    ImVec2 window_pos(0, 0);
    ImVec2 window_size((float)windowVar::display_w, (float)windowVar::display_h);

    // Get draw list for the background
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();

    // Define colors (RGBA)
    ImU32 color_top_left = IM_COL32(0, 0, 100, 255);      
    ImU32 color_top_right = IM_COL32(0, 0, 100, 255);   
    ImU32 color_bottom_left = IM_COL32(0, 0, 20, 255);     
    ImU32 color_bottom_right = IM_COL32(0, 0, 20, 255);

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

void WindowInitializer::GetPrimaryMonitorResolution(int& width, int& height) {
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);
    width = mode->width;
    height = mode->height;
    
}
void WindowInitializer::Setup() {
    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Setup style
    ImGui::StyleColorsDark();
}

void WindowInitializer::UpdateWindowSize(float scale) {
    // Update the display size
    windowVar::display_w = static_cast<int>(windowVar::monitor_w * scale);
    windowVar::display_h = static_cast<int>(windowVar::monitor_h * scale);
}



void WindowInitializer::Render() {
    // Rendering code here
}

void WindowInitializer::loadFonts()
{
    ImGuiIO& io = ImGui::GetIO();

    font18 = io.Fonts->AddFontFromFileTTF("src/gcs/fonts/Roboto-Regular.ttf", 18.0f);
    font24 = io.Fonts->AddFontFromFileTTF("src/gcs/fonts/Roboto-Bold.ttf", 24.0f);
    font28 = io.Fonts->AddFontFromFileTTF("src/gcs/fonts/Roboto-Regular.ttf", 28.0f);
    font40 = io.Fonts->AddFontFromFileTTF("src/gcs/fonts/Roboto-Bold.ttf", 40.0f);

    io.Fonts->Build();

    io.FontDefault = font18;   // set default font

}
ImFont* WindowInitializer::getFont(int size)        
{
    switch (size) {
        case 18:
            return font18;
        case 24:
            return font24;
        case 28:
            return font28;
        case 40:
            return font40;
        default:
            return font18; // default to font18 if size not found
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
} //This may not be in a class to work ??


bool Widgets::costum_square_button(const char* id, ImVec2 pos, ImVec2 size, ImFont* font, float font_size, ImU32 color)
{
    ImGuiIO& io = ImGui::GetIO();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    float rounding = 12.0f; // corner rounding radius
    // Button bounding box
    ImRect rect(pos, ImVec2(pos.x + size.x, pos.y + size.y));

    // Detect hover / click manually
    bool hovered = rect.Contains(io.MousePos);
    bool clicked = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left);

 // Define corner colors (RGBA)
    ImU32 col_tr = DarkenColor(color, 0.4f); // slightly darker
    ImU32 col_br = DarkenColor(color, 0.4f);
    ImU32 col_tl = color; // keep original
    ImU32 col_bl = color;


    if (hovered)
    {
        // Example hover effect: lighten colors
        col_tl = DarkenColor(color, 1.2f);
        col_bl = DarkenColor(color, 1.2f);
        col_tr = DarkenColor(color, 1.2f);
        col_br = DarkenColor(color, 1.2f); 
    }

     //Draw colorful gradient rectangle
    draw_list->AddRectFilledMultiColor(
        rect.Min, rect.Max,
        col_tl,
        col_tr,
        col_bl,
        col_br
    );
    float border = 2.5f;

    ImVec2 border_rect_min = ImVec2(rect.Min.x - border, rect.Min.y - border);
    ImVec2 border_rect_max = ImVec2(rect.Max.x + border, rect.Max.y + border);
    draw_list->AddRect(border_rect_min, border_rect_max, IM_COL32(0, 0, 0, 255), rounding, 0, 3.2f);
    
    // Draw outline
    ImVec2 text_size = font ? font->CalcTextSizeA(font_size, FLT_MAX, 0.0f, id)
                             : ImGui::CalcTextSize(id);
    ImVec2 text_pos = ImVec2(
        pos.x + (size.x - text_size.x) * 0.5f,
        pos.y + (size.y - text_size.y) * 0.5f
    );

    // Draw text
    draw_list->AddText(font, font_size, text_pos, IM_COL32(255,255,255,255), id);
  

    return clicked;
}


ImU32 DarkenColor(ImU32 col, float factor)
{
    // Extract RGBA components
    unsigned char r = (col >> IM_COL32_R_SHIFT) & 0xFF;
    unsigned char g = (col >> IM_COL32_G_SHIFT) & 0xFF;
    unsigned char b = (col >> IM_COL32_B_SHIFT) & 0xFF;
    unsigned char a = (col >> IM_COL32_A_SHIFT) & 0xFF;

    // Darken by factor (0..1)
    r = (unsigned char)(r * factor);
    g = (unsigned char)(g * factor);
    b = (unsigned char)(b * factor);

    return IM_COL32(r, g, b, a);
}

bool Widgets::DrawCircleGradientButton(ImDrawList* draw_list, ImFont* font, float scale, ImVec2 center, float radius,const char* label, float font_size)
{
    // Compute bounding box
    ImVec2 bb_min = ImVec2(center.x - radius, center.y - radius);
    ImVec2 bb_max = ImVec2(center.x + radius, center.y + radius);

    // Hover/active detection
    bool hovered = ImGui::IsMouseHoveringRect(bb_min, bb_max);
    bool active = hovered && ImGui::IsMouseDown(0);


    ImVec4 white_color  = ImVec4(0.9f, 0.1f, 0.1f, 0.3f);
    ImVec4 estop_color  = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    ImVec4 hovered_color= ImVec4(1.0f, 0.5f, 0.5f, 1.0f);
    ImVec4 active_color = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);

    // State choose
    ImVec4 base_color = active ? active_color :
                        hovered ? hovered_color : estop_color;

    // Draw layered radial gradient 
    const int num_segments = 64;

    for (int i = 0; i < num_segments; i++)
    {
        float t = (float)i / (num_segments - 1);

        // Color interpolation
        ImVec4 c;
        c.x = white_color.x + (base_color.x - white_color.x) * t;
        c.y = white_color.y + (base_color.y - white_color.y) * t;
        c.z = white_color.z + (base_color.z - white_color.z) * t;
        c.w = white_color.w + (base_color.w - white_color.w) * t;

        float r = radius * (1.0f - t); // decreasing radius

        draw_list->AddCircleFilled(
            center,
            r,
            ImGui::ColorConvertFloat4ToU32(c),
            64
        );
    }

    // Outer border circle
    draw_list->AddCircle(
        center,
        radius,
        IM_COL32(0, 0, 0, 255), // white 0.5 alpha
        64,
        3.0f * scale
    );

    // Draw label in center
    ImGui::PushFont(font);

    ImVec2 text_size = ImGui::CalcTextSize(label);
    ImVec2 text_pos = ImVec2(
        center.x - text_size.x * 0.5f,
        center.y - text_size.y * 0.5f
    );

    draw_list->AddText(text_pos, IM_COL32(255,255,255,255), label);

    ImGui::PopFont();

    return active;
}


void scroll_wheel(ImDrawList* draw_list, float startx, float starty, float width, float height, float scale) {

   
    draw_list->AddCircleFilled(ImVec2(startx, starty), 100.0f, IM_COL32(255, 255, 255, 255), 20); // Draw white circle at (400,200) with radius 30 



}
void AltitudeTape(float altitude, float tapeHeight = 300.0f, float numStep = 20.0f)
{
    ImGui::BeginChild("AltitudeTape", ImVec2(80, tapeHeight), true);

    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 pos = ImGui::GetWindowPos();
    ImVec2 size = ImGui::GetWindowSize();

    float centerY = pos.y + size.y / 2.0f;

    // How many numbers above/below center to draw
    int range = 10;

    // Determine the altitude number nearest to center
    float nearest = roundf(altitude / numStep) * numStep;

    // Vertical offset for smooth scrolling
    float offset = (altitude - nearest) * 4.0f;  
    // adjust 4.0f scaling to match your desired scroll speed/size

    for (int i = -range; i <= range; i++)
    {
        float value = nearest + i * numStep;
        float y = centerY + (i * 40.0f) + offset;

        // Draw text centered
        char buf[16];
        snprintf(buf, sizeof(buf), "%.0f", value);

        draw->AddText(
            ImVec2(pos.x + 20, y - ImGui::CalcTextSize(buf).y / 2),
            IM_COL32(255, 255, 255, 255),
            buf
        );

        // small horizontal tick mark
        draw->AddLine(
            ImVec2(pos.x + 5, y),
            ImVec2(pos.x + 18, y),
            IM_COL32(255, 255, 255, 255),
            2.0f
        );
    }

    // Draw the center reference line
    draw->AddLine(
        ImVec2(pos.x, centerY),
        ImVec2(pos.x + size.x, centerY),
        IM_COL32(255, 255, 0, 255),
        3.0f
    );

    ImGui::EndChild();
}
