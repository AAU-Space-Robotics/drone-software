#include <chrono>
#include <time.h>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "drone/msg/vicon_data.hpp"
// #include "drone/msg/drone_control_data.hpp"
#include "drone/msg/drone_command.hpp"

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        // Subscribe to altitude reference and measurement topics
        Data_subscription_ = this->create_subscription<drone::msg::ViconData>("/ViconData", 10, std::bind(&ControllerNode::DataCallback, this, std::placeholders::_1)); //KAMERA

        // Publish regulated altitude control value
        Control_publisher_ = this->create_publisher<drone::msg::DroneCommand>("/cmd_fc", 10);
        

        // Start the control loop in a separate thread
        control_loop_thread_ = std::thread(&ControllerNode::ControlLoop, this);
    }

private:
    // Variables to hold the output error after converting from global to local coordinates
    float local_error_x;
    float local_error_y;

    // XY controller function variables
    // Outputs
    float regulator_pitch_value;
    float regulator_roll_value;

    // Variables to store previous error
    float prev_x_error = 0;
    float prev_y_error = 0;

    // Z controller
    // Output
    float regulator_altitude_value;

    // Previous error value
    float prev_z_error = 0;

    // Yaw controller
    // Outputs
    float regulator_yaw_value;

    // Variable to check if waypoint is reached and new data is requested
    bool data_request = true;

    
    //*Task test
    //const static int array_size = 8;                           // size of array
    //float x_ref_list[array_size] = {698, -1538, 818, 818, 818, -1479, 698, 698};
    //float y_ref_list[array_size] = {-851, 960, 1115, 1115, 1115, -1161, -851, -851};
    //float z_ref_list[array_size] = {500, 1500, 500, 0, 500, 1500, 500, 0};
    //float yaw_ref_list[array_size] = {0, 0, 0, 0, 0, 0, 0, 0}; //Ref is in radians
    //int ghetto_wait[array_size] = {100, 500, 100, 100, 100, 500, 100, 100}; //Time to wait in point. 100 = 1 second when sample time is 0.01


    //*Task test new coords 28-05
    const static int array_size = 13;                           // size of array
    float x_ref_list[array_size] = {1900, 775, -300, 780, 2000, 2000, 2000, 2000, 850, -190, 775, 1900, 1900};
    float y_ref_list[array_size] = {-400, 525, 1500, 1530, 1550, 1550, 1550, 1550, 450, -500, -425, -400, -400};
    float z_ref_list[array_size] = {500, 1000, 1500, 1000, 500,  0,    500, 500, 1000, 1500, 1000, 500,   0};
    float yaw_ref_list[array_size] = {0,  0,    0,     0,    0,  0,     0,   0,    0,    0,    0,    0,   0}; //Ref is in radians
    int ghetto_wait[array_size] = {100,   1,    500,   1,   100, 100,  100, 100,   1,   500,   1,  100, 100}; //Time to wait in point. 100 = 1 second when sample time is 0.01



    int array_counter = 0;        // counter for array

    // Variables that hold individual reference points while running the controller
    float x_ref;
    float y_ref;
    float z_ref;
    float yaw_ref;
    float x_ref_old = 0;
    float y_ref_old = 0;
    float z_ref_old = 0;
    float total_error;
    float z_ref_signal;

    //Variables that hold current position
    float current_x;
    float current_y;
    float current_z;
    float current_yaw;

    //Publish varible
    int cmd_auto_land = 0;

    // Variables to hold previous filter value
    float prev_filter_val = 0;   // DER SKAL SQ NOK OPRETES EN FOR HVER SLAGS MÅLT VÆRDI SÅ DER IKKE GÅR GED I DEN

    // Sample time of all controllers
    float sample_time = 0.01;

    // New msg variable
    bool new_msg = false;

    // Defines time variables
    std::chrono::system_clock::time_point disarm_start;
    std::chrono::system_clock::time_point disarm_stop;
    std::chrono::system_clock::time_point time_start;
    std::chrono::system_clock::time_point time_stop;
    std::chrono::system_clock::time_point timestamp;
    std::chrono::system_clock::duration time_since_epoch;
    int ghetto_ur = 0; 
    int ghetto_ref;
    // Subscribers and publishers
    rclcpp::Subscription<drone::msg::ViconData>::SharedPtr Data_subscription_; // KAMERA
    rclcpp::Publisher<drone::msg::DroneCommand>::SharedPtr Control_publisher_;

    //Controller functions
    void DataCallback(const drone::msg::ViconData::SharedPtr msg) // skal ændres hvis vi vil køre på kamera data data
    { 
        current_x = msg->vicon_x; //KAMERA
        current_y = msg->vicon_y;
        current_z = msg->vicon_z;
        current_yaw = msg->vicon_yaw;
        new_msg = true;
    }

    void ControlLoop(){
        while(rclcpp::ok()){
            if(cmd_auto_land == 1){
                disarm_stop = std::chrono::system_clock::now();
                auto disarm_duration = std::chrono::duration_cast<std::chrono::milliseconds>(disarm_stop - disarm_start).count();
                std::cout<< "Auto land: 1"<< std::endl;
                if(disarm_duration > 10000){ //Waits for 10 seconds before sending the arm command
                    cmd_auto_land = 0;
                }
            }
            else if(new_msg == true){
                new_msg = false;
                // Check if data is requested. Reset data and timer if so
                if (data_request == true){
                    // Next waypoint is saved in seperate variable
                    x_ref = x_ref_list[array_counter];
                    y_ref = y_ref_list[array_counter];
                    z_ref = z_ref_list[array_counter]; 
                    yaw_ref = yaw_ref_list[array_counter];
                    ghetto_ref = ghetto_wait[array_counter]; // Set time to wait in point

                    time_start = std::chrono::system_clock::now(); // Timer is reset
                    data_request = false;                          // Reset data request
                    
                    // Only increment if there are more waypoints
                    if(array_counter < array_size-1){
                        array_counter++;
                    }
                }
                std::cout<< "array_counter: " << array_counter-1 << std::endl;
                // Check time now and calculate time duration
                time_stop = std::chrono::system_clock::now();
                auto time_duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start).count();
                // std::cout << "Time duration: " << time_duration << std::endl;

                // XY controller
                // Generates a reference signal according to exponential function
                // Signal function requires current time, final reference signal, and time constant
                float x_ref_signal = ref_signal(time_duration/1000, x_ref, x_ref_old, 1); // time duration is converted to seconds
                float y_ref_signal = ref_signal(time_duration/1000, y_ref, y_ref_old, 1); // time duration is converted to seconds

                // Denoise the recieved position variables
                // float denoised_vicon_x = low_pass(msg->vicon_x, 10);
                // float denoised_vicon_y = low_pass(msg->vicon_y, 10);
                // float denoised_vicon_z = low_pass(msg->vicon_z, 10);
                // float denoised_vicon_yaw = low_pass(msg->vicon_yaw, 10);

                // recieves the reference signal and the current position and calculates the local error
                globalErrorToLocalError(x_ref_signal, y_ref_signal, current_x, current_y, current_yaw);
                // recieves the local error and calculates controller values for roll and pitch
                XY_controller(local_error_x, local_error_y);
                

                // Z controller
                // Generates reference signal
                if (z_ref != 0){
                    z_ref_signal = ref_signal(time_duration/1000, z_ref, z_ref_old, 2); // time duration is converted to seconds
                    total_error = abs((current_x + current_y + current_z) - (x_ref + y_ref + z_ref));
                }
                else{
                    z_ref_signal = ref_signal(time_duration/1000, 100, z_ref_old, 4); //100 because it is 3 cm below floor height
                    total_error = abs((current_z) - (130));
                }
                std::cout << "total_error: " << total_error << std::endl;
                // Generates controller value for altitude
                Z_controller(z_ref_signal, current_z);
                // Yaw controller
                // Generates reference signal
                float yaw_signal = ref_signal(time_duration/1000, yaw_ref, 0, 1); // time duration is converted to seconds
                // Generates controller value for yaw 
                yaw_controller(yaw_signal, current_yaw);
                
                if (z_ref == 0 && total_error < 60){
                    ghetto_ur++;
                    if (ghetto_ur > ghetto_ref){
                        cmd_auto_land = 1; //Meaning it sends a request to disarm
                        data_request = true;
                        ghetto_ur = 0;
                    }
                }
                // Check if error is under threshold to request new data
                else if (total_error < 100 && z_ref != 0){   // SKAL SÆTTES TIL AFSTAND LIMIT FØR SKIDTET VIRKER //Ændre til 50
                    ghetto_ur++;
                    if (ghetto_ur > ghetto_ref){
                        data_request = true;    // Reset data request if close to waypoint
                        ghetto_ur = 0;
                        x_ref_old = x_ref;
                        y_ref_old = y_ref;
                        z_ref_old = z_ref;
                    }
                }
                else{
                    data_request = false;   // Still false if not close
                }
                std::cout << "ghetto ur: " << ghetto_ur << std::endl;
                std::cout << "ghetto ref: " << ghetto_ref << std::endl;
                timestamp = std::chrono::system_clock::now();
                time_since_epoch = timestamp.time_since_epoch();
                double time_since_epoch_double = time_since_epoch.count()*pow(10, -9);
                //std::cout << "timestamp: "<< time_since_epoch_double << std::endl;

                auto control_msg = drone::msg::DroneCommand();
                // Publish regulated pitch, roll, thrust, and yaw values
                control_msg.cmd_auto_roll = static_cast<int>(-regulator_roll_value); //(minus)Because of Henriks ligninger /Kamera
                control_msg.cmd_auto_pitch = static_cast<int>(regulator_pitch_value);
                control_msg.cmd_auto_thrust = static_cast<int>(regulator_altitude_value);
                control_msg.cmd_auto_yaw = static_cast<int>(-regulator_yaw_value);  //Minus because fc coordinates system is downwards maybe                
                control_msg.identifier = 1;
                control_msg.timestamp = time_since_epoch_double;
                control_msg.cmd_auto_disarm = cmd_auto_land;
                if(cmd_auto_land == 1){
                    disarm_start = std::chrono::system_clock::now();
                }
                Control_publisher_->publish(control_msg);
            }
        }
    }


    //XY_controller functions    
    void globalErrorToLocalError(float x_ref, float y_ref, float x_global_mes, float y_global_mes, float yaw_mes)
    {
        float x_global_error = x_ref - x_global_mes; //x_global_mes negative because of Vicon //Kamera
        float y_global_error = y_ref - y_global_mes; //y_global_mes negative because of Vicon //Kamera
        x_global_error = -x_global_error; //Kamera
        y_global_error = -y_global_error; //Kamera
        float roll = 0;
        float pitch = 0;
        float yaw = yaw_mes;

        float inv_R[3][3] = {{cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch)},
                             {cos(yaw) * sin(pitch) * sin(roll) - cos(roll) * sin(yaw), cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw), cos(pitch) * sin(roll)},
                             {sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch), cos(roll) * sin(pitch) * sin(yaw) - cos(yaw) * sin(roll), cos(pitch) * cos(roll)}};

        float global_error_vec[3][1] = {{x_global_error},
                                        {y_global_error},
                                        {0}};

        float result[3][1];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 1; j++)
            {
                result[i][j] = 0;

                for (int k = 0; k < 3; k++)
                {
                    result[i][j] += inv_R[i][k] * global_error_vec[k][j];
                }
            }
        }

        local_error_x = result[0][0];
        local_error_y = result[1][0];
    }

    void yaw_controller(float yaw_ref, float yaw_mes)
    // P controller for yaw position
    {
        float Kp_yaw = 150;              // Proportional gain
        float saturation_value = 300;   // Max and min value allowed to be sent to the drone

        float yaw_error = yaw_ref - yaw_mes;  // Error between reference and measurement
        std::cout<<"yaw_error: "<< yaw_error << std::endl;
        float yaw_value = Kp_yaw * yaw_error; // P regulaed value

        regulator_yaw_value = saturation(yaw_value, saturation_value);  // Saturate value
    }

    void Z_controller(float z_ref, float z_mes)
    // PD controller for altitude
    {
        float Kp_altitude = 0.6;       // Proportional gain
        float Kd_altitude = 0.5;         // Derivative gain
        float saturation_value = 100;   // Max and min value allowed to be sent to the drone
        float hover_value = 560;        // controller value for hovering (found by m*g/thrust to newton relation)

        float z_error = z_ref - z_mes;  // Error between reference and measurement
        std::cout<<"z_error: "<< z_error << std::endl;

        float altitude_value = Kd_altitude*((z_error - prev_z_error)/sample_time)+Kp_altitude*z_error; // PD regulated value

        regulator_altitude_value = saturation(altitude_value, saturation_value)+hover_value; // Saturate value and add hover value

        prev_z_error = z_error; // Update previous error
    }

    void XY_controller(float local_x_error, float local_y_error)
    // PD controller for x and y
    {
        std::cout<<"local_x_error: "<< local_x_error << std::endl;
        std::cout<<"local_y_error: "<< local_y_error << std::endl;
        
        // PD controller gains
        float Kp_pitch = 0.5; 
        float Kd_pitch = 1;
        float Kp_roll = 0.5;
        float Kd_roll = 1;

        // Max allowed value (1000 is max max, but we aint chill like that)
        float saturation_value = 500;  //CHANGE

        // Discretized PD controller for x and y
        float pitch_value = (Kd_pitch*(local_x_error-prev_x_error)/sample_time)+local_x_error*(Kp_pitch);
        float roll_value = (Kd_roll*(local_y_error-prev_y_error)/sample_time)+local_y_error*(Kp_roll);

        // Saturate values
        regulator_pitch_value = saturation(pitch_value, saturation_value);
        regulator_roll_value = saturation(roll_value, saturation_value);

        // Update previous errors
        prev_x_error = local_x_error;
        prev_y_error = local_y_error;
    } 

    float saturation(float value, float max_value, int only_positive = 0)
    // Saturate value to + or - max_value
    {
        if (value > max_value)       // Saturate to max value
        {
            value = max_value;
        }
        else if (value < -max_value) // Saturate to minimum
        {
            if (only_positive == 0){ // Check if only positive values are allowed
                value = -max_value;
            }
            else{
                value = 1;           // Smallest allowed value when only saturating to positive numbers
            }
        }
        else                         // Do nothing
        {
            ;
        }
        return value;
    }
    
    float ref_signal(float t, float ref, float old_ref, float delay)
    // Generates a reference signal according to exponential function
    {
        float dif = ref - old_ref;
        float signal = dif*(1-exp(-t/delay))+old_ref;
        return signal;
    }

    float ref_ramp(float t, float ref, float slope, float offset = 0)
    // Generates a reference signal according to ramp function
    {
        float signal = slope*t+offset;
        return saturation(signal, ref);
    }
    float low_pass(float signal, float filter_val)
    // Low pass filter for noise on the measured position signal
    {
        // Discretized low pass filter
        float lowpass_signal = (signal*filter_val*sample_time+prev_filter_val)/(1+filter_val*sample_time);
        // Update previous filter value
        prev_filter_val = lowpass_signal;

        return lowpass_signal;
    }

    //float landing_signal()

    std::thread control_loop_thread_;

};


int main(int argc, char *argv[])
{
    std::cout << "Starting controller node" << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

