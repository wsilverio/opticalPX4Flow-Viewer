#include <stdio.h>
#include <opencv2/opencv.hpp> // openCV

#include <common/mavlink.h>     // Protocolo Mavlink
#include "libs/serial_port.h"        // Serial_Port
#include "libs/px4flow_interface.h"  // PX4Flow_Interface

#define Constrain(VAL, LOW, HIGH) ( (VAL)<(LOW)? (LOW):( (VAL)>(HIGH)? (HIGH):(VAL) ) )

Serial_Port *serial_port_quit;
PX4Flow_Interface *px4flow_interface_quit;

float Map(float value, float inputMin, float inputMax, float outputMin, float outputMax){
    if (fabs(inputMin - inputMax) < FLT_EPSILON)
        return outputMin;
    else
        return outputMin + (outputMax - outputMin) * ((value - inputMin) / (inputMax - inputMin));
}

void Quit_Handler(int sig){

    printf("\n\n### PEDIDO DE TÉRMINO DE EXECUÇÃO ###\n\n");

    try {
        px4flow_interface_quit->handle_quit(sig);
    }
    catch (int error){}

    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}

    try {
        cv::destroyAllWindows();
    }
    catch (int error){}    

    exit(EXIT_SUCCESS);
}

int main(int argc, char const *argv[])
{
    system("clear");

    char *uart_name = (char*)"/dev/ttyACM0";
    int baudrate = 921600;
    bool debug = false;

    int msgID = MAVLINK_MSG_ID_ENCAPSULATED_DATA;
    int msgFieldName = 0; // VIDEO_ONLY

    Serial_Port serial_port(uart_name, baudrate, false);
    serial_port_quit = &serial_port;

    PX4Flow_Interface px4flow(&serial_port, msgID, msgFieldName, debug);
    px4flow_interface_quit = &px4flow;

    signal(SIGINT, Quit_Handler);

    #define windowName "OPTICAL FLOW"
    cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);

    serial_port.start();
    px4flow.start();

    float flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, ground_distance;
    uint64_t time_usec, time_usec_prev = 0;
    
    double delta_time, X_accum = 0, Y_accum = 0;

    cv::Mat imagemPlot = cv::Mat(cv::Size(600, 600), CV_8UC3, cv::Scalar(66, 59, 55));

    while(1){

        time_usec = px4flow.current_messages.time_stamps.optical_flow_rad;

        flow_x = px4flow.current_messages.optical_flow.flow_x;
        flow_y = px4flow.current_messages.optical_flow.flow_y;
        flow_comp_m_x = px4flow.current_messages.optical_flow.flow_comp_m_x;
        flow_comp_m_y = px4flow.current_messages.optical_flow.flow_comp_m_y;
        ground_distance = px4flow.current_messages.optical_flow.ground_distance;

        // printf( 
        //         "time_usec: %lu\n"
        //         "flow_x: %f\n"
        //         "flow_y: %f\n"
        //         "flow_comp_m_x: %f\n"
        //         "flow_comp_m_y: %f\n"
        //         "ground_distance: %f\n\n",
        //         time_usec,
        //         flow_x,
        //         flow_y,
        //         flow_comp_m_x,
        //         flow_comp_m_y,
        //         ground_distance
        //         );


        if(time_usec_prev == time_usec) continue;

        delta_time = (time_usec - time_usec_prev) / 1e6;
        // printf("delta_time: %f\n", delta_time);

        if(delta_time < 0.1){
            X_accum += flow_comp_m_x * delta_time;
            Y_accum += flow_comp_m_y * delta_time;
        } 

        time_usec_prev = time_usec;

        // printf(
        //     "x pos: %f\n"
        //     "y pos: %f\n\n",
        //     X_accum, Y_accum);

        cv::circle(imagemPlot, cv::Point( Map( Constrain(X_accum,-5,5), -5, 5, 0, imagemPlot.cols), Map( Constrain(Y_accum,-5,5), -5, 5, 0, imagemPlot.rows)), 1, cv::Scalar(242, 242, 242), -1, CV_AA, 0);


        // do{
            char c = (char) cvWaitKey(1); // 1ms
            if(c == 27) Quit_Handler(c); // ESC key

        // }while(not px4flow.img.rows && not px4flow.img.cols);

        // px4flow.img.copyTo(imagemPlot);

        // // Exibe a imagem
        cv::imshow(windowName, imagemPlot);
    }

    return 0;
}