#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp> // openCV
#include <vector>

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

    #define ACCUM_POS "Accumulated Position"
    cv::namedWindow(ACCUM_POS, CV_WINDOW_AUTOSIZE);

    #define CURRENT_POS "Current Position"
    cv::namedWindow(CURRENT_POS, CV_WINDOW_AUTOSIZE);    

    serial_port.start();
    px4flow.start();

    float flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, ground_distance;
    uint64_t time_usec, time_usec_prev = 0;
    
    double delta_time, X_accum = 0, Y_accum = 0;

    cv::Scalar bgColor(66, 59, 55);
    cv::Scalar fillColor(242, 242, 242);

    cv::Mat accumPosImg = cv::Mat(cv::Size(600, 600), CV_8UC3, bgColor);

    #define HEIGHT 300
    #define MARGEM 30
    #define TAM (HEIGHT - 2*MARGEM)
    cv::Mat currentPosImg = cv::Mat(cv::Size(4*MARGEM + 3*TAM, HEIGHT), CV_8UC3, bgColor);

    char *frase[3] = {"current position", "flow vector", "ground distance"};
    std::vector<cv::Rect> rects(3);

    for (int i = 0; i < rects.size(); ++i){
        rects[i] = cv::Rect((i+1)*MARGEM + i*TAM - 1, MARGEM - 1, TAM, TAM);
        cv::rectangle(currentPosImg, rects[i], fillColor, 1, 8, 0);
        putText(currentPosImg, frase[i], cv::Point(rects[i].x, HEIGHT - MARGEM/2.0), cv::FONT_HERSHEY_SIMPLEX, 0.4, fillColor, 1, CV_AA);
    }

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

        cv::Point2f pos(
            Map( Constrain(X_accum,-5,5), -5, 5, 0, accumPosImg.cols),
            Map( Constrain(Y_accum,-5,5), -5, 5, 0, accumPosImg.rows)
            );

        cv::circle(accumPosImg, pos, 1, fillColor, -1, CV_AA, 0);

        currentPosImg = cv::Mat(cv::Size(4*MARGEM + 3*TAM, HEIGHT), CV_8UC3, bgColor);
        for (int i = 0; i < rects.size(); ++i){
            cv::rectangle(currentPosImg, rects[i], fillColor, 1, 8, 0);
            putText(currentPosImg, frase[i], cv::Point(rects[i].x, HEIGHT - MARGEM/2.0), cv::FONT_HERSHEY_SIMPLEX, 0.4, fillColor, 1, CV_AA);
        }

        pos = cv::Point(
                Map( pos.x, 0, accumPosImg.cols, rects[0].x, rects[0].x + TAM),
                Map( pos.y, 0, accumPosImg.rows, rects[0].y, rects[0].y + TAM)
                );

        cv::circle( currentPosImg,
                    cv::Point(  Constrain(pos.x, rects[0].x + 4, rects[0].x + TAM - 4),
                                Constrain(pos.y, rects[0].y + 4, rects[0].y + TAM - 4)),
                    3, fillColor, 1, CV_AA, 0);

        #define SCALE 0.25
        cv::Point2f center(rects[1].x + TAM/2.0, rects[1].y + TAM/2.0);
        cv::line(currentPosImg, center, center - cv::Point2f(flow_x * SCALE, flow_y * SCALE), fillColor, 1, CV_AA, 0);
        cv::line(currentPosImg, center, center - cv::Point2f(flow_x * SCALE, 0), cv::Scalar(52, 41, 166), 1, CV_AA, 0);
        cv::line(currentPosImg, center, center - cv::Point2f(0, flow_y * SCALE), cv::Scalar(78, 104, 18), 1, CV_AA, 0);

        #define GND_DIST_MAX 5
        float distConstr = Constrain( Map(ground_distance, 0, GND_DIST_MAX, 0, TAM), 0, TAM);
        cv::rectangle(currentPosImg, cv::Rect(rects[2].x + TAM/3.0, rects[2].y + TAM - distConstr, TAM/3.0, distConstr), fillColor, -1, 8, 0);

        // do{
            char c = (char) cvWaitKey(1); // 1ms
            if(c == 27) Quit_Handler(c); // ESC key

        // }while(not px4flow.img.rows && not px4flow.img.cols);

        // px4flow.img.copyTo(imagePlot);

        // // Exibe a imagem
        cv::imshow(ACCUM_POS, accumPosImg);
        cv::imshow(CURRENT_POS, currentPosImg);
    }

    return 0;
}