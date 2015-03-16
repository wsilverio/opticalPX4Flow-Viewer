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
    exit(1);

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

    #define CURRENT_POS "PX4FLOW"
    cv::namedWindow(CURRENT_POS, CV_WINDOW_AUTOSIZE);

    #define IMAGE_DATA "Image"
    cv::namedWindow(IMAGE_DATA, CV_WINDOW_AUTOSIZE);    

    serial_port.start();
    px4flow.start();

    float   flow_x, flow_y, 
            flow_comp_m_x, flow_comp_m_y,
            integrated_xgyro, integrated_ygyro, integrated_zgyro,
            ground_distance;

    uint64_t time_usec, time_usec_prev = 0;
    double delta_time, X_accum = 0, Y_accum = 0;
    uint8_t quality;

    cv::Scalar bgColor(66, 59, 55);
    cv::Scalar fillColor(242, 242, 242);

    char *frase[3] = {"accumulated position", "flow vector (dezi-pixels)", "ground distance (m)"};
    std::vector<cv::Rect> rects(3);

    #define HEIGHT 300
    #define MARGEM 30
    #define TAM (HEIGHT - 2*MARGEM)
    #define WIDTH 4*MARGEM + 3*TAM

    for (int i = 0; i < rects.size(); ++i)
        rects[i] = cv::Rect((i+1)*MARGEM + i*TAM - 1, MARGEM - 1, TAM, TAM);

    cv::Mat currentPosImg = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC3, bgColor);

    #define POS_VECTOR_SIZE 100
    std::vector<cv::Point2f> accPosVector;

    enum RECT_NAME{
        AP, // accumulated_position
        FV, // flow_vector
        GD  // ground_distance
    };

    time_usec_prev = get_time_usec();

    while(1){

        // // atualiza os dados
        // time_usec = px4flow.current_messages.time_stamps.optical_flow_rad;
        // flow_x = px4flow.current_messages.optical_flow.flow_x;
        // flow_y = px4flow.current_messages.optical_flow.flow_y;
        // flow_comp_m_x = px4flow.current_messages.optical_flow.flow_comp_m_x;
        // flow_comp_m_y = px4flow.current_messages.optical_flow.flow_comp_m_y;
        // ground_distance = px4flow.current_messages.optical_flow.ground_distance;


        // // if(time_usec_prev == time_usec) continue;

        // delta_time = (time_usec - time_usec_prev) / 1e6;
        // // printf("delta_time: %f\n", delta_time);

        // if(delta_time < 0.1){ // aguarda uma fração de segundo
        //     X_accum += flow_comp_m_x * delta_time;
        //     Y_accum += flow_comp_m_y * delta_time;
        // } 

        // time_usec_prev = time_usec;

        time_usec = get_time_usec();
        flow_x = px4flow.current_messages.optical_flow_rad.integrated_x;
        flow_y = px4flow.current_messages.optical_flow_rad.integrated_y;
        integrated_xgyro = px4flow.current_messages.optical_flow_rad.integrated_xgyro;
        integrated_ygyro = px4flow.current_messages.optical_flow_rad.integrated_ygyro;
        integrated_zgyro = px4flow.current_messages.optical_flow_rad.integrated_zgyro;
        ground_distance = px4flow.current_messages.optical_flow_rad.distance;
        quality = px4flow.current_messages.optical_flow_rad.quality;

        delta_time = (time_usec - time_usec_prev) / 1e6;

        if(delta_time > 0.1)
        {

            float pixel_x = flow_x + integrated_xgyro;
            float pixel_y = flow_y + integrated_ygyro;

            float velocity_x = pixel_x * ground_distance / delta_time;
            float velocity_y = pixel_y * ground_distance / delta_time;

            X_accum += velocity_x * 100;
            Y_accum += velocity_y * 100;

            // reseta a imagem
            currentPosImg = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC3, bgColor);

            // reescreve os retângulos e seus rótulos
            for (int i = 0; i < rects.size(); ++i){
                cv::rectangle(currentPosImg, rects[i], fillColor, 1, 8, 0);
                putText(currentPosImg, frase[i], cv::Point(rects[i].x, HEIGHT - MARGEM/2.0), cv::FONT_HERSHEY_SIMPLEX, 0.4, fillColor, 1, CV_AA);
            }

            // atualiza o buffer de posição
            accPosVector.push_back(cv::Point2f( Map( Constrain(X_accum,-5,5), -5, 5, 0, TAM) + rects[AP].x + TAM/2,
                                                Map( Constrain(Y_accum,-5,5), -5, 5, 0, TAM) + rects[AP].y + TAM/2));

            // limita sua posição na tela
            accPosVector.back() = cv::Point(Constrain(accPosVector.back().x, rects[AP].x + 4, rects[AP].x + TAM - 4),
                                            Constrain(accPosVector.back().y, rects[AP].y + 4, rects[AP].y + TAM - 4));

            // limita o tamanho do buffer
            if(accPosVector.size() > POS_VECTOR_SIZE)
                accPosVector.erase(accPosVector.begin());

            // desenha o buffer de posição
            cv::circle( currentPosImg, accPosVector.back(), 3, fillColor, 1, CV_AA, 0);
            if (accPosVector.size() > 1){
                for (int i = 1; i < accPosVector.size(); ++i)
                    line(currentPosImg, accPosVector[i], accPosVector[i-1], fillColor, 1, CV_AA, 0);
            }

            // const cv::Point *pts = (const cv::Point*) cv::Mat(accPosVector).data;
            // int npts = accPosVector.size();
            // polylines(currentPosImg, &pts, &npts, 1, true, fillColor, 1, CV_AA, 0);

            // desenha o vetor do optical flow
            #define SCALE 0.25
            cv::Point2f center(rects[FV].x + TAM/2.0, rects[FV].y + TAM/2.0);
            cv::line(currentPosImg, center, center - cv::Point2f(flow_x * SCALE, flow_y * SCALE), fillColor, 1, CV_AA, 0);
            cv::line(currentPosImg, center, center - cv::Point2f(flow_x * SCALE, 0), cv::Scalar(52, 41, 166), 1, CV_AA, 0);
            cv::line(currentPosImg, center, center - cv::Point2f(0, flow_y * SCALE), cv::Scalar(78, 104, 18), 1, CV_AA, 0);

            // desenha a barra de distância
            #define GND_DIST_MAX 5
            float distConstr = Constrain( Map(ground_distance, 0, GND_DIST_MAX, 0, TAM), 0.3/5.0*TAM, TAM);
            cv::rectangle(currentPosImg, cv::Rect(rects[GD].x + TAM/3.0, rects[FV].y + TAM - distConstr, TAM/3.0, distConstr), fillColor, -1, 8, 0);

            char c = (char) cvWaitKey(1); // 1ms
            if(c == 27) Quit_Handler(c); // ESC key

            if(px4flow.img.rows && px4flow.img.cols){
                cv::Mat imagePlot;
                px4flow.img.copyTo(imagePlot);
                cv::imshow(IMAGE_DATA, imagePlot);        
            }
            
            time_usec_prev = time_usec;
        }

        // Exibe a imagem
        cv::imshow(CURRENT_POS, currentPosImg);

    }

    return 0;
}