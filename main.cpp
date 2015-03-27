#include <iostream>
#include <stdio.h> // printf
#include <opencv2/opencv.hpp> // openCV
#include <vector>
#include <string>

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
    bool debug_msg = false; // Serial
    bool show_img = false;

    int msgID = MAVLINK_MSG_ID_ENCAPSULATED_DATA;
    int msgFieldName = 0; // VIDEO_ONLY

    Serial_Port serial_port(uart_name, baudrate, false);
    serial_port_quit = &serial_port;

    PX4Flow_Interface px4flow(&serial_port, msgID, msgFieldName, debug_msg);
    px4flow_interface_quit = &px4flow;

    signal(SIGINT, Quit_Handler);

    #define CURRENT_POS "PX4FLOW"
    cv::namedWindow(CURRENT_POS, CV_WINDOW_AUTOSIZE);

    #define IMAGE_DATA "Image"
    if (show_img) cv::namedWindow(IMAGE_DATA, CV_WINDOW_AUTOSIZE);    

    serial_port.start();
    px4flow.start();

    const char *frase[3] = {"accumulated position (m, m)", "speed (m/s)", "ground distance LP (m)"};
    std::vector<cv::Rect> rects(3);

    #define HEIGHT 300
    #define MARGEM 30
    #define TAM (HEIGHT - 2*MARGEM)
    #define WIDTH 4*MARGEM + 3*TAM

    for (int i = 0; i < rects.size(); ++i)
        rects[i] = cv::Rect((i+1)*MARGEM + i*TAM - 1, MARGEM - 1, TAM, TAM);

    cv::Scalar bgColor(66, 59, 55);
    cv::Scalar fillColor(242, 242, 242);

    cv::Mat imagePlot;
    cv::Mat currentPosImg = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC3, bgColor);

    enum RECT_NAME{
        AP,     // accumulated_position
        VEL,    // vel vector
        GD      // ground_distance
    };

    uint64_t time_usec_prev = 0;    // [us]  -> timestamp (desde o boot da PX4Flow) (usado no cálculo do delta_time_sec)
    float delta_time_sec;           // [s]   -> intervalo de tempo dentre os loops (usado no cálc da posição)
    float ground_distance_lp = 0;   // [m]   -> distância do solo com filtro passa baixa (LP) (usada no cálc da posição)
    float X_accum = 0, Y_accum = 0; // [m]   -> posição (x,y) acumulada
    float vel_x = 0, vel_y = 0;     // [m/s] -> vetores de velocidade
    // float teta_ang = 0;             // [rad] -> compensação angular (usado na exibição dos dados)

    #define GND_DIST_MAX 5      // [m]
    #define GND_DIST_MIN 0.3    // [m]
    #define POS_X_MIN -5.0      // [m]
    #define POS_X_MAX +5.0      // [m]
    #define POS_Y_MIN -5.0      // [m]
    #define POS_Y_MAX +5.0      // [m]
    #define VEL_SCALE 50        // apenas para visualização
    #define QUALITY_MIN 100
    #define DT_S_MIN 0.1        // delta time in [s]

    #define POS_VECTOR_SIZE 20
    std::vector<cv::Point2f> accPosVector;
    accPosVector.push_back(cv::Point2f( Map( Constrain(X_accum,POS_X_MIN,POS_X_MAX), POS_X_MIN, POS_X_MAX, 0, TAM) + rects[AP].x,
                                                    Map( Constrain(Y_accum,POS_Y_MIN,POS_Y_MAX), POS_Y_MAX, POS_Y_MIN, 0, TAM) + rects[AP].y));
    // limita sua posição na tela
    accPosVector.back() = cv::Point(Constrain(accPosVector.back().x, rects[AP].x + 4, rects[AP].x + TAM - 4),
                                    Constrain(accPosVector.back().y, rects[AP].y + 4, rects[AP].y + TAM - 4));

    while(1){

        // optical flow fields #100
        // uint64_t time_usec = px4flow.current_messages.time_stamps.optical_flow;
        // // uint64_t time_usec = px4flow.current_messages.optical_flow.time_usec;
        // float flow_comp_m_x = px4flow.current_messages.optical_flow.flow_comp_m_x;
        // float flow_comp_m_y = px4flow.current_messages.optical_flow.flow_comp_m_y;
        float distance = px4flow.current_messages.optical_flow.ground_distance;
        // int16_t flow_x = px4flow.current_messages.optical_flow.flow_x;
        // int16_t flow_y = px4flow.current_messages.optical_flow.flow_y;
        // // uint8_t sensor_id = px4flow.current_messages.optical_flow.sensor_id;
        // uint8_t quality = px4flow.current_messages.optical_flow.quality;

        // optical flow rad fields #106
        uint64_t time_usec = px4flow.current_messages.optical_flow_rad.time_usec;
        // uint64_t time_stamps = px4flow.current_messages.time_stamps.optical_flow_rad;
        uint32_t integration_time_us = px4flow.current_messages.optical_flow_rad.integration_time_us;
        float integrated_x = px4flow.current_messages.optical_flow_rad.integrated_x;
        float integrated_y = px4flow.current_messages.optical_flow_rad.integrated_y;
        float integrated_xgyro = px4flow.current_messages.optical_flow_rad.integrated_xgyro;
        float integrated_ygyro = px4flow.current_messages.optical_flow_rad.integrated_ygyro;
        // float integrated_zgyro = px4flow.current_messages.optical_flow_rad.integrated_zgyro;
        // uint32_t time_delta_distance_us = px4flow.current_messages.optical_flow_rad.time_delta_distance_us;
        // float distance = px4flow.current_messages.optical_flow_rad.distance;
        // int16_t temperature = px4flow.current_messages.optical_flow_rad.temperature;
        // uint8_t sensor_id = px4flow.current_messages.optical_flow_rad.sensor_id;
        uint8_t quality = px4flow.current_messages.optical_flow_rad.quality;

        // ignora a primeira msg
        if (time_usec_prev == 0){
            time_usec_prev = time_usec;
            continue;
        }

        // calcula o intervalo de tempo
        delta_time_sec = (time_usec - time_usec_prev) / 1e6f; // [s]

        if(delta_time_sec > DT_S_MIN) // 100 ms
        {
            time_usec_prev = time_usec;

            if (distance > GND_DIST_MIN && quality >= QUALITY_MIN){

                ground_distance_lp = 0.10f * distance + 0.90f * ground_distance_lp;

                float flow_x = -integrated_y - integrated_xgyro;
                float flow_y = -integrated_x - integrated_ygyro;
                vel_x = flow_x * ground_distance_lp / (float)(integration_time_us / 1e6f);
                vel_y = flow_y * ground_distance_lp / (float)(integration_time_us / 1e6f);
                X_accum += vel_x * delta_time_sec;
                Y_accum += vel_y * delta_time_sec;

                // printf(
                //     "flow x:\t\t\t%f\n"
                //     "flow y:\t\t\t%f\n"
                //     "vel x:\t\t\t%f\n"
                //     "vel y:\t\t\t%f\n"
                //     "X_accum:\t\t%f\n"
                //     "Y_accum:\t\t%f\n"
                //     "distance:\t\t%f\n"
                //     "distance LP:\t\t%f\n"
                //     "integration_time_sec:\t%f\n"
                //     "\n", flow_x, flow_y, vel_x, vel_y, X_accum, Y_accum, distance, ground_distance_lp, (float)integration_time_us/1e6f);

                // atualiza o buffer de posição
                accPosVector.push_back(cv::Point2f( Map( Constrain(X_accum,POS_X_MIN,POS_X_MAX), POS_X_MIN, POS_X_MAX, 0, TAM) + rects[AP].x,
                                                    Map( Constrain(Y_accum,POS_Y_MIN,POS_Y_MAX), POS_Y_MAX, POS_Y_MIN, 0, TAM) + rects[AP].y));

                // limita sua posição na tela
                accPosVector.back() = cv::Point(Constrain(accPosVector.back().x, rects[AP].x + 4, rects[AP].x + TAM - 4),
                                                Constrain(accPosVector.back().y, rects[AP].y + 4, rects[AP].y + TAM - 4));

                // limita o tamanho do buffer
                if(accPosVector.size() > POS_VECTOR_SIZE)
                    accPosVector.erase(accPosVector.begin());

            }

            // reseta a imagem
            currentPosImg = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC3, bgColor);

            // redesenha os retângulos e seus rótulos
            for (int i = 0; i < rects.size(); ++i){
                cv::rectangle(currentPosImg, rects[i], fillColor, 1, 8, 0);
                putText(currentPosImg, frase[i], cv::Point(rects[i].x, HEIGHT - MARGEM/2.0), cv::FONT_HERSHEY_SIMPLEX, 0.4, fillColor, 1, CV_AA);
            }

            // marcadores
            for(int i = 0; i < 10; ++i){
                line(currentPosImg,
                    cv::Point(rects[0].x + i*TAM/10.0, rects[0].y + ((i==5)?-5:0)),
                    cv::Point(rects[0].x + i*TAM/10.0, rects[0].y + 5),
                    fillColor, 0.4, CV_AA, 0);

                line(currentPosImg,
                    cv::Point(rects[0].x + i*TAM/10.0, rects[0].y + TAM - 1 + ((i==5)?5:0)),
                    cv::Point(rects[0].x + i*TAM/10.0, rects[0].y + TAM - 6),
                    fillColor, 0.4, CV_AA, 0);

                line(currentPosImg,
                    cv::Point(rects[0].x + ((i==5)?-5:0), rects[0].y + i*TAM/10.0),
                    cv::Point(rects[0].x + 5, rects[0].y + i*TAM/10.0),
                    fillColor, 0.4, CV_AA, 0);

                line(currentPosImg,
                    cv::Point(rects[0].x + TAM - 1 + ((i==5)?5:0), rects[0].y + i*TAM/10.0),
                    cv::Point(rects[0].x + TAM - 1 - 6, rects[0].y + i*TAM/10.0),
                    fillColor, 0.4, CV_AA, 0);
            }

            // desenha o buffer de posição
            cv::circle( currentPosImg, accPosVector.back(), 3, fillColor, 1, CV_AA, 0);
           if (accPosVector.size() > 1){
                for (int i = 1; i < accPosVector.size(); ++i)
                    line(currentPosImg, accPosVector[i], accPosVector[i-1], fillColor, 0.4, CV_AA, 0);
            }

            // const cv::Point *pts = (const cv::Point*) cv::Mat(accPosVector).data;
            // int npts = accPosVector.size();
            // polylines(currentPosImg, &pts, &npts, 1, true, fillColor, 1, CV_AA, 0);

            // desenha o vetor do optical flow
            cv::Point2f center(rects[VEL].x + TAM/2.0, rects[VEL].y + TAM/2.0);

            cv::line(currentPosImg, center, center - cv::Point2f( 
                Constrain(vel_x * VEL_SCALE,-TAM/2.0,TAM/2.0),
                Constrain(vel_y * VEL_SCALE,-TAM/2.0,TAM/2.0)),
                fillColor, 1, CV_AA, 0);

            cv::line(currentPosImg, center, center - cv::Point2f(
                Constrain(vel_x * VEL_SCALE,-TAM/2.0,TAM/2.0),
                0),
                cv::Scalar(52, 41, 166), 1, CV_AA, 0);

            cv::line(currentPosImg, center, center - cv::Point2f(
                0,
                Constrain(vel_y * VEL_SCALE,-TAM/2.0,TAM/2.0)),
                cv::Scalar(78, 104, 18), 1, CV_AA, 0);

            // desenha a barra de distância
            float distConstr = Constrain( Map(ground_distance_lp, 0, GND_DIST_MAX, 0, TAM), GND_DIST_MIN/GND_DIST_MAX*TAM, TAM);
            cv::Rect rect_GD = cv::Rect(rects[GD].x + TAM/3.0, rects[VEL].y + TAM - distConstr, TAM/3.0, distConstr);
            cv::rectangle(currentPosImg, rect_GD, fillColor, -1, 8, 0);

            std::string text_GD = std::to_string(ground_distance_lp);
            text_GD = text_GD.substr(0, text_GD.find(".")+3);
            putText(currentPosImg, text_GD, cv::Point(rect_GD.x, rect_GD.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.4, fillColor, 1, CV_AA);

            if(px4flow.img.rows && px4flow.img.cols && show_img){
                px4flow.img.copyTo(imagePlot);
                cv::imshow(IMAGE_DATA, imagePlot);        
            }
            
        }

        // Exibe a imagem
        cv::imshow(CURRENT_POS, currentPosImg);

        char c = (char) cvWaitKey(1); // 1 ms
        if(c == 27){
            Quit_Handler(c); // ESC key
        }
        else if(c == 'r' || c == 'R'){
            // teta_ang = 0;
            // reset -> center position
            accPosVector.clear();
            X_accum = Y_accum = 0;
            accPosVector.push_back(cv::Point2f( Map( Constrain(X_accum,POS_X_MIN,POS_X_MAX), POS_X_MIN, POS_X_MAX, 0, TAM) + rects[AP].x,
                                                Map( Constrain(Y_accum,POS_Y_MIN,POS_Y_MAX), POS_Y_MAX, POS_Y_MIN, 0, TAM) + rects[AP].y));
        }

    }

    return 0;
}