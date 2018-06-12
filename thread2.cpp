#include <cv.h>
#include <stdio.h>
#include <highgui.h>
#include <iostream> 
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include "serial.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;
using std::string;

#define TAM 20
#define TAM_4K 50
#define STANDBY 30
#define TH_MASK 30
#define TH_IMG_IGUAL 97
#define TH_NOVO_PREV 200
#define TH_CHAMA_ALARME 50
#define TH_SIMULA_RFID 15000

struct Objeto{
    bool Alarmado;
    long int Pixels;
    long int RFID;
    Mat img;
    int cont_semelhanca;
    int cont_chamada_alarme;
    struct Objeto *Prox;
};

typedef struct Objeto OBJETO;
typedef OBJETO *OBJETO_PTR;

void AddObejto(OBJETO_PTR *ListaObjetos, int Pixels, int RFID, Mat mask){
    OBJETO_PTR novo = new OBJETO;
    novo->Pixels = Pixels;
    novo->RFID = RFID;
    novo->img = mask;
    novo->Prox = NULL;
    novo->cont_semelhanca = 0;
    novo->cont_chamada_alarme = 0;
    
    if (*ListaObjetos == NULL){
        *ListaObjetos = novo;
        return;
    }
    
    OBJETO_PTR atual = *ListaObjetos;
    while(atual->Prox != NULL){
        atual = atual->Prox;
    }
    atual->Prox = novo;
}


void RemoveObjeto(OBJETO_PTR *ListaObjetos, int RFID){
    if (*ListaObjetos == NULL)
        return;
    
    OBJETO_PTR atual = *ListaObjetos;
    OBJETO_PTR prev = NULL;
    
    if ((*ListaObjetos)->RFID == RFID){
        *ListaObjetos = (*ListaObjetos)->Prox;
        free(atual);
        return;
    }
    
    while (atual != NULL && atual->RFID != RFID){
        prev = atual;
        atual = atual->Prox;
    }
    
    if(atual == NULL)
        return;
    
    prev->Prox = atual->Prox;
    free(atual);
}

bool EstaAlarmado(OBJETO_PTR ListaObjetos, int RFID){
    while(ListaObjetos != NULL){
        if (ListaObjetos->RFID == RFID && ListaObjetos->Alarmado)
            return true;
        ListaObjetos = ListaObjetos->Prox;
    }
    return false;
}

bool JaExiste(OBJETO_PTR ListaObjetos, int RFID){
    while(ListaObjetos != NULL){
        if (ListaObjetos->RFID == RFID && ListaObjetos->Alarmado)
            return true;
        ListaObjetos = ListaObjetos->Prox;
    }
    return false;
}

bool TemAlarmado(OBJETO_PTR ListaObjetos){
    while(ListaObjetos != NULL){
        if (ListaObjetos->Alarmado)
            return true;
        ListaObjetos = ListaObjetos->Prox;
    }
    return false;
}

void MostraObjetos(OBJETO_PTR ListaObjetos, Mat *Cor, int R_Inicio, int R_Fim){
    if (ListaObjetos == NULL)
        return;
    
    Vec3b Verde, Vermelho, Preto;
    Verde[0] =  0;      Vermelho[0] = 0;      Preto[0] = 0;
    Verde[1] =  255;  Vermelho[1] = 0;      Preto[1] = 0;
    Verde[2] =  0;      Vermelho[2] = 255;  Preto[2] = 0;
    
    OBJETO_PTR atual = ListaObjetos;
    int i, j;
    while (atual != NULL){
        for (j = R_Inicio; j < R_Fim; j++){
            for (i = 0; i < Cor->cols; i++){
                //printf("j: %d, i: %d\n", j, i);
                if (Cor->at<Vec3b>(j,i) == Verde || Cor->at<Vec3b>(j,i) == Vermelho)
                    continue;
                
                if (atual->img.at<uchar>(j,i) == 255)
                    Cor->at<Vec3b>(j,i) = Verde;
                //else
                   // Cor->at<Vec3b>(j,i) = Preto;
            }
        }
        atual = atual->Prox;
    }
}

void contPixelsMask(OBJETO_PTR ListaObjetos, Mat mask, int min, int max){
    int i, j;
    while(ListaObjetos != NULL){
        ListaObjetos->cont_semelhanca = 0;
       for( j = min; j < max; j++){
           for (i = 0; i < mask.rows; i++){
               if (mask.at<uchar>(j,i) == 255 && ListaObjetos->img.at<uchar>(j,i) == 255)
                   ListaObjetos->cont_semelhanca++;
           }
       }
        ListaObjetos = ListaObjetos->Prox;
    }
}


void* Thread(void * arg){
    long int *args = ((long int*)arg);
    
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
   
    //Inicia Serial
    string AlteraAlarme = "1";
    string Porta("/dev/ttyACM0");
    unsigned long baud = 9600;
    serial::Serial my_serial(Porta, baud, serial::Timeout::simpleTimeout(1000));
    
    int i, j;
    int pix;
    Mat prev, atual, diff, mask; //Mat necessarias
    Mat antigo4k, cor; //Mat gambiarras
    VideoCapture Webcam("tre_bike.mp4");
    //VideoCapture Webcam(0);
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    
    Webcam.read(prev);
    if(prev.empty()){
        printf("prev vazio\n");
        return NULL;
    }

    cvtColor( prev, prev, CV_BGR2GRAY );
   
    //Cores
    Vec3b Verde, Vermelho, Preto;
    Verde[0] =  0;      Vermelho[0] = 0;      Preto[0] = 0;
    Verde[1] =  255;  Vermelho[1] = 0;      Preto[1] = 0;
    Verde[2] =  0;      Vermelho[2] = 255;  Preto[2] = 0;
    
    const int R_Inicio = prev.rows/2 - prev.rows*TAM/200;
    const int R_Fim    = prev.rows/2 + prev.rows*TAM/200;
    int N_Pixels_Total = (R_Fim-R_Inicio)*prev.cols;
    
    Webcam.read(atual);
    cvtColor( atual, atual, CV_BGR2GRAY );

    absdiff(prev, atual, diff);
    Mat temp(atual.size(),CV_8UC1, Scalar(0));
    
    int val;
    for(j=R_Inicio; j<R_Fim; j++) {
        for(i=1; i<diff.cols; i++){
            val  = (int)diff.at<uchar>(Point(j,i));
            if(val>= TH_MASK){
               temp.at<uchar>(j,i) = 255;
            } 
        }
    }
    
   antigo4k = temp;
   
   
   int cont_standby = 0;
   bool Alarmado = false;
   int cont_simula_rfid = 0;
   int cont_pixels_mask = 0;
   int cont_chamada_alarme = 0;
   int cont4k = 0, cont4kPIxels = 0; 
   int cont_frames = 0, cont_pixels = 0;
   
   OBJETO_PTR ListaObjetos = NULL;
   OBJETO_PTR Temp = NULL;
   
   while (true && !false){

        //printf("inicio\n");
       Webcam.read(atual);
        cor = atual;
        
        if(atual.empty()){
            printf("Sucesso, fim de video\n");
            return NULL;
        }
        //printf("1\n");
        //printf("11\n");
        cont_pixels = 0;
        cont_pixels_mask = 0;
    
        cvtColor( atual, atual, CV_BGR2GRAY );
            
        //printf("12\n");
        diff.setTo(Scalar(0));
        absdiff(prev, atual, diff);
        mask = Mat(atual.size(),CV_8UC1, Scalar(0));
        
        //printf("13\n");
        MostraObjetos(ListaObjetos, &cor, R_Inicio, R_Fim);
        
        if(cont_standby){
            cont_standby--;
            imshow( "Display window", cor);
            waitKey(1);
            continue;
        }
            
        
        //printf("5\n");
        //printf("14\n");
        for(int j=R_Inicio; j<R_Fim; j++) {
            for(int i=0; i<diff.cols; i++){
                
                if((int)diff.at<uchar>(j,i) >= TH_MASK){
                    mask.at<uchar>(j,i) = 255;
                    cor.at<Vec3b>(j,i) = Vermelho;
                    cont_pixels_mask++;
                } 
                
                if (mask.at<uchar>(j,i) == temp.at<uchar>(j,i))
                    cont_pixels++;
                
                if(cont4k == TAM_4K &&  (mask.at<uchar>(j,i) == antigo4k.at<uchar>(j,i))){
                    cont4kPIxels++;
                }
            }
        }
        contPixelsMask(ListaObjetos, mask, R_Inicio, R_Fim);
        
        //printf("15\n");
        
        //printf("10\n");
        if(cont4k == TAM_4K ){
            if((cont4kPIxels*100)/N_Pixels_Total > TH_IMG_IGUAL){
                prev = atual;
                cont_frames = 0;
            }
            cont4k = 0;
            antigo4k = mask;
            cont4kPIxels = 0;
        }
        //printf("17\n");
        
        if((cont_pixels*100)/N_Pixels_Total < TH_IMG_IGUAL){
            cont_frames = 0;
            temp.setTo(Scalar(0));
        }else 
            cont_frames++;      
        
        if (cont_frames == TH_NOVO_PREV){
            prev = atual;
            cont_frames = 0;
        }
        
        //printf("18\n");
        
        //printf("20\n");
        Temp = ListaObjetos;
        while(Temp != NULL){
            if (Temp->cont_chamada_alarme > TH_CHAMA_ALARME && Temp->Alarmado == 0){
                if (!Alarmado){
                    my_serial.write(AlteraAlarme);
                    Alarmado = 1;
                }
                Temp->Alarmado = 1;
            }
            Temp = Temp->Prox;
        }
        
        //printf("22\n");
        //printf("1\n");
        int RFID = args[0];
        if(RFID!= -1 || cont_simula_rfid == TH_SIMULA_RFID){
            //printf("23\n");
            //printf("2\n");
            if (cont_simula_rfid == TH_SIMULA_RFID){
                RFID = 123;
                cont_simula_rfid = 0;
                //printf("24\n");
            }
            
            //printf("25\n");
            if(Alarmado){
                //printf("26\n");
                
                if(EstaAlarmado(ListaObjetos, RFID)){
        //printf("26.2\n");
                    RemoveObjeto(&ListaObjetos, RFID);
        //printf("27\n");
                    if(!TemAlarmado(ListaObjetos)){
                        Alarmado = false;
                        my_serial.write(AlteraAlarme);
                    }
                } 
            } 
            else {
                if(JaExiste(ListaObjetos, RFID)){
                    RemoveObjeto(&ListaObjetos, RFID);
                    prev = atual;
                    cont_standby = STANDBY;
                }
                else
                {
                    cout << "Novo Cartao: ";
                    cout << RFID;
                    cout << "\n";
                    
                    AddObejto(&ListaObjetos, cont_pixels_mask, RFID, mask);
                    prev = atual;
                    cont4k = 0;
                    cont_frames = 0;
                    cont4kPIxels = 0;
                    cont_chamada_alarme = 0;
                    
                    args[0] = -1;
                }
            }
        //printf("30\n");
        } 
        else if (ListaObjetos != NULL){
       // printf("31\n");
            Temp = ListaObjetos;
            while(Temp != NULL){
                if((Temp->cont_semelhanca*100/Temp->Pixels) > TH_CHAMA_ALARME)
                    Temp->cont_chamada_alarme++;
                else
                    Temp->cont_chamada_alarme = 0;
                
                Temp = Temp->Prox;
        //printf("32\n");
            }
        //printf("33\n");
        }
        
        //printf("final\n");
        cont4k++;
        cont_simula_rfid++;
        
        imshow( "Display window", cor);
        waitKey(1);
        //printf("10\n");
   }
}

int main(){
    pthread_t tid;
    
    long int a[2] = {-1,2};
    long int (*p)[2] = &a;
    
    if(pthread_create(&tid,NULL,Thread,p) != 0){
        printf("pthread create failed\n");
        return 0;
    }
    long int Id;
    while(a[1] != -1){
        cin >> Id;
        a[0] = Id;
        setbuf(stdin, NULL);
    }
    return 0;
}
