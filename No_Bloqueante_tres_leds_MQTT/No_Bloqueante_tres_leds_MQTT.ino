//*****************************************************************************************
//                                  No_Bloqueante_tres_leds_MQTT
//
//    Autor: Jorge Miguel Jaimes Ponce
//    Diplomado: Internet de las cosas
//
//    Este programa es una tarea, la cual consiste en utilizar la funcuón millis() para 
// realizar pausas en bloques de programa, sin detener la ejecución del programa en
// general.
//    Se tienen tres leds, en los pines 12, 13 y 14, los cuales se encenderán y apagarán
// con frecuencias distintas. Y uno a la vez varias veces:
//          Led1 : Encender y apagar 4 veces con una frecuencia de 5 segundos:
//          Led2 : Encender y apagar 8 veces con una frecuencia de 3 segundos
//          Led3 : Encender y apagar 10 veces con una frecuencia de 1 segundo 
//
//    Además enviara a nodered utilizando MQTT y un beoker, la cantidad de veces que
// se enciende la secuencia de led en curso.
//
//                                       14/10/2021
//
//*****************************************************************************************

#define CAMERA_MODEL_AI_THINKER
#define Led12 12
#define Led13 13
#define Led14 14

//Inicializar los tiempos de los leds Led12, Led13 y Led14
unsigned long t0_12 = 0;
unsigned long t0_13 = 0;
unsigned long t0_14 = 0;

//  Variables correspondientes a lo tiempos siguientes
//  de los leds Led12, Led13 y Led14
unsigned long t1_12 = 0;
unsigned long t1_13 = 0;
unsigned long t1_14 = 0;

bool Led1 = false;
bool Led2 = false;
bool Led3 = false;

byte loop_Led1 = 0;
byte loop_Led2 = 0 ;
byte loop_Led3 = 0 ;
byte veces = 1;


void setup() {
  //Configurando los LedsXX como salidas
  pinMode(Led12,OUTPUT);
  pinMode(Led13,OUTPUT);
  pinMode(Led14,OUTPUT);
  
  //Inicializando estado lógico de las salidas
  digitalWrite(Led12,Led1);
  digitalWrite(Led13,Led2);
  digitalWrite(Led14,Led3);

  //Inicializando t0 paa cada led
  t0_12 = millis();
  t0_13 = t0_12;
  t0_14 = t0_12;

   Serial.begin(9600);
  
}


void loop() {
  if(veces >=1  and veces <=8){
    if(veces == 1){
      Serial.println("4 veces");
    }
        blink_Led12(2500);    //Blink de Led12 con intervalo de 5 seg de encndido y 5 seg apagado
  }

  if(veces >=9  and veces <= 24){
    if(veces == 9){
      Serial.println("8 veces");
    }
        blink_Led13(1500);    //Blink de Led13 con intervalo de 3 seg de encndido y 3 seg apagado
  }

  if(veces >= 25 and veces <= 44){
    if(veces == 25){
      Serial.println("10 veces");
    }
        blink_Led14(500);     //Blink de Led14 con intervalo de 0.5 seg de encndido y 0.5 seg apagado
  }
}



//****************************************************
//  Funciones de blink de leds Led12, Led13 y Led14
//****************************************************

void blink_Led12(int tiempo){

   t1_12 = millis();                //Cargando tiempo actual
    //Determinando el tiempó transcurrido
     if (t1_12 - t0_12 >= tiempo){              //Ya son 5 segundos...?
        t0_12 = millis();
        Led1 = !Led1;
        digitalWrite(Led12,Led1);  
         
        veces = veces + 1;   
     }
}


void blink_Led13(int tiempo){
     t1_13 = millis();                //Cargando tiempo actual
    //Determinando el tiempó transcurrido
     if (t1_13 = t1_13 - t0_13 >= tiempo){              //Ya son 3 segundos...?
        t0_13 = millis();
        Led2 = !Led2;
        digitalWrite(Led13,Led2);
        
        veces = veces + 1;   
     }
}

void blink_Led14(int tiempo){
  t1_14 = millis();                //Cargando tiempo actual
     //Determinando el tiempó transcurrido
     if (t1_14 - t0_14 >= tiempo){               //Ya son 500 mili segundos...?  
        t0_14 = millis();  
        Led3 = !Led3;
        digitalWrite(Led14,Led3);
        
       veces = veces + 1;   
        if(veces >=45){
          veces = 1;
          //digitalWrite(Led14,LOW);
        }
     
     }
}
