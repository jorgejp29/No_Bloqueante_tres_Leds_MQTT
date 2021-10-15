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

#define CAMERA_MODEL_AI_THINKER // Has PSRAM


/*
 * Conexión básica por MQTT del NodeMCU
 * por: Hugo Escalpelo
 * Fecha: 28 de julio de 2021
 * 
 * Este programa envía datos  por Internet a través del protocolo MQTT. Para poder
 * comprobar el funcionamiento de este programa, es necesario conectarse a un broker
 * y usar NodeRed para visualzar que la información se está recibiendo correctamente.
 * Este programa no requiere componentes adicionales.
 * 
 * Componente     PinESP32CAM     Estados lógicos
 * ledStatus------GPIO 33---------On=>LOW, Off=>HIGH
 * ledFlash-------GPIO 4----------On=>HIGH, Off=>LOW
 */

//Bibliotecas
#include <WiFi.h>  // Biblioteca para el control de WiFi
#include <PubSubClient.h> //Biblioteca para conexion MQTT

//Datos de WiFi
const char* ssid = "*********";  // Aquí debes poner el nombre de tu red
const char* password = "************";  // Aquí debes poner la contraseña de tu red

//Datos del broker MQTT
//const char* mqtt_server = "3.122.36.163"; // Si estas en una red local, coloca la IP asignada, en caso contrario, coloca la IP publica
const char* mqtt_server = "3.122.36.163"; 
IPAddress server(3,122,36,163);

// Objetos
WiFiClient espClient; // Este objeto maneja los datos de conexion WiFi
PubSubClient client(espClient); // Este objeto maneja los datos de conexion al broker

// Variables
int flashLedPin = 4;  // Para indicar el estatus de conexión
int statusLedPin = 33; // Para ser controlado por MQTT
long timeNow, timeLast; // Variables de control de tiempo no bloqueante
int data = 0; // Contador
int wait = 5000;  // Indica la espera cada 5 segundos para envío de mensajes MQTT

//***************************************************************************
//   Variables utilizadas en la secuenca y encendido apagado de e leds

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

char dataString[8];

//***************************************************************************




// Inicialización del programa
void setup() {
  // Iniciar comunicación serial
  Serial.begin (115200);
  pinMode (flashLedPin, OUTPUT);
  pinMode (statusLedPin, OUTPUT);
  digitalWrite (flashLedPin, LOW);
  digitalWrite (statusLedPin, HIGH);

  Serial.println();
  Serial.println();
  Serial.print("Conectar a ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password); // Esta es la función que realiz la conexión a WiFi
 
  while (WiFi.status() != WL_CONNECTED) { // Este bucle espera a que se realice la conexión
    digitalWrite (statusLedPin, HIGH);
    delay(500); //dado que es de suma importancia esperar a la conexión, debe usarse espera bloqueante
    digitalWrite (statusLedPin, LOW);
    Serial.print(".");  // Indicador de progreso
    delay (5);
  }
  
  // Cuando se haya logrado la conexión, el programa avanzará, por lo tanto, puede informarse lo siguiente
  Serial.println();
  Serial.println("WiFi conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());

  // Si se logro la conexión, encender led
  if (WiFi.status () > 0){
  digitalWrite (statusLedPin, LOW);
  }
  
  delay (1000); // Esta espera es solo una formalidad antes de iniciar la comunicación con el broker

  //Serial.end();

  // Conexión con el broker MQTT
  client.setServer(server, 1883); // Conectarse a la IP del broker en el puerto indicado
  client.setCallback(callback); // Activar función de CallBack, permite recibir mensajes MQTT y ejecutar funciones a partir de ellos
  //delay(1500);  // Esta espera es preventiva, espera a la conexión para no perder información
  delay(1500);  // Esta espera es preventiva, espera a la conexión para no perder información

  timeLast = millis (); // Inicia el control de tiempo


//**************************************************************************************
//   Inicialización de variables utilizadas en la secuenca y encendido apagado de e leds

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

//**************************************************************************************
  
}// fin del void setup ()


//*****************************************
// Cuerpo del programa, bucle principal
//*****************************************

void loop() {

  //distancia = distanceSensor.measureDistanceCm();

  
  //Verificar siempre que haya conexión al broker
  if (!client.connected()) {
    reconnect();  // En caso de que no haya conexión, ejecutar la función de reconexión, definida despues del void setup ()
  }// fin del if (!client.connected())
  client.loop(); // Esta función es muy importante, ejecuta de manera no bloqueante las funciones necesarias para la comunicación con el broker
  
  timeNow = millis(); // Control de tiempo para esperas no bloqueantes
  if (timeNow - timeLast > wait) { // Manda un mensaje por MQTT cada cinco segundos
    timeLast = timeNow; // Actualización de seguimiento de tiempo

        //***********************************************************************************************************************************
        //                      Bloque de programa para el blink de leds, y repetición de n veces
        //***********************************************************************************************************************************

        char dataString[8]; // Define una arreglo de caracteres para enviarlos por MQTT, especifica la longitud del mensaje en 8 caracteres
      
        if(veces >=1  and veces <=8){
          if(veces == 1){
            //Serial.println("4 veces");
            data = 4;
         
          }
              blink_Led12(2500);    //Blink de Led12 con intervalo de 5 seg de encndido y 5 seg apagado
        }
      
        if(veces >=9  and veces <= 24){
          if(veces == 9){
            //Serial.println("8 veces");
            data = 8;
          
          }
              blink_Led13(1500);    //Blink de Led13 con intervalo de 3 seg de encndido y 3 seg apagado
        }
      
        if(veces >= 25 and veces <= 44){
          if(veces == 25){
            //Serial.println("10 veces");
            data = 10;
           
          }
              blink_Led14(500);     //Blink de Led14 con intervalo de 0.5 seg de encndido y 0.5 seg apagado
        }
        
      //*************************************************************************************************************************************
      //                      FIN de Bloque de programa para el blink de leds, y repetición de n veces
      //*************************************************************************************************************************************

  }// fin del if (timeNow - timeLast > wait)
}// fin del void loop ()

//**********************************************************************************************************************************************
//                                        FIN de Cuerpo del programa, bucle principal
//**********************************************************************************************************************************************


//*************************
// Funciones de usuario
//*************************

// Esta función permite tomar acciones en caso de que se reciba un mensaje correspondiente a un tema al cual se hará una suscripción
void callback(char* topic, byte* message, unsigned int length) {

  // Indicar por serial que llegó un mensaje
  Serial.print("Llegó un mensaje en el tema: ");
  Serial.print(topic);

  // Concatenar los mensajes recibidos para conformarlos como una varialbe String
  String messageTemp; // Se declara la variable en la cual se generará el mensaje completo  
  for (int i = 0; i < length; i++) {  // Se imprime y concatena el mensaje
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  // Se comprueba que el mensaje se haya concatenado correctamente
  Serial.println();
  Serial.print ("Mensaje concatenado en una sola variable: ");
  Serial.println (messageTemp);

  // En esta parte puedes agregar las funciones que requieras para actuar segun lo necesites al recibir un mensaje MQTT

  // Ejemplo, en caso de recibir el mensaje true - false, se cambiará el estado del led soldado en la placa.
  // El ESP323CAM está suscrito al tema esp/output
  if (String(topic) == "codigoiot/foco/jjaimes") {  // En caso de recibirse mensaje en el tema esp32/output
    if(messageTemp == "true"){
      Serial.println("Led encendido");
      digitalWrite(flashLedPin, HIGH);
    }// fin del if (String(topic) == "codigoiot/foco/jjaimes")
    else if(messageTemp == "false"){
      Serial.println("Led apagado");
      digitalWrite(flashLedPin, LOW);
    }// fin del else if(messageTemp == "false")
  }// fin del if (String(topic) == "codigoiot/foco/jjaimes")
}// fin del void callback

// Función para reconectarse
void reconnect() {
  // Bucle hasta lograr conexión
  while (!client.connected()) { // Pregunta si hay conexión
    Serial.print("Tratando de contectarse...");
    // Intentar reconexión
    if (client.connect("ESP32CAMClient")) { //Pregunta por el resultado del intento de conexión
      Serial.println("Conectado");
      client.subscribe("codigoiot/foco/jjaimes"); // Esta función realiza la suscripción al tema
    }// fin del  if (client.connect("ESP32CAMClient"))
    else {  //en caso de que la conexión no se logre
      Serial.print("Conexion fallida, Error rc=");
      Serial.print(client.state()); // Muestra el codigo de error
      Serial.println(" Volviendo a intentar en 5 segundos");
      // Espera de 5 segundos bloqueante
      delay(50);
      Serial.println (client.connected ()); // Muestra estatus de conexión
    }// fin del else
  }// fin del bucle while (!client.connected())
}// fin de void reconnect()



//****************************************************
//  Funciones de blink de leds Led12, Led13 y Led14
//****************************************************

void blink_Led12(int tiempo){

   t1_12 = millis();                //Cargando tiempo actual
    //Determinando el tiempó transcurrido
     if (t1_12 - t0_12 >= tiempo){              //Ya son 5 segundos...?
        t0_12 = millis();
        data = 4;
        dtostrf(data, 1, 2, dataString);
        client.publish("codigoiot/secuencia/jjaimes", dataString );
        Led1 = toggleLed(Led12,Led1);
 
        veces = veces + 1;   
     }
}


void blink_Led13(int tiempo){
     t1_13 = millis();                //Cargando tiempo actual
    //Determinando el tiempó transcurrido
     if (t1_13 = t1_13 - t0_13 >= tiempo){              //Ya son 3 segundos...?
        t0_13 = millis();
        data = 8;
        dtostrf(data, 1, 2, dataString);
        client.publish("codigoiot/secuencia/jjaimes", dataString );
        Led2 = toggleLed(Led13,Led2);
              
        veces = veces + 1;   
     }
}

void blink_Led14(int tiempo){
  t1_14 = millis();                //Cargando tiempo actual
     //Determinando el tiempó transcurrido
     if (t1_14 - t0_14 >= tiempo){               //Ya son 500 mili segundos...?  
        t0_14 = millis();  
        data = 10;
        dtostrf(data, 1, 2, dataString);
        client.publish("codigoiot/secuencia/jjaimes", dataString );
        Led3 = toggleLed(Led14,Led3);
      
       veces = veces + 1;   
       
        if(veces >=45){
          veces = 1;
          
        }   
     }
}

//**********************************************************
//  FIN de Funciones de blink de leds Led12, Led13 y Led14
//**********************************************************

bool toggleLed(int pinLed, bool estado) {
  digitalWrite(pinLed,!estado);
  return(!estado);
}
