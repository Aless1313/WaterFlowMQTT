#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>


const char* ssid = "BisonbotsFCFM";
const char* password = "RaspberryFCFM2023";

#define SENSORAGUA 14
char msg3[50];

/*Variables para el sensor de flujo de agua*/
int interval = 1000;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1sec =0;
float flowRate;
unsigned long flowMililliLitres;
unsigned long totalMilliLitres;
float flowLitres;
float totalLitres;
int costo = 0;


/*Variables para tiempos*/
long currentMillis = 0;
long previousMillis = 0;
long previousMillis2 = 0;

/*Funcion para leer pulsos del sensor de flujo de agua*/
void IRAM_ATTR pulseCounter(){
  pulseCount++;
}

char msg[25];

// Configura el servidor MQTT público (ejemplo: broker.hivemq.com)
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_user ="";
const char* mqtt_pass = "";

WiFiClient espClient;
PubSubClient client(espClient);
void callback(char* topic, byte* payload, int length);
void reconnect();

void setup() {
  Serial.begin(9600);
  delay(10);

  // Conexión a la red Wi-Fi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }

  Serial.println("Conexión WiFi establecida");

   //Clientes de mqtt
  client.setServer(mqtt_server,mqtt_port);
  client.setCallback(callback);

   attachInterrupt(digitalPinToInterrupt(SENSORAGUA), pulseCounter, FALLING);
}

void loop() {
  if(!client.connected()){
    reconnect();
  }
  client.loop();

   //Variable de contadora de tiempo
  currentMillis = millis();

    /*Definir tiempo de intervalo entre actualizaciones mqtt*/
  if(currentMillis - previousMillis > interval){
    pulse1sec = pulseCount;
    pulseCount = 0;

    /*Conteo de pulsos en un segundo en el sensor de flujo*/
    flowRate = ((1000.0 /(millis() - previousMillis)) * pulse1sec) / calibrationFactor;
    previousMillis = millis();

    /*Conversión de flujo a litros*/
    flowLitres = (flowRate/60);
    totalLitres += flowLitres;

    Serial.print("Flujo de agua: ");
    Serial.print(float(flowRate));
    Serial.print("L/Min");
    Serial.print("\t");

    Serial.print("Litros consumidos: ");
    Serial.print(totalLitres);
    Serial.println("L");

    String flujo = String(flowRate, 3); 
    String to_Send = flujo;
    to_Send.toCharArray(msg, 25);
    client.publish("values", msg);
    Serial.println("Enviado a topico");   
    
  }
  
}

void callback(char* topic, byte* payload, int length ){
  String incoming = "";
  Serial.print("Mensaje recibido de ->");
  Serial.print(topic);
  Serial.println("");

  for(int i=0; i<length; i++){
    incoming += (char)payload[i];
  }

  incoming.trim(); 
  Serial.println("Mensaje -> " + incoming);

  if(incoming == "on"){
    Serial.println("ON");
    }
  
  if(incoming == "off"){
    Serial.println("OFF");
  }
}

void reconnect(){
  while(!client.connected()){
    Serial.print("Conectando a MQTT...");
    String clientId = "ESP8266";
    clientId += String(random(0xffff), HEX);

    if(client.connect(clientId.c_str(), mqtt_user, mqtt_pass)){
      Serial.println("Conectado a mqtt");
      client.subscribe("datasensor");
      Serial.println("Subscrito a topico");
    }else{
      Serial.println("Fallo ->" + client.state());
      delay(2500);
    }
  }
}