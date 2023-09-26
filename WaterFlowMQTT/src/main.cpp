#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "BisonbotsFCFM";
const char* password = "RaspberryFCFM2023";

/*Configuración de internet*/
WiFiClient espClient;
PubSubClient client(espClient);
ESP8266WebServer server(80);

/*Setup de mqtt*/
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_user ="admin";
const char* mqtt_pass = "public";
HTTPClient http;

/*Mensajeria entrante de mqtt*/
long lastMsg = 0;
char msg[25];
char msg2[25];
char msg3[50];

/*Sensor de flujo de agua*/
#define SENSORAGUA 14

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

void callback(char* topic, byte* payload, int length);
void reconnect();

/*Funcion para leer pulsos del sensor de flujo de agua*/
void IRAM_ATTR pulseCounter(){
  pulseCount++;
}

void setup() {
   // Conéctate a la red Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Conectando a la red WiFi...");
  }

  Serial.println("Conexión exitosa a la red WiFi");
  
  
  //Iniciando valores de sensor en 0 para su primer uso
  pulseCount = 0;
  flowRate = 0;
  flowMililliLitres = 0;
  totalLitres = 0;

  //Clientes de mqtt
  client.setServer(mqtt_server,mqtt_port);
  client.setCallback(callback);

    //Definición de sensor de flujo y led
  attachInterrupt(digitalPinToInterrupt(SENSORAGUA), pulseCounter, FALLING);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
    /*Conectando a cliente mqtt*/
  if(!client.connected()){
    reconnect();
  }
  client.loop();

  StaticJsonDocument<256> doc;

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

    JsonArray data = doc.createNestedArray("data");
    data.add(flowRate);                               //Flujo
    data.add(totalLitres);                            //Litros
    data.add(0); 
    
    char out[256];
    int b = serializeJson(doc, out);
    Serial.print("bytes = ");
    Serial.println(out);
    Serial.print(b);                                     //Estado de valvula
    
    
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

  /*
  if(incoming == "on"){
    digitalWrite(ledconf, HIGH);
  }
  
  if(incoming == "off"){
      digitalWrite(ledconf, LOW);
  }*/
}

void reconnect(){
  while(!client.connected()){
    Serial.print("Conectando a MQTT...");
    String clientId = "ESP8266";
    clientId += String(random(0xffff), HEX);

    if(client.connect(clientId.c_str(), mqtt_user, mqtt_pass)){
      Serial.println("Conectado a mqtt");
      client.subscribe("servo");
      Serial.println("Subscrito a topico");
    }else{
      Serial.println("Fallo ->" + client.state());
      delay(2500);
    }
  }
}

