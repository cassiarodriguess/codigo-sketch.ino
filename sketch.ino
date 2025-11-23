/* ESP32 DS18B20 + LED + Buzzer + MQTT
   Medições de tempo: registra timestamps em millis()
   Publica em vacinas/box01/temp
   Subscrição em vacinas/box01/cmd para acionamento remoto
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ===== CONFIG =====
const char* WIFI_SSID = "Wokwi-GUEST"; // Assumindo que você está usando o Wokwi para o teste
const char* WIFI_PASS = ""; 
// Para ambiente real, use a sua rede 2.4GHz: const char* WIFI_SSID = "SUA_REDE_2.4G";
const char* MQTT_SERVER = "broker.hivemq.com"; // Trocando para o HiveMQ, mais estável
const uint16_t MQTT_PORT = 1883;
const char* TOPIC_TEMP = "vacinas/box01/temp";
const char* TOPIC_ALARME = "vacinas/box01/alarme";
const char* TOPIC_CMD = "vacinas/box01/cmd";
const char* CLIENT_ID = "esp32_box01";

// ===== HARDWARE PINS =====
#define ONE_WIRE_PIN 13  // GPIO 13 (Fio Verde do Sensor)
#define LED_PIN 17
#define BUZZER_PIN 16

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);
WiFiClient espClient;
PubSubClient client(espClient);

// Variável para rastrear se o sensor foi encontrado
bool sensor_detected = false;

// Variável para armazenar o endereço do sensor
DeviceAddress tempSensorAddress; 

// Histerese / limites
float TEMP_LOW = 2.0;
float TEMP_HIGH = 8.0;
float HISTERESE = 0.5;

unsigned long lastSample = 0;
const unsigned long SAMPLE_INTERVAL = 30000; // 30s (Conforme Artigo)

// Tempo de conversão aumentado para garantir leitura (1000ms)
const unsigned long CONVERSION_DELAY = 1000; 

// --- util para medir tempo
unsigned long t_sensor_read = 0;
unsigned long t_publish = 0;
unsigned long t_mqtt_received_cmd = 0;
unsigned long t_action = 0;

// ****************************************************
// FUNÇÃO: Realiza a amostragem, leitura e publicação
// ****************************************************
void performSamplingAndPublish() {
  if (sensor_detected) {
    // 1) Solicitar conversão de temperatura
    // Não marcamos tempo aqui pois o request é rápido, o delay é que demora
    sensors.requestTemperatures();
    
    // ** AGUARDAR CONVERSÃO (Bloqueante, mas necessário para DS18B20 simples) **
    delay(CONVERSION_DELAY); 
    
    // 2) Ler temperatura
    t_sensor_read = millis();
    float tempC = sensors.getTempC(tempSensorAddress); 
    unsigned long t_after_read = millis();
    
    // ****** VERIFICAÇÃO CRÍTICA DE LEITURA NO LOOP ******
    // Dispositivos desconectados retornam -127. Erros podem vir como 85.0 (power-on reset)
    if (tempC == DEVICE_DISCONNECTED_C || tempC == 85.0) { 
      Serial.print("ALERTA: Leitura instável ou inválida (");
      Serial.print(tempC);
      Serial.println("). Tentando novamente na próxima amostragem.");
      // Não publica dados inválidos, apenas ignora este ciclo
    } else {
      // Se tempC for 0.00 exatamente, pode ser real (0 graus) ou erro.
      // Vamos assumir como válido se não for -127 ou 85.
      
      Serial.print("[MEASURE] sensor->read (ms): ");
      Serial.print(tempC);
      Serial.print(" C / Time to read (getTempC): ");
      Serial.println(t_after_read - t_sensor_read);
      
      // 3) publicar via MQTT e marcar tempo
      char payload[128];
      snprintf(payload, sizeof(payload), "{\"tC\": %.2f, \"ts\": \"%lu\"}", tempC, (unsigned long)time(NULL)); 
      t_publish = millis();
      bool ok = client.publish(TOPIC_TEMP, payload, true);
      unsigned long t_after_pub = millis();
      
      // Log para latência (pub)
      Serial.print("[LATENCY] publish (ms): ");
      Serial.println(t_after_pub - t_publish);
      Serial.print("Published: ");
      Serial.println(payload);

      // 4) Lógica de alarme local
      // Faixa segura: 2.0 a 8.0. Com histerese 0.5:
      // ALARME SE: T < 1.5 OU T > 8.5
      if (tempC < (TEMP_LOW - HISTERESE) || tempC > (TEMP_HIGH + HISTERESE)) {
        digitalWrite(LED_PIN, HIGH);
        // O Buzzer ativo precisa de oscilação se for passivo, mas assumindo ativo:
        digitalWrite(BUZZER_PIN, HIGH); 
        Serial.println("ALARM: Temperatura FORA da faixa! (ON)");
        client.publish(TOPIC_ALARME, "ON");
      } else {
        digitalWrite(LED_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("Status: Temperatura OK (OFF)");
        client.publish(TOPIC_ALARME, "OFF");
      }
    } // Fim da verificação de leitura
  } else {
    Serial.println("Aguardando sensor... (Não detectado no setup)");
    // Tenta reinicializar o sensor se ele foi perdido
    sensors.begin();
    if (sensors.getDeviceCount() > 0) {
        sensor_detected = true;
        sensors.getAddress(tempSensorAddress, 0);
        sensors.setResolution(tempSensorAddress, 12);
        Serial.println("Sensor recuperado!");
    }
  } 
}


void callback(char* topic, byte* payload, unsigned int length) {
  // Quando receber comando via MQTT
  unsigned long t_receive = millis();
  Serial.print("[MQTT RX] ");
  Serial.print(topic);
  Serial.print(" - ");
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.println(msg);

  // medir tempo entre recebimento e ação
  t_mqtt_received_cmd = t_receive;
  if (String(topic) == TOPIC_CMD) {
    if (msg == "ALARM_ON") {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
      t_action = millis();
      Serial.print("[LATENCY] cmd->action (ms): ");
      Serial.println(t_action - t_mqtt_received_cmd);
      // opcional: publicar estado
      client.publish(TOPIC_ALARME, "ON");
    } else if (msg == "ALARM_OFF") {
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      client.publish(TOPIC_ALARME, "OFF");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(CLIENT_ID, NULL, NULL, "vacinas/box01/status", 1, true, "offline")) {
      Serial.println("connected");
      client.publish("vacinas/box01/status", "online", true);
      client.subscribe(TOPIC_CMD);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2s");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // ****** Tenta conectar ao sensor em um loop para garantir estabilidade ******
  int retry_count = 0;
  while (sensors.getDeviceCount() == 0 && retry_count < 5) {
      Serial.print("Tentativa de buscar DS18B20 #");
      Serial.print(retry_count + 1);
      Serial.print("... ");
      
      sensors.begin();
      delay(500); // Aguarda a estabilização do bus antes de escanear

      if (sensors.getDeviceCount() > 0) {
          Serial.print(sensors.getDeviceCount());
          Serial.println(" sensor(es) encontrado(s).");
          sensor_detected = true;
          
          if (sensors.getAddress(tempSensorAddress, 0)) {
              Serial.println("Endereço do sensor primário capturado.");
              sensors.setResolution(tempSensorAddress, 12); 
              Serial.println("Resolução do sensor fixada para 12 bits.");
          } else {
              Serial.println("ERRO: Endereço do sensor primário não encontrado.");
              sensor_detected = false; // Falhou na captura do endereço
          }
          break; // Sai do loop se o sensor foi encontrado
      } else {
          Serial.println("Nenhum sensor encontrado.");
          retry_count++;
          delay(500);
      }
  }

  if (sensor_detected == false) {
      Serial.println("ERRO CRÍTICO: DS18B20 não detectado após 5 tentativas.");
  }
  // **************************************************************************


  // Conectar WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  int wifiTrials = 0;
  Serial.println(WiFi.status());
  while (WiFi.status() != WL_CONNECTED && wifiTrials < 40) {
    delay(250);
    Serial.print(".");
    wifiTrials++;
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  reconnect();
  
  // ****** FORÇA A PRIMEIRA LEITURA IMEDIATA APÓS CONECTAR ******
  Serial.println("Forçando primeira amostragem (0ms)...");
  lastSample = millis() - SAMPLE_INTERVAL; // Seta lastSample para forçar o if(now - lastSample >= SAMPLE_INTERVAL) a ser TRUE
  // ***********************************************************
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();
  if (now - lastSample >= SAMPLE_INTERVAL) {
    lastSample = now;
    
    performSamplingAndPublish(); // Chama a rotina de amostragem
  }
}
