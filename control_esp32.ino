// ============================================================
//  CARRO 4WD OMNIDIRECCIONAL (Mecanum)
//  ESP32 Arduino Core 3.x + MQTT + mDNS
//  FL=M1  FR=M2  RL=M3  RR=M4
// ============================================================

#include <WiFi.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>

const char* wifi_ssid     = "S23 Ultra de jesus";
const char* wifi_password = "jsvsol.0";

// --- Cambio mDNS ---
// Ya no se usa una IP fija. En vez de eso, se busca el RPi5 por su
// hostname mDNS. Pon aqui el hostname SIN ".local" y SIN puntos.
// Por default en Raspberry Pi OS suele ser "raspberrypi".
// Verificalo en el RPi5 con: hostname
const char* mqtt_hostname = "keplerberry";
const int   mqtt_port     = 1883;
const char* topic_cmd     = "robot/control/comando";
const char* topic_estado  = "robot/estado";

IPAddress mqtt_ip; // se llena en tiempo de ejecucion via mDNS

#define M1_RPWM 25  // FL delantero izquierdo
#define M1_LPWM 26
#define M2_RPWM 27  // FR delantero derecho
#define M2_LPWM 14
#define M3_RPWM 33  // RL trasero izquierdo
#define M3_LPWM 32
#define M4_RPWM 18  // RR trasero derecho
#define M4_LPWM 19

#define FREQ 5000
#define RES  8
#define VEL  200

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

void motor(int rpwm, int lpwm, int vel, int dir) {
  if (dir == 1)       { ledcWrite(rpwm, vel); ledcWrite(lpwm, 0);   }
  else if (dir == -1) { ledcWrite(rpwm, 0);   ledcWrite(lpwm, vel); }
  else                { ledcWrite(rpwm, 0);   ledcWrite(lpwm, 0);   }
}

void frenar() {
  motor(M1_RPWM, M1_LPWM, 0, 0);
  motor(M2_RPWM, M2_LPWM, 0, 0);
  motor(M3_RPWM, M3_LPWM, 0, 0);
  motor(M4_RPWM, M4_LPWM, 0, 0);
  mqttClient.publish(topic_estado, "robot_parado");
  Serial.println("STOP");
}

void adelante() {
  motor(M1_RPWM, M1_LPWM, VEL,  1);
  motor(M2_RPWM, M2_LPWM, VEL,  1);
  motor(M3_RPWM, M3_LPWM, VEL,  1);
  motor(M4_RPWM, M4_LPWM, VEL,  1);
  mqttClient.publish(topic_estado, "robot_moviendose");
}

void atras() {
  motor(M1_RPWM, M1_LPWM, VEL, -1);
  motor(M2_RPWM, M2_LPWM, VEL, -1);
  motor(M3_RPWM, M3_LPWM, VEL, -1);
  motor(M4_RPWM, M4_LPWM, VEL, -1);
  mqttClient.publish(topic_estado, "robot_moviendose");
}

void lateralDer() {
  motor(M1_RPWM, M1_LPWM, VEL,  1);
  motor(M2_RPWM, M2_LPWM, VEL, -1);
  motor(M3_RPWM, M3_LPWM, VEL, -1);
  motor(M4_RPWM, M4_LPWM, VEL,  1);
  mqttClient.publish(topic_estado, "robot_moviendose");
}

void lateralIzq() {
  motor(M1_RPWM, M1_LPWM, VEL, -1);
  motor(M2_RPWM, M2_LPWM, VEL,  1);
  motor(M3_RPWM, M3_LPWM, VEL,  1);
  motor(M4_RPWM, M4_LPWM, VEL, -1);
  mqttClient.publish(topic_estado, "robot_moviendose");
}

void girarDer() {
  motor(M1_RPWM, M1_LPWM, VEL,  1);
  motor(M2_RPWM, M2_LPWM, VEL, -1);
  motor(M3_RPWM, M3_LPWM, VEL,  1);
  motor(M4_RPWM, M4_LPWM, VEL, -1);
}

void girarIzq() {
  motor(M1_RPWM, M1_LPWM, VEL, -1);
  motor(M2_RPWM, M2_LPWM, VEL,  1);
  motor(M3_RPWM, M3_LPWM, VEL, -1);
  motor(M4_RPWM, M4_LPWM, VEL,  1);
}

void diagFD() {
  motor(M1_RPWM, M1_LPWM, VEL,  1);
  motor(M2_RPWM, M2_LPWM, 0,    0);
  motor(M3_RPWM, M3_LPWM, 0,    0);
  motor(M4_RPWM, M4_LPWM, VEL,  1);
}

void diagFI() {
  motor(M1_RPWM, M1_LPWM, 0,    0);
  motor(M2_RPWM, M2_LPWM, VEL,  1);
  motor(M3_RPWM, M3_LPWM, VEL,  1);
  motor(M4_RPWM, M4_LPWM, 0,    0);
}

void diagAD() {
  motor(M1_RPWM, M1_LPWM, 0,    0);
  motor(M2_RPWM, M2_LPWM, VEL, -1);
  motor(M3_RPWM, M3_LPWM, VEL, -1);
  motor(M4_RPWM, M4_LPWM, 0,    0);
}

void diagAI() {
  motor(M1_RPWM, M1_LPWM, VEL, -1);
  motor(M2_RPWM, M2_LPWM, 0,    0);
  motor(M3_RPWM, M3_LPWM, 0,    0);
  motor(M4_RPWM, M4_LPWM, VEL, -1);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String cmd = "";
  for (int i = 0; i < length; i++) cmd += (char)payload[i];
  Serial.println("CMD: " + cmd);

  if      (cmd == "persona_frente")    adelante();
  else if (cmd == "persona_izquierda") lateralIzq();
  else if (cmd == "persona_derecha")   lateralDer();
  else if (cmd == "persona_cerca")     frenar();
  else if (cmd == "sin_persona")       frenar();
  else if (cmd == "adelante")          adelante();
  else if (cmd == "atras")             atras();
  else if (cmd == "lateral_der")       lateralDer();
  else if (cmd == "lateral_izq")       lateralIzq();
  else if (cmd == "giro_der")          girarDer();
  else if (cmd == "giro_izq")          girarIzq();
  else if (cmd == "diag_fd")           diagFD();
  else if (cmd == "diag_fi")           diagFI();
  else if (cmd == "diag_ad")           diagAD();
  else if (cmd == "diag_ai")           diagAI();
  else if (cmd == "stop")              frenar();
}

// --- Funcion nueva: resuelve el hostname mDNS a una IP ---
// Reintenta varias veces porque justo despues de conectar WiFi
// la red mDNS a veces tarda un poco en responder.
bool resolverMqttHost() {
  Serial.printf("Buscando %s.local por mDNS...\n", mqtt_hostname);
  for (int intento = 1; intento <= 5; intento++) {
    IPAddress ip = MDNS.queryHost(mqtt_hostname);
    if (ip != IPAddress(0, 0, 0, 0)) {
      mqtt_ip = ip;
      Serial.printf("Resuelto %s.local -> %s\n", mqtt_hostname, mqtt_ip.toString().c_str());
      return true;
    }
    Serial.printf("  intento %d fallido, reintentando...\n", intento);
    delay(1000);
  }
  Serial.println("No se pudo resolver el hostname mDNS.");
  return false;
}

void mqttReconnect() {
  if (mqttClient.connected()) return;

  // Si por alguna razon perdimos la IP (ej. el RPi5 se reinicio
  // y cambio de IP), la volvemos a resolver antes de reconectar.
  if (mqtt_ip == IPAddress(0, 0, 0, 0)) {
    if (!resolverMqttHost()) return; // sin IP no intentamos conectar
    mqttClient.setServer(mqtt_ip, mqtt_port);
  }

  Serial.print("Conectando MQTT...");
  if (mqttClient.connect("Robot-ESP32")) {
    Serial.println("OK");
    mqttClient.subscribe(topic_cmd);
    mqttClient.publish(topic_estado, "robot_conectado");
  } else {
    Serial.printf("Error: %d\n", mqttClient.state());
    // Si falla la conexion, invalidamos la IP guardada por si
    // el broker cambio de direccion; se volvera a resolver en el
    // siguiente intento de mqttReconnect().
    mqtt_ip = IPAddress(0, 0, 0, 0);
  }
}

void setup() {
  Serial.begin(115200);

  ledcAttach(M1_RPWM, FREQ, RES);
  ledcAttach(M1_LPWM, FREQ, RES);
  ledcAttach(M2_RPWM, FREQ, RES);
  ledcAttach(M2_LPWM, FREQ, RES);
  ledcAttach(M3_RPWM, FREQ, RES);
  ledcAttach(M3_LPWM, FREQ, RES);
  ledcAttach(M4_RPWM, FREQ, RES);
  ledcAttach(M4_LPWM, FREQ, RES);

  frenar();

  WiFi.begin(wifi_ssid, wifi_password);
  Serial.print("Conectando WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi OK — IP: " + WiFi.localIP().toString());

  // --- Cambio mDNS ---
  // "robot-esp32" es el nombre que el ESP32 anuncia en la red
  // (quedaria como robot-esp32.local). No tiene que coincidir
  // con mqtt_hostname, son cosas distintas.
  if (!MDNS.begin("robot-esp32")) {
    Serial.println("Error iniciando mDNS");
  }

  resolverMqttHost();
  mqttClient.setServer(mqtt_ip, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  mqttReconnect();

  Serial.println("Robot Mecanum listo");
}

void loop() {
  if (!mqttClient.connected()) mqttReconnect();
  mqttClient.loop();
}