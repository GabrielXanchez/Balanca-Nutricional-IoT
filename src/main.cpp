/* =========================================================================
   BALANÇA NUTRICIONAL IoT — ESP32 + Wokwi + MQTT (broker público)
   Versão 2: SEM Blynk. Os dados são consumidos por um app WEB (HTML/CSS/JS)
   e por um app MOBILE (React Native / Expo) que se conectam ao MESMO broker.
   =========================================================================

   SENSORES (2):
     1) HX711 + Célula de Carga -> peso do alimento sobre o prato (g)
     2) DHT22                   -> temperatura (°C) e umidade (%) do
                                    ambiente onde o alimento é mantido

   ATUADORES (2):
     1) Buzzer + LED vermelho — AUTOMÁTICO (regra de negócio)
        Dispara sozinho quando:
          a) a porção sobre a balança ultrapassa LIMITE_PESO_PORCAO, OU
          b) a temperatura ambiente ultrapassa LIMITE_TEMPERATURA
     2) Servo motor — MANUAL
        Abre/fecha a "tampa inteligente" do recipiente. Só se move quando
        o usuário envia o comando pelo app web ou pelo app mobile.

   COMUNICAÇÃO: MQTT (biblioteca PubSubClient) em um broker público.
   Todos os tópicos ficam sob  balanca-nutricional/<PROJECT_ID>/...

     ESP32 -> apps (telemetria, publicada com retain):
       .../peso                     peso em gramas
       .../calorias/refeicao        kcal do alimento que está no prato
       .../calorias/dia             kcal acumuladas no dia
       .../meta/progresso           % da meta diária já consumida
       .../ambiente/temperatura     °C
       .../ambiente/umidade         %
       .../alarme/status            0 = normal, 1 = alarme ativo
       .../alarme/motivo            nenhum | porcao | temperatura | porcao+temperatura
       .../tampa/estado             0 = fechada, 1 = aberta
       .../config/alimento          0 = Arroz, 1 = Frango, 2 = Ovo
       .../config/meta              meta calórica diária (kcal)
       .../config/limite_peso       limite de porção (g)
       .../config/limite_temperatura  limite de temperatura (°C)
       .../status                   online | offline (Last Will do MQTT)

     apps -> ESP32 (comandos):
       .../cmd/alimento             0, 1 ou 2           (retain)
       .../cmd/meta                 100 a 10000 kcal    (retain)
       .../cmd/tampa                0 = fechar, 1 = abrir (retain)
       .../cmd/resetar              1 = zerar calorias do dia

   ========================================================================= */

#include <WiFi.h>
#include <PubSubClient.h>
#include "HX711.h"
#include <ESP32Servo.h>
#include "DHTesp.h"
// A documentação oficial do Wokwi recomenda especificamente a DHTesp para
// ler o wokwi-dht22 a partir de um ESP32:
// https://docs.wokwi.com/parts/wokwi-dht22

// --------------------------- REDE (Wokwi) ---------------------------------
const char* WIFI_SSID     = "Wokwi-GUEST";   // rede virtual do Wokwi
const char* WIFI_PASSWORD = "";              // sem senha
const int   WIFI_CHANNEL  = 6;               // acelera a conexão no Wokwi

// --------------------------- MQTT (broker público) --------------------------
const char* MQTT_SERVER = "broker.hivemq.com";   // alternativa: "broker.emqx.io"
const int   MQTT_PORT   = 1883;                  // MQTT sem TLS (TCP)

// >>> TROQUE por um texto único seu (letras/números, sem espaços nem "/").
// >>> Use EXATAMENTE o mesmo valor em web/config.js e mobile/config.js.
// O broker é público: quem souber esse ID enxerga e comanda a sua balança.
#define PROJECT_ID  "bn7k2q9"

#define BASE_TOPIC       "balanca-nutricional/" PROJECT_ID
#define MQTT_CLIENT_ID   "esp32-balanca-" PROJECT_ID

// Telemetria (ESP32 -> apps)
#define T_PESO            BASE_TOPIC "/peso"
#define T_CAL_REFEICAO    BASE_TOPIC "/calorias/refeicao"
#define T_CAL_DIA         BASE_TOPIC "/calorias/dia"
#define T_PROGRESSO       BASE_TOPIC "/meta/progresso"
#define T_TEMPERATURA     BASE_TOPIC "/ambiente/temperatura"
#define T_UMIDADE         BASE_TOPIC "/ambiente/umidade"
#define T_ALARME_STATUS   BASE_TOPIC "/alarme/status"
#define T_ALARME_MOTIVO   BASE_TOPIC "/alarme/motivo"
#define T_TAMPA_ESTADO    BASE_TOPIC "/tampa/estado"
#define T_CFG_ALIMENTO    BASE_TOPIC "/config/alimento"
#define T_CFG_META        BASE_TOPIC "/config/meta"
#define T_CFG_LIM_PESO    BASE_TOPIC "/config/limite_peso"
#define T_CFG_LIM_TEMP    BASE_TOPIC "/config/limite_temperatura"
#define T_STATUS          BASE_TOPIC "/status"

// Comandos (apps -> ESP32)
#define T_CMD_ALIMENTO    BASE_TOPIC "/cmd/alimento"
#define T_CMD_META        BASE_TOPIC "/cmd/meta"
#define T_CMD_TAMPA       BASE_TOPIC "/cmd/tampa"
#define T_CMD_RESETAR     BASE_TOPIC "/cmd/resetar"

WiFiClient   espClient;
PubSubClient mqtt(espClient);

// ------------------------------ PINAGEM -------------------------------------
#define HX711_SCK   4     // ESP32 -> HX711 SCK
#define HX711_DT    19    // ESP32 <- HX711 DT
#define DHT_PIN     15    // ESP32 <- DHT22 SDA
#define BUZZER_PIN  25    // ATUADOR 1 (parte sonora)
#define LED_ALARME  26    // ATUADOR 1 (parte luminosa)
#define SERVO_PIN   13    // ATUADOR 2 (tampa - controle manual)

HX711  balanca;
Servo  tampa;
DHTesp dhtSensor;

// -------------------------- TABELA NUTRICIONAL -------------------------------
// Calorias = Peso(g) x (kcal por 100g / 100)   <- fórmula do projeto original
struct Alimento {
  const char* nome;
  float kcalPor100g;
};

Alimento tabelaAlimentos[3] = {
  { "Arroz cozido",    130.0 },  // índice 0
  { "Frango grelhado", 165.0 },  // índice 1
  { "Ovo cozido",      155.0 }   // índice 2
};

// ------------------------------- ESTADO ---------------------------------------
int   alimentoSelecionado = 0;       // escolhido no app (cmd/alimento)
float metaCalorica        = 2000.0;  // meta diária (kcal), ajustável no app
float caloriasDoDia       = 0.0;     // acumulado do dia
bool  pratoOcupado         = false;   // controla quando somar uma nova refeição
bool  tampaAberta          = false;   // estado atual do servo

unsigned long ultimaLeitura = 0;
const unsigned long INTERVALO_LEITURA = 3000; // lê/envia a cada 3 segundos

// Limites usados pela regra automática do ATUADOR 1 (e publicados para os
// apps, que só os exibem — a decisão é sempre tomada aqui no ESP32).
const float LIMITE_PESO_PORCAO = 300.0;   // g
const float LIMITE_TEMPERATURA = 30.0;    // °C

// Controle da comunicação
bool forcarLeitura = false;          // true = processa/publica já, sem esperar 3 s
bool configPendente = false;         // true = falta publicar o estado inicial
unsigned long instanteConexao = 0;

// ===============================================================================
// PROTÓTIPOS DE FUNÇÃO (necessários em arquivos .cpp do PlatformIO)
// ===============================================================================
void conectarWiFi();
void reconectarMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publicar(const char* topico, const String& valor, bool retido = true);
void publicarConfig();
void processarSensoresEEnviar();
void bipManual(int pin, int freqHz, int duracaoMs);

// ===============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_ALARME, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_ALARME, LOW);

  // ---- SENSOR 1: HX711 ----
  balanca.begin(HX711_DT, HX711_SCK);
  // A célula simulada no Wokwi está configurada como "5kg" (faixa crua 0-2100).
  // 2100 contagens equivalem a 5000 g  ->  fator = 2100 / 5000 = 0.42
  balanca.set_scale(0.42f);
  balanca.tare();  // zera a balança (prato vazio ao ligar)

  // ---- SENSOR 2: DHT22 ----
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  // ---- ATUADOR 2: Servo (tampa) ----
  tampa.attach(SERVO_PIN);
  tampa.write(0);  // tampa começa fechada

  conectarWiFi();

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}

// ===============================================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) conectarWiFi();
  if (!mqtt.connected()) reconectarMQTT();
  mqtt.loop();

  // Logo após conectar, o broker entrega os últimos comandos guardados (retain).
  // Esperamos um instante para aplicá-los e só então publicamos o estado.
  if (configPendente && millis() - instanteConexao > 1500) {
    configPendente = false;
    publicarConfig();
  }

  if (forcarLeitura || millis() - ultimaLeitura >= INTERVALO_LEITURA) {
    forcarLeitura = false;
    ultimaLeitura = millis();
    processarSensoresEEnviar();
  }
}

// ------------------------------- WI-FI -----------------------------------------
void conectarWiFi() {
  Serial.print("Conectando ao WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println(" conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ------------------------------- MQTT -------------------------------------------
void reconectarMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Conectando ao broker MQTT...");
    // Last Will: se o ESP32 cair sem avisar, o broker publica "offline" em T_STATUS
    if (mqtt.connect(MQTT_CLIENT_ID, nullptr, nullptr, T_STATUS, 0, true, "offline")) {
      Serial.println(" ok!");
      mqtt.publish(T_STATUS, "online", true);
      mqtt.subscribe(BASE_TOPIC "/cmd/#");   // recebe todos os comandos dos apps
      configPendente = true;
      instanteConexao = millis();
    } else {
      Serial.print(" falhou (rc=");
      Serial.print(mqtt.state());
      Serial.println("), tentando de novo em 3s...");
      delay(3000);
    }
  }
}

// Executado toda vez que chega um comando MQTT vindo dos apps
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  String t = String(topic);

  Serial.print("MQTT [");
  Serial.print(t);
  Serial.print("] -> ");
  Serial.println(msg);

  // Dropdown de alimento (0=Arroz, 1=Frango, 2=Ovo)
  if (t == T_CMD_ALIMENTO) {
    int v = msg.toInt();
    if (v >= 0 && v <= 2) {
      alimentoSelecionado = v;
      publicar(T_CFG_ALIMENTO, String(alimentoSelecionado));
      forcarLeitura = true;
    }
  }

  // Meta calórica diária
  else if (t == T_CMD_META) {
    float v = msg.toFloat();
    if (v >= 100 && v <= 10000) {
      metaCalorica = v;
      publicar(T_CFG_META, String(metaCalorica, 0));
      forcarLeitura = true;
    }
  }

  // ATUADOR 2 (MANUAL): abre/fecha a tampa (servo)
  else if (t == T_CMD_TAMPA) {
    tampaAberta = (msg.toInt() == 1);
    tampa.write(tampaAberta ? 90 : 0);
    publicar(T_TAMPA_ESTADO, tampaAberta ? "1" : "0");
    Serial.println(tampaAberta ? ">> Tampa ABERTA (comando manual)"
                                : ">> Tampa FECHADA (comando manual)");
  }

  // Zera o acumulado do dia (uso em demonstração / novo dia)
  else if (t == T_CMD_RESETAR) {
    if (msg.toInt() == 1) {
      caloriasDoDia = 0;
      forcarLeitura = true;
      Serial.println(">> Total de calorias do dia foi zerado.");
    }
  }
}

// Publica um valor (retain = true por padrão: quem abrir o app depois já
// recebe o último valor, sem esperar o próximo ciclo)
void publicar(const char* topico, const String& valor, bool retido) {
  mqtt.publish(topico, valor.c_str(), retido);
}

// Estado e configurações que não mudam a cada ciclo
void publicarConfig() {
  publicar(T_CFG_ALIMENTO, String(alimentoSelecionado));
  publicar(T_CFG_META, String(metaCalorica, 0));
  publicar(T_CFG_LIM_PESO, String(LIMITE_PESO_PORCAO, 0));
  publicar(T_CFG_LIM_TEMP, String(LIMITE_TEMPERATURA, 0));
  publicar(T_TAMPA_ESTADO, tampaAberta ? "1" : "0");
}

// ------------------------- LÓGICA PRINCIPAL DO PROJETO ---------------------------
void processarSensoresEEnviar() {

  // ============== SENSOR 1: peso (HX711) ==============
  float peso = 0;
  if (balanca.is_ready()) {
    peso = balanca.get_units(5);
    if (peso < 0) peso = 0;
  } else {
    Serial.println("Aviso: HX711 não respondeu nesta leitura.");
  }

  // ============== Cálculo nutricional ==============
  float kcal100g = tabelaAlimentos[alimentoSelecionado].kcalPor100g;
  float caloriasRefeicao = peso * (kcal100g / 100.0);

  // Só soma ao total do dia quando detecta uma NOVA refeição sendo colocada
  // (transição "prato vazio" -> "prato ocupado"), evitando somar sem parar
  // enquanto o mesmo alimento permanece sobre a balança.
  if (!pratoOcupado && peso > 10) {
    pratoOcupado = true;
    caloriasDoDia += caloriasRefeicao;
    Serial.println(">> Nova refeição detectada — calorias somadas ao total do dia.");
  } else if (pratoOcupado && peso < 5) {
    pratoOcupado = false; // prato foi retirado, pronto para a próxima pesagem
  }

  float progresso = (metaCalorica > 0) ? (caloriasDoDia / metaCalorica) * 100.0 : 0;
  if (progresso > 100) progresso = 100;

  // ============== SENSOR 2: temperatura/umidade (DHT22) ==============
  TempAndHumidity leituraDht = dhtSensor.getTempAndHumidity();
  float temperatura = leituraDht.temperature;
  float umidade      = leituraDht.humidity;
  // A DHTesp não usa NAN para indicar falha — o jeito certo de checar é getStatus().
  bool leituraDhtOk = (dhtSensor.getStatus() == DHTesp::ERROR_NONE);

  if (!leituraDhtOk) {
    Serial.print("Aviso: leitura do DHT22 falhou. Status: ");
    Serial.println(dhtSensor.getStatusString());
    temperatura = -99;
    umidade = -99;
  }

  // ============== ATUADOR 1 (AUTOMÁTICO) — regra de negócio ==============
  // Reage ao peso ATUAL (não ao acumulado do dia), por isso é totalmente
  // reversível: liga sozinho ao ultrapassar o limite, desliga sozinho ao
  // voltar pra baixo dele.
  bool porcaoExcessiva = peso > LIMITE_PESO_PORCAO;
  bool temperaturaAlta = temperatura > LIMITE_TEMPERATURA;
  bool alarmeAtivo     = porcaoExcessiva || temperaturaAlta;

  const char* motivoAlarme = (porcaoExcessiva && temperaturaAlta) ? "porcao+temperatura"
                           : porcaoExcessiva ? "porcao"
                           : temperaturaAlta ? "temperatura"
                           : "nenhum";

  digitalWrite(LED_ALARME, alarmeAtivo ? HIGH : LOW);
  // Bipe "na unha" (digitalWrite alternado, sem tone()/LEDC): no ESP32 a função
  // tone() e a biblioteca Servo disputam o mesmo periférico de PWM, e usar
  // tone() travaria a tampa. O buzzer é um piezo passivo: só faz som com uma
  // tensão oscilando, por isso o nível fixo não bastaria.
  if (alarmeAtivo) {
    bipManual(BUZZER_PIN, 2000, 150); // bipe de ~150ms a 2kHz
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // ============== Envio dos dados para os apps (MQTT) ==============
  publicar(T_PESO,          String(peso, 2));
  publicar(T_CAL_REFEICAO,  String(caloriasRefeicao, 2));
  publicar(T_CAL_DIA,       String(caloriasDoDia, 2));
  publicar(T_PROGRESSO,     String(progresso, 2));
  if (leituraDhtOk) {        // não publica lixo se o sensor falhou
    publicar(T_TEMPERATURA, String(temperatura, 2));
    publicar(T_UMIDADE,     String(umidade, 2));
  }
  publicar(T_ALARME_STATUS, alarmeAtivo ? "1" : "0");
  publicar(T_ALARME_MOTIVO, motivoAlarme);

  // ============== Log no Serial Monitor ==============
  Serial.println("---------------------------------------------------");
  Serial.print("Alimento selecionado : "); Serial.println(tabelaAlimentos[alimentoSelecionado].nome);
  Serial.print("Peso                 : "); Serial.print(peso); Serial.println(" g");
  Serial.print("Calorias da refeição : "); Serial.print(caloriasRefeicao); Serial.println(" kcal");
  Serial.print("Calorias do dia      : "); Serial.print(caloriasDoDia); Serial.println(" kcal");
  Serial.print("Meta calórica        : "); Serial.print(metaCalorica); Serial.println(" kcal");
  Serial.print("Progresso da meta    : "); Serial.print(progresso); Serial.println(" %");
  Serial.print("Temperatura          : "); Serial.print(temperatura); Serial.println(" C");
  Serial.print("Umidade              : "); Serial.print(umidade); Serial.println(" %");
  Serial.print("Alarme automático    : "); Serial.println(alarmeAtivo ? "ATIVO" : "inativo");
  Serial.print("  -> porção excessiva : "); Serial.println(porcaoExcessiva ? "sim" : "não");
  Serial.print("  -> temperatura alta : "); Serial.println(temperaturaAlta ? "sim" : "não");
  Serial.print("Tampa (manual)       : "); Serial.println(tampaAberta ? "ABERTA" : "FECHADA");
}

// Gera um bipe alternando o pino manualmente (sem usar tone()/LEDC), pra
// nunca disputar o hardware de PWM com a Servo. Bloqueia pelo tempo do
// bipe (bem curto), o que é aceitável já que roda só uma vez por ciclo.
void bipManual(int pin, int freqHz, int duracaoMs) {
  unsigned long periodoUs = 1000000UL / freqHz;
  unsigned long meioPeriodoUs = periodoUs / 2;
  unsigned long fimEm = millis() + duracaoMs;
  while (millis() < fimEm) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(meioPeriodoUs);
    digitalWrite(pin, LOW);
    delayMicroseconds(meioPeriodoUs);
  }
}
