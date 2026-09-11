/* =========================================================================
   BALANÇA NUTRICIONAL IoT — ESP32 + Wokwi + Blynk IoT (via MQTT)
   Versão para PlatformIO (src/main.cpp)
   =========================================================================

   SENSORES (2):
     1) HX711 + Célula de Carga -> peso do alimento sobre o prato (g)
     2) DHT22                   -> temperatura (°C) e umidade (%) do
                                    ambiente onde o alimento é mantido

   ATUADORES (2):
     1) Buzzer + LED vermelho — AUTOMÁTICO (regra de negócio)
        Dispara sozinho quando:
          a) o total de calorias do dia ultrapassa a meta da dieta, OU
          b) a temperatura ambiente ultrapassa 30 °C (risco de o
             alimento estragar)
     2) Servo motor — MANUAL
        Abre/fecha a "tampa inteligente" do recipiente. É controlado
        exclusivamente pelo usuário, através de um Switch no dashboard
        do Blynk (datastream TampaManual).

   COMUNICAÇÃO: MQTT puro, usando a Device MQTT API oficial da Blynk.Cloud
   (NÃO é a biblioteca Blynk.h clássica — aqui falamos MQTT de verdade
   com a lib PubSubClient, publicando/assinando os tópicos documentados
   em https://docs.blynk.io/en/blynk.cloud-mqtt-api/device-mqtt-api)

     Publicar dado do dispositivo p/ nuvem:  ds/<NomeDoDatastream>
     Receber alterações vindas do app:       downlink/ds/<NomeDoDatastream>

   ========================================================================= */

#include <WiFi.h>
#include <PubSubClient.h>
#include "HX711.h"
#include <ESP32Servo.h>
#include "DHTesp.h"
// A documentação oficial do Wokwi recomenda especificamente esta biblioteca
// pra ler o wokwi-dht22 a partir de um ESP32 (outras podem não funcionar de
// forma confiável com esse componente simulado neste microcontrolador):
// https://docs.wokwi.com/parts/wokwi-dht22

// --------------------------- REDE (Wokwi) ---------------------------------
const char* WIFI_SSID     = "Wokwi-GUEST";   // rede virtual do Wokwi
const char* WIFI_PASSWORD = "";              // sem senha
const int   WIFI_CHANNEL  = 6;               // acelera a conexão no Wokwi

// --------------------------- BLYNK (MQTT) ----------------------------------
// Documentação: https://docs.blynk.io/en/blynk.cloud-mqtt-api/device-mqtt-api
const int   MQTT_PORT   = 1883;              // porta MQTT sem TLS (mais simples p/ Wokwi)
const char* MQTT_USER   = "device";          // literal, exigido pela Blynk
const char* BLYNK_AUTH_TOKEN = "q8L7CDpRZ_C2D_5otnqua3YqgvaS9-MD";
const char* MQTT_CLIENT_ID   = "ESP32-BalancaNutricional";

// A Blynk.Cloud pode pedir pra gente trocar de servidor (GeoDNS, servidor
// regional mais próximo) logo após conectar, via mensagem downlink/redirect.
// A própria documentação da Blynk avisa que IGNORAR isso pode fazer o
// dispositivo não conseguir trocar dados direito e aparecer como "Offline"
// — foi exatamente esse sintoma que você teve.
char mqttServerAtual[64] = "blynk.cloud";
char novoServidorPendente[64] = "";
bool precisaRedirecionar = false;

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
int   alimentoSelecionado = 0;       // V0 - vindo do Dropdown do Blynk
float metaCalorica        = 2000.0;  // V9 - meta diária (kcal), ajustável no app
float caloriasDoDia       = 0.0;     // V3 - acumulado do dia
bool  pratoOcupado         = false;   // controla quando somar uma nova refeição
bool  tampaAberta          = false;   // V8 - estado atual do servo

unsigned long ultimaLeitura = 0;
const unsigned long INTERVALO_LEITURA = 3000; // lê/envia a cada 3 segundos

// Limite de peso considerado uma porção excessiva (g). Usado pelo ATUADOR 1
// (automático) — diferente de metaCalorica, este valor reage em tempo real
// ao peso ATUAL sobre a balança, então o alarme liga e desliga sozinho
// conforme você tira/coloca peso, em vez de ficar "travado" ligado.
const float LIMITE_PESO_PORCAO = 300.0;

// ===============================================================================
// PROTÓTIPOS DE FUNÇÃO
// Necessário no PlatformIO: diferente do Arduino IDE (que gera isso sozinho
// para arquivos .ino), um arquivo .cpp "puro" exige que toda função usada
// antes de ser definida no arquivo seja declarada aqui primeiro.
// ===============================================================================
void conectarWiFi();
void reconectarMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publicarBlynkBatch(float peso, float caloriasAtual, float caloriasDia,
                         float metaProgresso, float temperatura, float umidade,
                         int alarmeStatus);
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

  mqtt.setServer(mqttServerAtual, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}

// ===============================================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) conectarWiFi();

  // Se o servidor pediu redirecionamento (GeoDNS), troca e reconecta
  if (precisaRedirecionar) {
    precisaRedirecionar = false;
    strncpy(mqttServerAtual, novoServidorPendente, sizeof(mqttServerAtual));
    mqtt.disconnect();
    mqtt.setServer(mqttServerAtual, MQTT_PORT);
    Serial.print(">> Reconectando no servidor regional: ");
    Serial.println(mqttServerAtual);
  }

  if (!mqtt.connected()) reconectarMQTT();
  mqtt.loop();

  if (millis() - ultimaLeitura >= INTERVALO_LEITURA) {
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
    Serial.print("Conectando ao Blynk via MQTT...");
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, BLYNK_AUTH_TOKEN)) {
      Serial.println(" ok!");

      // Assina TODAS as mensagens que a Blynk manda para o dispositivo
      // (mudanças de widgets, ping, redirect, etc.)
      mqtt.subscribe("downlink/#");

      // Pede o valor atual dos datastreams de entrada, para sincronizar
      // caso o ESP32 tenha reiniciado e o app já tenha valores salvos.
      mqtt.publish("get/ds", "Alimento,MetaCalorica,TampaManual");

    } else {
      Serial.print(" falhou (rc=");
      Serial.print(mqtt.state());
      Serial.println("), tentando de novo em 3s...");
      Serial.println("Dica: confira o AUTH TOKEN e, se persistir, tente");
      Serial.println("trocar MQTT_SERVER pelo servidor regional mostrado");
      Serial.println("na aba 'Info' do seu device no console da Blynk.");
      delay(3000);
    }
  }
}

// Executado toda vez que chega uma mensagem MQTT vinda do Blynk
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  String t = String(topic);

  Serial.print("MQTT [");
  Serial.print(t);
  Serial.print("] -> ");
  Serial.println(msg);

  // Servidor pediu pra trocar de endpoint (GeoDNS) — extrai só o host da
  // URI (ex: "mqtt://ny3.blynk.cloud:1883") e sinaliza pro loop() trocar,
  // mantendo nossa porta 1883 sem TLS.
  if (t == "downlink/redirect") {
    int hostStart = msg.indexOf("://");
    if (hostStart >= 0) {
      hostStart += 3;
      int hostEnd = msg.indexOf(':', hostStart);
      if (hostEnd < 0) hostEnd = msg.indexOf('/', hostStart);
      if (hostEnd < 0) hostEnd = msg.length();
      String host = msg.substring(hostStart, hostEnd);
      host.toCharArray(novoServidorPendente, sizeof(novoServidorPendente));
      precisaRedirecionar = true;
      Serial.print(">> Blynk pediu redirecionamento para: ");
      Serial.println(novoServidorPendente);
    }
    return;
  }

  // V0 - Dropdown "Alimento" (0=Arroz, 1=Frango, 2=Ovo)
  if (t == "downlink/ds/Alimento") {
    int v = msg.toInt();
    alimentoSelecionado = (v >= 0 && v <= 2) ? v : 0;
  }

  // V9 - meta calórica ajustável pelo app
  else if (t == "downlink/ds/MetaCalorica") {
    metaCalorica = msg.toFloat();
  }

  // V8 - ATUADOR 2 (MANUAL): Switch que abre/fecha a tampa (servo)
  else if (t == "downlink/ds/TampaManual") {
    tampaAberta = (msg.toInt() == 1);
    tampa.write(tampaAberta ? 90 : 0);
    Serial.println(tampaAberta ? ">> Tampa ABERTA (comando manual)"
                                : ">> Tampa FECHADA (comando manual)");
  }

  // V10 - botão opcional para reiniciar o acumulado do dia (uso em demonstração)
  else if (t == "downlink/ds/ResetarDia") {
    if (msg.toInt() == 1) {
      caloriasDoDia = 0;
      Serial.println(">> Total de calorias do dia foi zerado.");
    }
  }
}

// Publica VÁRIOS datastreams de uma vez só, em uma única mensagem MQTT
// (tópico batch_ds da Blynk), reduzindo o número de mensagens enviadas
// por ciclo de 7 para 1.
void publicarBlynkBatch(float peso, float caloriasAtual, float caloriasDia,
                         float metaProgresso, float temperatura, float umidade,
                         int alarmeStatus) {
  char payload[220];
  snprintf(payload, sizeof(payload),
    "{\"Peso\":%.2f,\"CaloriasAtual\":%.2f,\"CaloriasDia\":%.2f,"
    "\"MetaProgresso\":%.2f,\"Temperatura\":%.2f,\"Umidade\":%.2f,"
    "\"AlarmeStatus\":%d}",
    peso, caloriasAtual, caloriasDia, metaProgresso, temperatura, umidade, alarmeStatus);
  mqtt.publish("batch_ds", payload);
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
  // A DHTesp não usa NAN para indicar falha (diferente da lib Adafruit) —
  // o jeito certo de checar é via getStatus().
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
  // voltar pra baixo dele. O acumulado do dia (caloriasDoDia/metaCalorica)
  // continua sendo mostrado no dashboard (V3/V4), só não controla mais
  // o buzzer/LED diretamente.
  bool porcaoExcessiva = peso > LIMITE_PESO_PORCAO;
  bool temperaturaAlta = temperatura > 30.0;
  bool alarmeAtivo     = porcaoExcessiva || temperaturaAlta;

  digitalWrite(LED_ALARME, alarmeAtivo ? HIGH : LOW);
  // Usamos um bipe "na unha" (digitalWrite alternado, sem tone()/LEDC) de
  // propósito: no ESP32, a função tone() e a biblioteca Servo disputam o
  // mesmo periférico de PWM internamente. Usar tone() "rouba" o canal da
  // Servo e trava o motor até a placa ser reiniciada — era esse o motivo
  // da tampa parar de responder depois do primeiro alarme. Um simples
  // digitalWrite constante resolvia o conflito, mas o buzzer é um piezo
  // passivo: só faz som com uma tensão oscilando, não com nível fixo.
  if (alarmeAtivo) {
    bipManual(BUZZER_PIN, 2000, 150); // bipe de ~150ms a 2kHz
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // ============== Envio dos dados para o Blynk (MQTT) ==============
  // Tudo em UMA mensagem só (batch_ds), em vez de 7 separadas.
  publicarBlynkBatch(peso, caloriasRefeicao, caloriasDoDia, progresso,
                      temperatura, umidade, alarmeAtivo ? 1 : 0);

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
// bipe (bem curto, alguns milissegundos), o que é aceitável já que roda
// só uma vez por ciclo de leitura.
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