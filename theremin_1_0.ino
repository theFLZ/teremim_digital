#include "Wire.h"
#include "Adafruit_VL53L0X.h"
#include "AudioToolsMin.h"
#include "math.h"

#define limiteAlcanceMM 700

#define saida_led 15

// Alimentações extras
#define Vcc_1 14 //alimentação do potenciômetro
#define Vcc_2 19 //alimentação do UDA1334
#define Vcc_3 32 //alimentação do sensor de volume
#define Vcc_4 18 //alimentação relacionada ao botão de distorção
// 3v3 padrão sendo de alimentação pro sensor de frequencia

// Potenciômetro para frequência
#define Pot_setfreqmax 13

// botão para alterar distorção
#define btn_distorcao  34

// botões para forçar desligamento dos sensores
#define XSHUT_1 5
#define XSHUT_2 26

// endereço de assinatura do I2C para os dois sensores
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// Comunicação Bluetooth: causou muita intereferência no circuito,
// prejudicando o funcionamento do mesmo
//BluetoothSerial SerialBT;

Adafruit_VL53L0X lox = Adafruit_VL53L0X(); //comunicação I2C com o sensor
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

TwoWire I2C = TwoWire(0); //configuração de dois pinos da I2C
TwoWire I2C_2 = TwoWire(1); //configuração de dois pinos da I2C

AudioInfo info(22000, 2, 16);
SineWaveGenerator<int16_t> sineWave(1000);     // subclasse do soundGenerator com amplitude máxima de 32000
GeneratedSoundStream<int16_t> sound(sineWave); // transmissão gerada por uma onda senoidal

I2SStream out; 
StreamCopy copier(out, sound);     // copia o som no I2S

/* Protótipo das tarefas */
void vEnviaAudio(void *pvParameters);
void vLeFrequencia(void *pvParameters);
void vLeVolume(void *pvParameters);
void vSetFreqMaxAndDist(void *pvParameters);

// Frequência máxima de operação (definida pelo potenciômetro)
int freqMax = 400; 

// atual distorção aplicada
int distorcao_n = 0;

// Variáveis para se caso a distância for maior que o máximo, ficar zero
uint16_t distFreq = 0;
uint16_t distVol = 0;

// controle de debounce do botão de alterar distorção
int debounce_delay = 100; //em ms
bool nivel_passado_botao = HIGH;
bool nivel_antes_debounce_botao = HIGH;
volatile bool botao_pressionado = false;

unsigned long ultima_mudanca_switch = 0;

void setup() {
  Serial.begin(115200);
  
  // Pinos de ativação dos sensores (ao utilizar mais de 1, é necessário, controloar apenas pelo Vin não funciona)
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  // Desligar ambos sensores
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(10);
  
  pinMode(Vcc_1, OUTPUT);
  pinMode(Vcc_2, OUTPUT);
  pinMode(Vcc_3, OUTPUT);
  pinMode(Vcc_4, OUTPUT);
  digitalWrite(Vcc_1,HIGH);
  // O Vcc_2 é do UDA1334, irei ativar mais adiante
  digitalWrite(Vcc_3,HIGH);
  digitalWrite(Vcc_4,HIGH);

  pinMode(saida_led, OUTPUT);
  digitalWrite(saida_led, LOW);

  // Par de pinos para comunicação I2C
  I2C.begin(16,4);
  I2C_2.begin(25,33);

  // Sensor 1
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  Serial.println("Adafruit VL53L0X test");
  while (!lox.begin(LOX1_ADDRESS, false, &I2C)) {
    Serial.println(F("Failed to boot VL53L0X\n"));
  }
  delay(50);

  // Sensor 2
  Serial.println("Adafruit 2 VL53L0X test");
  digitalWrite(XSHUT_2, HIGH);
  delay(50);
  while (!lox2.begin(LOX2_ADDRESS, false, &I2C_2)) {
    Serial.println(F("Failed to boot VL53L0X 2\n"));
  }

  // criando o I2S
  digitalWrite(Vcc_2,HIGH); // Pino do UDA1334
  Serial.println("starting I2S...");
  auto config = out.defaultConfig(TX_MODE);

  // Pinos de saída I2S
  config.pin_bck = 21;
  config.pin_data = 22;
  config.pin_ws = 23;
  out.begin(config);

  // Setup da onda senoidal
  sineWave.begin(info, N_C4);
  Serial.println("started...");

  // criando as tasks em determinados núcleos (0 ou 1)
  xTaskCreatePinnedToCore(vEnviaAudio,        "EnviaAudio",       2048, NULL, 20, NULL, 0);
  xTaskCreatePinnedToCore(vLeFrequencia,      "LeFrequencia",     2048, NULL, 15, NULL, 1);
  xTaskCreatePinnedToCore(vLeVolume,          "LeVolume",         2048, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(vSetFreqMaxAndDist, "setaFreqMax",      2048, NULL, 5,  NULL, 1);
  
  // Botão para alterar a distorção
  pinMode(btn_distorcao, INPUT);

}

VL53L0X_RangingMeasurementData_t measure_1;
VL53L0X_RangingMeasurementData_t measure_2;

int volume = 0;
float amp = 0;

unsigned long ultimaExec = millis();

// frequência com base na distância
float freq = 200;

void loop() {
  // Todo o processamento é feito nas tasks
}

void vEnviaAudio(void *pvParameters){
  while(true){
    copier.copy();
  }
}

// suavização de curva
float alpha = 0.8f; // de 0 a 1 (quanto menor, mais suave, mas mais lento)
float suavizado = 0;


void vLeFrequencia(void *pvParameters){

  while (true) {
    lox.rangingTest(&measure_1, false); // pass in 'true' to get debug data printout!
  
    if (measure_1.RangeStatus != 4) {  // phase failures have incorrect data
      
      distFreq = measure_1.RangeMilliMeter;

      if (distFreq <= limiteAlcanceMM){
        // Distancia Atual, Distancia Minima, Distancia Maxima, Frequencia Maxima, Frequencia Minima
        freq = float(map(distFreq, 0, limiteAlcanceMM, freqMax, 20));
      } else {
        freq = 0.0;
      }

    } else {
      freq = 0.0;
    }
    
    // suavizar com curva exponencial é melhor, mas ainda não tem tanta suavização quanto precisaria (quanto menor alpha, mais suave, mas mais lento)
    suavizado = suavizado + alpha * (freq - suavizado);
    
    freq = suavizado;

    sineWave.setFrequency(freq);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void vLeVolume(void *pvParameters){
  
  while(true) {

    lox2.rangingTest(&measure_2, false); // passe 'true' para debugar os dados

    if (measure_2.RangeStatus != 4) {  // reusltado == 4 possuem dados incorretos de dados (falha)
      
      distVol = measure_2.RangeMilliMeter;
      
      amp = 3000.0f * (1.0f - (float)distVol / limiteAlcanceMM);
      if (amp < 0) amp = 0;
    } else {
      amp = 0.0;
    }

    sineWave.setAmplitude(amp);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vSetFreqMaxAndDist(void *pvParameters){
  
  int debouncingContagem = 0;

  while(true){

    freqMax = float(map(analogRead(Pot_setfreqmax), 0, 1850, 400, 3000));

    /****************/
    if (digitalRead(btn_distorcao) == LOW){
      debouncingContagem++;
      if(debouncingContagem >= 15){
        debouncingContagem = 0;

        if (distorcao_n > 10){
          distorcao_n = -1;
        }
        sineWave.setDistorcao(++distorcao_n);
        digitalWrite(saida_led, !digitalRead(saida_led));
      }
    }
    else {
      debouncingContagem = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}