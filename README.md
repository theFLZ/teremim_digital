LEIA-ME

Os arquivos são os principais para que o processamento do teremim digital seja feito corretamente. Para isso:
1. Instalar a IDE do Arduino e possuir os reespectivos componentes adequados: ESP032, VL53L0X, UDA1334, resistores 1k e 10k e portênciometro de 1k;
2. Instalar as bibliotecas "Wire.h", "Adafruit_VL53L0X.h"e "AudioTools.h" na IDE do Arduino;
3. No diretório da biblioteca "AudioTools" incluir o arquivo "AudioToolsMin.h" e substituir o arquivo "SoundGenerator.h" pelo arquivo presente aqui nesse repositório;

Se tudo ocorrer sem problemas o teremim deverá funcionar corretamente após compilar e fazer o upload pela IDE do Arduino.

Os arquivos do teremim digital implementado fisicamente:
SCH_teremin_2025-11-19.json - arquivo esquema do EasyEDA;
PCB_teremin_2025-11-19.json - arquivo da PCB do EasyEDA;
caixa_teremim.f3d - modelo 3D feito no Fusion.
