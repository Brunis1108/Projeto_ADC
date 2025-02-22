# Projeto de Controle de LEDs RGB e Display SSD1306 com Joystick no RP2040

Este repositório contém o código-fonte e a documentação para a atividade prática da Unidade 4, Capítulo 8, sobre Conversores A/D (ADC) no microcontrolador RP2040. O projeto utiliza a placa de desenvolvimento BitDogLab para controlar a intensidade de LEDs RGB com base nos valores de um joystick e exibir a posição do joystick em um display SSD1306.

## Objetivos

- Compreender o funcionamento do conversor analógico-digital (ADC) no RP2040.
- Utilizar o PWM para controlar a intensidade de dois LEDs RGB com base nos valores do joystick.
- Representar a posição do joystick no display SSD1306 por meio de um quadrado móvel.
- Aplicar o protocolo de comunicação I2C na integração com o display.

## Descrição do Projeto

O projeto consiste em controlar a intensidade luminosa de LEDs RGB com base nos valores analógicos fornecidos pelo joystick. O joystick fornece valores para os eixos X e Y, que são utilizados para ajustar o brilho dos LEDs. Além disso, a posição do joystick é representada por um quadrado móvel no display SSD1306.

### Funcionalidades

- **Controle de LEDs RGB:**
  - O LED Azul tem seu brilho ajustado conforme o valor do eixo Y.
  - O LED Vermelho tem seu brilho ajustado conforme o valor do eixo X.
  - O LED Verde é alternado a cada acionamento do botão do joystick.

- **Display SSD1306:**
  - Um quadrado de 8x8 pixels é exibido no display, movendo-se proporcionalmente aos valores do joystick.
  - O botão do joystick alterna o estilo da borda do display a cada acionamento.

- **Botão A:**
  - Ativa ou desativa o controle PWM dos LEDs.

## Componentes Utilizados

- **LED RGB:** Conectado às GPIOs 11, 12 e 13.
- **Joystick:** Conectado às GPIOs 26 (eixo X) e 27 (eixo Y).
- **Botão do Joystick:** Conectado à GPIO 22.
- **Botão A:** Conectado à GPIO 5.
- **Display SSD1306:** Conectado via I2C (GPIOs 14 e 15).

## Requisitos do Projeto

1. **Uso de Interrupções:** Todas as funcionalidades relacionadas aos botões devem ser implementadas utilizando rotinas de interrupção (IRQ).
2. **Debouncing:** Implementação do tratamento do bouncing dos botões via software.
3. **Utilização do Display 128x64:** Demonstração do entendimento do princípio de funcionamento do display e do protocolo I2C.
4. **Organização do Código:** O código deve estar bem estruturado e comentado para facilitar o entendimento.

## Estrutura do Código

O código está organizado em funções que inicializam os componentes, configuram o ADC, PWM, e I2C, e implementam as funcionalidades descritas. O uso de interrupções e debouncing é aplicado para garantir a responsividade e a precisão do sistema.

### Funções Principais

- **led_init():** Inicializa os pinos dos LEDs RGB.
- **button_init():** Configura os pinos dos botões e habilita as interrupções.
- **display_init():** Configura o display SSD1306 via I2C.
- **iniciar_adc():** Inicializa o ADC para leitura dos valores do joystick.
- **setup_pwm():** Configura o PWM para controle dos LEDs.
- **debounce():** Implementa o debouncing para os botões.

## Como Executar

1. Clone este repositório.
2. Compile o código utilizando o ambiente de desenvolvimento apropriado para o RP2040.
3. Carregue o código na placa BitDogLab.
4. Conecte os componentes conforme descrito.
5. Execute o projeto e interaja com o joystick e botões para observar as funcionalidades.

## Vídeo de Demonstração

Link: https://www.youtube.com/watch?v=08czqBzPMUc

## Considerações Finais

Este projeto é uma excelente oportunidade para consolidar conhecimentos em programação de microcontroladores, manipulação de hardware e utilização do periférico ADC.
