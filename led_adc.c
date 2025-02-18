#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include <ctype.h>
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/adc.h"

// Definições para o display SSD1306 (comunicação I2C)
#define I2C_PORT i2c1          // Porta I2C utilizada (i2c1)
#define PIN_I2C_SDA 14         // Pino SDA (GPIO 14)
#define PIN_I2C_SCL 15         // Pino SCL (GPIO 15)
#define endereco 0x3C          // Endereço I2C do display SSD1306

// Definições dos pinos dos LEDs RGB
#define led_red 13             // Pino do LED Vermelho (GPIO 13)
#define led_blue 12            // Pino do LED Azul (GPIO 12)
#define led_green 11           // Pino do LED Verde (GPIO 11)

// Definição do pino do botão A
#define buttonA 5              // Pino do botão A (GPIO 5)
bool buttonA_state = false;    // Estado do botão A (ligado/desligado)
bool piscar_borda = false;     // Estado da borda do display (piscar ou não)
bool led_state = false;        // Estado do LED Verde (ligado/desligado)

// Definição dos pinos do joystick
#define analogicox 26          // Pino do eixo X do joystick (GPIO 26)
#define analogicoy 27          // Pino do eixo Y do joystick (GPIO 27)
#define botao_joystick 22      // Pino do botão do joystick (GPIO 22)

// Definições PWM
const uint16_t period = 4096;  // Período do PWM (12 bits)
const float divider_pwm = 16.0; // Divisor de frequência do PWM

// Variáveis globais
ssd1306_t ssd; // Objeto do display SSD1306

// Protótipos das funções
void led_init();               // Inicializa os LEDs RGB
void button_init();            // Inicializa os botões
void display_init();           // Inicializa o display SSD1306
void iniciar_adc();            // Inicializa o ADC
void setup_pwm();              // Configura o PWM para os LEDs
void debounce(uint gpio, uint32_t events); // Função de debouncing para os botões

// Função principal
int main()
{
    stdio_init_all();           // Inicializa a comunicação serial (para depuração)
    iniciar_adc();              // Inicializa o ADC
    led_init();                 // Inicializa os LEDs RGB
    setup_pwm();                // Configura o PWM para os LEDs
    button_init();              // Inicializa os botões
    uint16_t led_level_red = 0; // Nível de brilho do LED Vermelho
    uint16_t led_level_blue = 0; // Nível de brilho do LED Azul
    bool cor = true;            // Variável para alternar a cor da borda do display
    sleep_ms(1000);             // Aguarda 1 segundo para estabilização

    // Lê os valores centrais do joystick (posição neutra)
    adc_select_input(1);        // Seleciona o canal do eixo X
    uint16_t valor_centralX = adc_read(); // Lê o valor do eixo X

    adc_select_input(0);        // Seleciona o canal do eixo Y
    uint16_t valor_centralY = adc_read(); // Lê o valor do eixo Y

    display_init();             // Inicializa o display SSD1306
    uint8_t quadrado[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Define um quadrado de 8x8 pixels

    // Configura as interrupções para os botões
    gpio_set_irq_enabled_with_callback(buttonA, GPIO_IRQ_EDGE_FALL, true, &debounce);
    gpio_set_irq_enabled_with_callback(botao_joystick, GPIO_IRQ_EDGE_FALL, true, &debounce);

    // Loop principal
    while (true)
    {
        // Lê o valor do eixo X do joystick
        adc_select_input(1);
        uint16_t adcx_value = adc_read();
        led_level_red = adcx_value; // Atualiza o nível de brilho do LED Vermelho

        // Lê o valor do eixo Y do joystick
        adc_select_input(0);
        uint16_t adcy_value = period - adc_read(); // Inverte o valor para corresponder ao movimento do joystick
        led_level_blue = adcy_value; // Atualiza o nível de brilho do LED Azul

        // Verifica se o joystick está sendo movido no eixo X
        bool eixo_x = !buttonA_state && (adcx_value > valor_centralX + 250 || adcx_value < valor_centralX - 250);
        pwm_set_gpio_level(led_red, eixo_x ? led_level_red : 0); // Ajusta o brilho do LED Vermelho

        // Verifica se o joystick está sendo movido no eixo Y
        bool eixo_y = !buttonA_state && (adcy_value > valor_centralY + 250 || adcy_value < valor_centralY - 250);
        pwm_set_gpio_level(led_blue, eixo_y ? led_level_blue : 0); // Ajusta o brilho do LED Azul

        // Calcula a posição do quadrado no display com base nos valores do joystick
        int posicaoX = ((adcx_value * 125) / period) - 4; // Mapeia o valor do eixo X para a posição X no display
        int posicaoY = ((adcy_value * 64) / period) - 4;  // Mapeia o valor do eixo Y para a posição Y no display

        // Limita a posição do quadrado para evitar que ele saia dos limites do display
        if (posicaoX <= 0)
            posicaoX = 0 + 4;
        if (posicaoX >= 127 - 13)
            posicaoX = 127 - 13;
        if (posicaoY <= 0)
            posicaoY = 0 + 4;
        if (posicaoY >= 63 - 13)
            posicaoY = 63 - 13;

        // Atualiza o conteúdo do display com animações
        cor = !cor; // Alterna a cor da borda
        ssd1306_fill(&ssd, piscar_borda ? !cor : false); // Preenche o display com a cor atual

        // Desenha um retângulo na borda do display
        ssd1306_rect(&ssd, 3, 3, 122, 58, piscar_borda ? cor : true, piscar_borda ? !cor : false);

        // Desenha o quadrado na posição calculada
        ssd1306_draw(&ssd, quadrado, posicaoX, posicaoY);
        ssd1306_send_data(&ssd); // Envia os dados para o display

        sleep_ms(100); // Aguarda 100ms antes de atualizar novamente
    }
}

// Função para inicializar os LEDs RGB
void led_init()
{
    gpio_init(led_red);        // Inicializa o pino do LED Vermelho
    gpio_init(led_blue);       // Inicializa o pino do LED Azul
    gpio_init(led_green);      // Inicializa o pino do LED Verde
    gpio_set_dir(led_red, GPIO_OUT); // Configura o pino do LED Vermelho como saída
    gpio_set_dir(led_blue, GPIO_OUT); // Configura o pino do LED Azul como saída
    gpio_set_dir(led_green, GPIO_OUT); // Configura o pino do LED Verde como saída
    gpio_put(led_red, false);  // Desliga o LED Vermelho
    gpio_put(led_blue, false); // Desliga o LED Azul
    gpio_put(led_green, false); // Desliga o LED Verde
}

// Função para inicializar os botões
void button_init()
{
    gpio_init(buttonA);        // Inicializa o pino do botão A
    gpio_set_dir(buttonA, GPIO_IN); // Configura o pino do botão A como entrada
    gpio_pull_up(buttonA);     // Habilita resistor de pull-up no botão A

    gpio_init(botao_joystick); // Inicializa o pino do botão do joystick
    gpio_set_dir(botao_joystick, GPIO_IN); // Configura o pino do botão do joystick como entrada
    gpio_pull_up(botao_joystick); // Habilita resistor de pull-up no botão do joystick
}

// Função para inicializar o display SSD1306
void display_init()
{
    i2c_init(I2C_PORT, 400 * 1000); // Configura o I2C a 400kHz
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C); // Configura o pino SDA para função I2C
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C); // Configura o pino SCL para função I2C
    gpio_pull_up(PIN_I2C_SDA);      // Habilita pull-up no pino SDA
    gpio_pull_up(PIN_I2C_SCL);      // Habilita pull-up no pino SCL

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display SSD1306
    ssd1306_config(&ssd);         // Configura o display
    ssd1306_send_data(&ssd);      // Envia os dados iniciais para o display
}

// Função para inicializar o ADC
void iniciar_adc()
{
    adc_init();                   // Inicializa o ADC
    adc_gpio_init(analogicox);    // Configura o pino do eixo X do joystick como entrada analógica
    adc_gpio_init(analogicoy);    // Configura o pino do eixo Y do joystick como entrada analógica
}

// Função para configurar o PWM para os LEDs
void setup_pwm()
{
    // Configura o PWM para o LED Vermelho
    gpio_set_function(led_red, GPIO_FUNC_PWM); // Configura o pino do LED Vermelho para função PWM
    uint slice_red = pwm_gpio_to_slice_num(led_red); // Obtém o slice do PWM para o LED Vermelho
    pwm_set_wrap(slice_red, period); // Define o período do PWM
    pwm_set_clkdiv(slice_red, divider_pwm); // Define o divisor de frequência do PWM
    pwm_set_gpio_level(led_red, 0); // Define o nível inicial do PWM como 0 (LED desligado)
    pwm_set_enabled(slice_red, true); // Habilita o PWM para o LED Vermelho

    // Configura o PWM para o LED Azul
    gpio_set_function(led_blue, GPIO_FUNC_PWM); // Configura o pino do LED Azul para função PWM
    uint slice_blue = pwm_gpio_to_slice_num(led_blue); // Obtém o slice do PWM para o LED Azul
    pwm_set_wrap(slice_blue, period); // Define o período do PWM
    pwm_set_clkdiv(slice_blue, divider_pwm); // Define o divisor de frequência do PWM
    pwm_set_gpio_level(led_blue, 0); // Define o nível inicial do PWM como 0 (LED desligado)
    pwm_set_enabled(slice_blue, true); // Habilita o PWM para o LED Azul

    // Configura o PWM para o LED Verde
    gpio_set_function(led_green, GPIO_FUNC_PWM); // Configura o pino do LED Verde para função PWM
    uint slice_green = pwm_gpio_to_slice_num(led_green); // Obtém o slice do PWM para o LED Verde
    pwm_set_wrap(slice_green, period); // Define o período do PWM
    pwm_set_clkdiv(slice_green, divider_pwm); // Define o divisor de frequência do PWM
    pwm_set_gpio_level(led_green, 0); // Define o nível inicial do PWM como 0 (LED desligado)
    pwm_set_enabled(slice_green, true); // Habilita o PWM para o LED Verde
}

// Função de debouncing para os botões
void debounce(uint gpio, uint32_t events)
{
    static uint32_t last_time = 0; // Armazena o tempo da última interrupção
    uint32_t current_time = to_us_since_boot(get_absolute_time()); // Obtém o tempo atual em microssegundos

    // Verifica se o tempo desde a última interrupção é maior que 200ms (debouncing)
    if (current_time - last_time > 200000)
    {
        last_time = current_time; // Atualiza o tempo da última interrupção

        // Verifica qual botão foi pressionado
        if (gpio == buttonA)
        {
            buttonA_state = !buttonA_state; // Alterna o estado do botão A
        }
        if (gpio == botao_joystick)
        {
            led_state = !led_state; // Alterna o estado do LED Verde
            pwm_set_gpio_level(led_green, !led_state ? 0 : 50); // Ajusta o brilho do LED Verde

            piscar_borda = !piscar_borda; // Alterna o estado da borda do display
        }
    }
}