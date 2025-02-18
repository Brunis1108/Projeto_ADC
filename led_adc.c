#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include <ctype.h>
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/adc.h"

// Definições para o display SSD1306 (comunicação I2C)
#define I2C_PORT i2c1
#define PIN_I2C_SDA 14
#define PIN_I2C_SCL 15
#define endereco 0x3C

// Definições dos pinos dos LEDs RGB
#define led_red 13
#define led_blue 12
#define led_green 11

// Definição do pino do botão A
#define buttonA 5
bool buttonA_state = false;
bool piscar_borda = false;
bool led_state = false;

// Definição dos pinos do joystick
#define analogicox 26
#define analogicoy 27
#define botao_joystick 22

// Definições pwm
const uint16_t period = 4096;
const float divider_pwm = 16.0;

// variaveis globais
ssd1306_t ssd; // Objeto do display SSD1306

// prototipos
void led_init();
void button_init();
void display_init();
void iniciar_adc();
void setup_pwm();
void debounce(uint gpio, uint32_t events);

// Função para inicializar os LEDs RGB
void led_init()
{
    gpio_init(led_red);
    gpio_init(led_blue);
    gpio_init(led_green);
    gpio_set_dir(led_red, GPIO_OUT);
    gpio_set_dir(led_blue, GPIO_OUT);
    gpio_set_dir(led_green, GPIO_OUT);
    gpio_put(led_red, false);
    gpio_put(led_blue, false);
    gpio_put(led_green, false);
}

void button_init()
{
    gpio_init(buttonA); // inicia o pino do botão A
    gpio_set_dir(buttonA, GPIO_IN);
    gpio_pull_up(buttonA); // Habilita resistor de pull-up no botão A

    gpio_init(botao_joystick); // inicia o pino do botão joystick
    gpio_set_dir(botao_joystick, GPIO_IN);
    gpio_pull_up(botao_joystick); // Habilita resistor de pull-up no botão joystick
}

void display_init()
{
    i2c_init(I2C_PORT, 400 * 1000);                // Configura o I2C a 400kHz
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C); // Configura o pino SDA
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C); // Configura o pino SCL
    gpio_pull_up(PIN_I2C_SDA);                     // Habilita pull-up no pino SDA
    gpio_pull_up(PIN_I2C_SCL);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
}

// Inicializa o ADC
void iniciar_adc()
{
    adc_init();
    adc_gpio_init(analogicox);

    adc_gpio_init(analogicoy);
}

void setup_pwm()
{
    // pwm para lede vermelho
    gpio_set_function(led_red, GPIO_FUNC_PWM);
    uint slice_red = pwm_gpio_to_slice_num(led_red);
    pwm_set_wrap(slice_red, period);
    pwm_set_clkdiv(slice_red, divider_pwm);
    pwm_set_gpio_level(led_red, 0);
    pwm_set_enabled(slice_red, true);

    // pwm para lede azul
    gpio_set_function(led_blue, GPIO_FUNC_PWM);
    uint slice_blue = pwm_gpio_to_slice_num(led_blue);
    pwm_set_wrap(slice_blue, period);
    pwm_set_clkdiv(slice_blue, divider_pwm);
    pwm_set_gpio_level(led_blue, 0);
    pwm_set_enabled(slice_blue, true);

    // pwm para lede verde
    gpio_set_function(led_green, GPIO_FUNC_PWM);
    uint slice_green = pwm_gpio_to_slice_num(led_green);
    pwm_set_wrap(slice_green, period);
    pwm_set_clkdiv(slice_green, divider_pwm);
    pwm_set_gpio_level(led_green, 0);
    pwm_set_enabled(slice_green, true);
}

void debounce(uint gpio, uint32_t events)
{
    static uint32_t last_time = 0; // Armazena o tempo da última interrupção
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    // Verifica se o tempo desde a última interrupção é maior que 200ms (debouncing)
    if (current_time - last_time > 200000)
    {
        last_time = current_time; // Atualiza o tempo da última interrupção

        // Verifica qual botão foi pressionado
        if (gpio == buttonA)
        {
            buttonA_state = !buttonA_state;
        }
        if (gpio == botao_joystick)
        {
            led_state = !led_state;
            pwm_set_gpio_level(led_green, !led_state ? 0 : 50);

            piscar_borda = !piscar_borda;
            
            
        }
    }
}

int main()
{
    stdio_init_all();
    iniciar_adc();
    led_init();
    setup_pwm();
    button_init();
    uint16_t led_level_red = 0;
    uint16_t led_level_blue = 0;
    bool cor = true;
    sleep_ms(1000);

    adc_select_input(1);
    uint16_t valor_centralX = adc_read();

    adc_select_input(0);
    uint16_t valor_centralY = adc_read();

    display_init();
    uint8_t quadrado[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    gpio_set_irq_enabled_with_callback(buttonA, GPIO_IRQ_EDGE_FALL, true, &debounce);
    gpio_set_irq_enabled_with_callback(botao_joystick, GPIO_IRQ_EDGE_FALL, true, &debounce);

    while (true)
    {
       

        adc_select_input(1);
        uint16_t adcx_value = adc_read();
        led_level_red = adcx_value;

        adc_select_input(0);
        uint16_t adcy_value = period - adc_read();
        led_level_blue = adcy_value;

        bool eixo_x = !buttonA_state && (adcx_value > valor_centralX + 250 || adcx_value < valor_centralX - 250);
        pwm_set_gpio_level(led_red, eixo_x ? led_level_red : 0);

        bool eixo_y = !buttonA_state && (adcy_value > valor_centralY + 250 || adcy_value < valor_centralY - 250);
        pwm_set_gpio_level(led_blue, eixo_y ? led_level_blue : 0);

        // dividir 4096 por resultado de 4096/128 eixo x
        // dividir 4096 por resultado de 4096/64 eixo y
       
        int posicaoX = ((adcx_value * 125) / period) - 4;
        int posicaoY = ((adcy_value * 64) / period) - 4;


        if (posicaoX <= 0)
            posicaoX = 0 + 4;
        if (posicaoX >= 127 - 13)
            posicaoX = 127 - 13;
        if (posicaoY <= 0)
            posicaoY = 0 + 4;
        if (posicaoY >= 63 - 13)
            posicaoY = 63 - 13;


        // Atualiza o conteúdo do display com animações
        cor =!cor;
        ssd1306_fill(&ssd, piscar_borda ? !cor : false );
        
        ssd1306_rect(&ssd, 3, 3, 122, 58, piscar_borda ? cor : true, piscar_borda ? !cor : false); // Desenha um retângulo

        ssd1306_draw(&ssd, quadrado, posicaoX, posicaoY);
        ssd1306_send_data(&ssd);

        sleep_ms(100);
    }
}
