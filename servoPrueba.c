/*
 * PWM por software fiable para servos usando Timer1 (PIC16F877A)
 * - Usa Timer1 como contador libre a 1 MHz -> 1 tick = 1 us
 * - Genera pulsos de 1..2 ms cada 20 ms (50 Hz)
 * - No usa interrupciones (polling sobre TMR1), evita saltos de encendido
 *
 * Requisitos hardware:
 * - Alimentación del servo aparte y GND común con el PIC
 * - Señal al pin RC0 (no alimentar el servo desde el pin)
 */

#include <xc.h>
#include <stdint.h>

#pragma config FOSC = XT, WDTE = OFF, PWRTE = ON, BOREN = OFF, LVP = OFF, CPD = OFF, WRT = OFF, CP = OFF
#define _XTAL_FREQ 4000000UL

#define SERVO_TRIS TRISCbits.TRISC0
#define SERVO_PIN  PORTCbits.RC0

// Ajustá estos valores si tu servo necesita otro rango
#define PULSE_US_0DEG   1000u   // 1000 us -> 0°
#define PULSE_US_90DEG  2000u   // 2000 us -> 90°
#define PERIOD_US       20000u  // 20 ms

// Lectura segura de TMR1 (lectura de alta y baja)
static inline uint16_t getTMR1(void)
{
    uint8_t h, l;
    // lectura: primero TMR1L luego TMR1H en PIC, pero aseguramos consistencia
    // deshabilitar interrupciones no es necesario aquí porque no usamos ISRs
    l = TMR1L;
    h = TMR1H;
    return ((uint16_t)h << 8) | l;
}

// Espera 'delay_us' microsegundos usando Timer1 (delay_us up to 65535)
static void wait_us(uint16_t delay_us)
{
    uint16_t start = getTMR1();
    while ((uint16_t)(getTMR1() - start) < delay_us) {
        // spin
    }
}

// Convierte ángulo (0..180) a microsegundos (lineal)
static inline uint16_t angle_to_us(uint8_t ang)
{
    if (ang > 180) ang = 180;
    // mapa: 0 -> PULSE_US_0DEG, 180 -> PULSE_US_90DEG*2?? (si quieres 0..90 usar otro rango)
    // Para tu caso específico 0..90: mapear 0..90 a PULSE_US_0DEG..PULSE_US_90DEG
    // Si quieres usar 0..90, el cálculo sería con 90 en lugar de 180.
    return (uint16_t)(PULSE_US_0DEG + ((uint32_t)ang * (PULSE_US_90DEG - PULSE_US_0DEG) / 90));
}

void main(void)
{
    // Configurar RC0 como salida y forzar a 0 antes de generar cualquier pulso
    SERVO_TRIS = 0;   // RC0 output
    SERVO_PIN = 0;

    // Inicializar Timer1: Fosc/4 -> 1 MHz, prescaler 1:1, Timer1 ON
    T1CON = 0x00;     // TMR1CS=0 (Fosc/4), T1CKPS=00 (1:1), TMR1ON=0 por ahora
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 1; // iniciar Timer1

    // Pequeña espera para estabilizar alimentación
    __delay_ms(50);

    // Ejemplo de uso: barrido 0° -> 90° -> 0° 

        // Subir de 0 a 90 grados
    for (uint8_t ang = 0; ang <= 90; ang++) {
        uint16_t pulse = angle_to_us(ang); // microsegundos

        // Inicio del periodo: generar pulso
        SERVO_PIN = 1;
        wait_us(pulse);             // mantener el pulso
        SERVO_PIN = 0;

        // esperar hasta completar 20 ms total
        wait_us(PERIOD_US - pulse);

        // opcional: cambio de velocidad, para suavizar podés agregar pequeñas esperas
    }

    // Espera en 90° un segundo
    for (uint8_t i = 0; i < 50; i++) {
        // mantener posición: enviar un pulso de mantenimiento cada 20 ms
        uint16_t pulse = angle_to_us(90);
        SERVO_PIN = 1;
        wait_us(pulse);
        SERVO_PIN = 0;
        wait_us(PERIOD_US - pulse);
    }

    // Bajar de 90 a 0
    for (int16_t ang = 90; ang >= 0; ang--) {
        uint16_t pulse = angle_to_us((uint8_t)ang);
        SERVO_PIN = 1;
        wait_us(pulse);
        SERVO_PIN = 0;
        wait_us(PERIOD_US - pulse);
    }
    while (1)
    {
    }
    

    
}