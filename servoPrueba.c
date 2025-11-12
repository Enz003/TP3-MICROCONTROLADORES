/*
 * PWM por software con Timer1 (PIC16F877A)
 * Movimiento: inicia en 90°, baja a 0°, y vuelve a 90°
 */

#include <xc.h>
#include <stdint.h>

#pragma config FOSC = XT, WDTE = OFF, PWRTE = ON, BOREN = OFF, LVP = OFF, CPD = OFF, WRT = OFF, CP = OFF
#define _XTAL_FREQ 4000000UL

#define SERVO_TRIS TRISDbits.TRISD2
#define SERVO_PIN  PORTDbits.RD2

#define PULSE_US_0DEG   1000u
#define PULSE_US_90DEG  2000u
#define PERIOD_US       20000u

static inline uint16_t getTMR1(void)
{
    uint8_t h, l;
    l = TMR1L;
    h = TMR1H;
    return ((uint16_t)h << 8) | l;
}

static void wait_us(uint16_t delay_us)
{
    uint16_t start = getTMR1();
    while ((uint16_t)(getTMR1() - start) < delay_us);
}

static inline uint16_t angle_to_us(uint8_t ang)
{
    if (ang > 90) ang = 90; // límite a 90°
    return (uint16_t)(PULSE_US_0DEG + ((uint32_t)ang * (PULSE_US_90DEG - PULSE_US_0DEG) / 90));
}

void main(void)
{
    SERVO_TRIS = 0;
    SERVO_PIN = 0;

    T1CON = 0x00;
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 1;

    __delay_ms(50);

        // --- De 90° a 0° ---
        for (int16_t ang = 90; ang >= 0; ang--) {
            uint16_t pulse = angle_to_us((uint8_t)ang);
            SERVO_PIN = 1;
            wait_us(pulse);
            SERVO_PIN = 0;
            wait_us(PERIOD_US - pulse);
        }

        // --- Mantener en 0° un segundo ---
        for (uint8_t i = 0; i < 50; i++) {
            uint16_t pulse = angle_to_us(0);
            SERVO_PIN = 1;
            wait_us(pulse);
            SERVO_PIN = 0;
            wait_us(PERIOD_US - pulse);
        }

        // --- De 0° a 90° ---
        for (uint8_t ang = 0; ang <= 90; ang++) {
            uint16_t pulse = angle_to_us(ang);
            SERVO_PIN = 1;
            wait_us(pulse);
            SERVO_PIN = 0;
            wait_us(PERIOD_US - pulse);
        }

        // --- Mantener en 90° un segundo ---
        for (uint8_t i = 0; i < 50; i++) {
            uint16_t pulse = angle_to_us(90);
            SERVO_PIN = 1;
            wait_us(pulse);
            SERVO_PIN = 0;
            wait_us(PERIOD_US - pulse);
        }
    while(1){
    }

}
