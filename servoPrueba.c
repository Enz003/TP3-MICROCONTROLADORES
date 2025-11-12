#include <xc.h>
#pragma config FOSC = XT, WDTE = OFF, PWRTE = ON, BOREN = OFF, LVP = OFF, CPD = OFF, WRT = OFF, CP = OFF
#define _XTAL_FREQ 4000000

// === CONSTANTES PARA EL SERVO ===
// (ajustá PW_0DEG y PW_90DEG si tu servo no llega al ángulo correcto)
#define PW_0DEG     20    // duración relativa del pulso (0°)
#define PW_90DEG    10    // duración relativa del pulso (90°)

unsigned char PW, PER;

void __interrupt() isr(void)
{
    if (TMR1IF)   // Interrupción del Timer1
    {
        // Reinicia Timer1 para próximo desborde (≈ cada 0.5 ms)
        TMR1H = 0xF8;     // valor alto (ajusta periodo)
        TMR1L = 0x2F;     // valor bajo  →  0xF82F = 63535 (0.5 ms aprox a 4 MHz)

        if (++PER == 0)
            RC0 = 1;      // inicio del pulso

        if (PER == PW)
            RC0 = 0;      // fin del pulso (control de ancho)

        TMR1IF = 0;       // limpia bandera
    }
}

void main(void)
{
    // === CONFIGURACIÓN DE PUERTOS ===
    TRISC0 = 0;
    RC0 = 0;

    // === CONFIGURACIÓN DE TIMER1 ===
    // Fuente interna (Fosc/4), prescaler 1:2 → cada incremento ≈ 2 µs
    T1CON = 0b00100001;   // TMR1ON=1, TMR1CS=0, T1CKPS1:T1CKPS0=01 (prescaler 1:2)
    TMR1H = 0xF8;
    TMR1L = 0x2F;
    TMR1IF = 0;
    TMR1IE = 1;
    PEIE = 1;
    GIE = 1;

    // === PROGRAMA PRINCIPAL ===
    PW = PW_0DEG;   // empieza en 0°
    PER = 0;

    __delay_ms(500);  // estabiliza

    // Mueve a 90°
    PW = PW_90DEG;
    __delay_ms(500);

    // Vuelve a 0°
    PW = PW_0DEG;
    __delay_ms(1000);

    // Queda detenido en 0°
    while (1)
    {
        // bucle infinito
    }
}
