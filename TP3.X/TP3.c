#include <xc.h>
#define _XTAL_FREQ 4000000
#pragma config FOSC = XT, WDTE = OFF, PWRTE = ON, BOREN = ON, LVP = OFF, CPD = OFF, WRT = OFF, CP = OFF

// === VARIABLES GLOBALES ===
int periodo = 0;
int ancho = 20; // 6 interrupciones ? 1.5 ms ? 90°

void __interrupt() ISR(void) {
    if (T0IF) {
        TMR0 = 156;    // reinicia Timer0 (cada overflow ? 0.256 ms)
        T0IF = 0;    // limpia bandera
        periodo++;

        if (periodo == 200)
            periodo = 0;
            RC1 = 1; // comienza el pulso
        if (periodo == ancho)
            RC1 = 0; // termina el pulso

    }
}

void main(void) {
    TRISC1 = 0;  // RC1 salida (conectado al pin de señal del servo)
    RC1 = 0;

    OPTION_REG = 0b00001000; // PSA=1 ? sin prescaler
    TMR0 = 0;
    T0IF = 0;
    T0IE = 1;
    PEIE = 1;
    GIE = 1;

    while (1) {
        // Servo fijo en 90°
        // (Si querés probar otros ángulos, cambiá 'ancho' a 4 o 8)
        ancho = 20; // centro (90°)
    }
}
