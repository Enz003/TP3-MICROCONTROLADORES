/*
 * Control de Servo MG996R con PIC16F877A usando Timer0
 * Mueve el servo a 90 grados y luego a 0 grados
 * Conexión: Servo conectado al pin RC0
 * Cristal: 20 MHz
 */

#include <xc.h>
#include <stdint.h>

// Configuración de bits
#pragma config FOSC = HS        // Oscilador HS (cristal externo)
#pragma config WDTE = OFF       // Watchdog Timer deshabilitado
#pragma config PWRTE = ON       // Power-up Timer habilitado
#pragma config BOREN = ON       // Brown-out Reset habilitado
#pragma config LVP = OFF        // Low Voltage Programming deshabilitado
#pragma config CPD = OFF        // Data EEPROM protección deshabilitada
#pragma config WRT = OFF        // Flash Program Memory Write deshabilitado
#pragma config CP = OFF         // Flash Program Memory Code Protection deshabilitado

#define _XTAL_FREQ 20000000     // Frecuencia del cristal: 20 MHz
#define SERVO_PIN PORTCbits.RC0 // Pin de salida para el servo

// Variables globales
volatile uint16_t pulso_alto = 1500;  // Ancho de pulso en microsegundos (inicial 90°)
volatile uint16_t contador_20ms = 0;   // Contador para periodo de 20ms
volatile uint8_t estado_pwm = 0;       // Estado del PWM (0=bajo, 1=alto)

// Función de interrupción
void __interrupt() ISR(void) {
    if (INTCONbits.TMR0IF) {  // Interrupción del Timer0
        INTCONbits.TMR0IF = 0; // Limpiar bandera
        
        if (estado_pwm == 1) {
            // Pulso alto completado, poner pin en bajo
            SERVO_PIN = 0;
            estado_pwm = 0;
            // Cargar timer para completar el periodo de 20ms
            TMR0 = 0;
        } else {
            contador_20ms++;
            // Cada 20ms generar un nuevo pulso
            if (contador_20ms >= 200) {  // Aproximadamente 20ms
                contador_20ms = 0;
                SERVO_PIN = 1;
                estado_pwm = 1;
            }
        }
    }
}

// Función para inicializar Timer0 y configuraciones
void Sistema_Init() {
    // Configurar puerto C
    TRISCbits.TRISC0 = 0;  // RC0 como salida
    SERVO_PIN = 0;          // Iniciar en bajo
    
    // Configurar Timer0
    // Prescaler 1:32
    OPTION_REGbits.T0CS = 0;   // Usar reloj interno
    OPTION_REGbits.PSA = 0;    // Asignar prescaler a Timer0
    OPTION_REGbits.PS = 0b100; // Prescaler 1:32
    
    TMR0 = 0;  // Limpiar Timer0
    
    // Habilitar interrupciones
    INTCONbits.TMR0IE = 1;  // Habilitar interrupción de Timer0
    INTCONbits.GIE = 1;     // Habilitar interrupciones globales
}

// Función para generar pulso PWM por software
void Generar_Pulso_Servo() {
    SERVO_PIN = 1;  // Poner pin en alto
    
    // Generar retardo del ancho de pulso
    for(uint16_t i = 0; i < pulso_alto; i++) {
        __delay_us(1);
    }
    
    SERVO_PIN = 0;  // Poner pin en bajo
}

// Función para mover el servo a un ángulo específico
// angulo: 0 a 180 grados
void Servo_SetAngulo(uint8_t angulo) {
    // Limitar el ángulo entre 0 y 180
    if (angulo > 180) angulo = 180;
    
    // Mapear el ángulo a microsegundos (1000us = 0°, 2000us = 180°)
    // pulso = 1000 + (angulo * 1000) / 180
    pulso_alto = 1000 + ((uint32_t)angulo * 1000) / 180;
}

// Función de retardo
void Delay_ms(uint16_t ms) {
    for(uint16_t i = 0; i < ms; i++) {
        __delay_ms(1);
    }
}

void main(void) {
    // Inicializar el sistema
    Sistema_Init();
    
    while(1) {
        // Mover el servo a 90 grados
        Servo_SetAngulo(90);
        
        // Mantener la posición enviando pulsos cada 20ms
        for(uint8_t i = 0; i < 100; i++) {  // Durante 2 segundos
            Generar_Pulso_Servo();
            Delay_ms(18);  // Completar periodo de 20ms (aproximado)
        }
        
        // Mover el servo a 0 grados
        Servo_SetAngulo(0);
        
        // Mantener la posición enviando pulsos cada 20ms
        for(uint8_t i = 0; i < 100; i++) {  // Durante 2 segundos
            Generar_Pulso_Servo();
            Delay_ms(18);  // Completar periodo de 20ms (aproximado)
        }
    }
}