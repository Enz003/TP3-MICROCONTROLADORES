/*
 * Control de Servo MG996R con PIC16F877A usando Timer0
 * Mueve el servo a 90 grados y luego a 0 grados
 * Mantiene cada posición por el tiempo especificado
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
volatile uint16_t pulso_us = 1500;     // Ancho de pulso en microsegundos
volatile uint16_t contador_ms = 0;      // Contador de milisegundos
volatile uint8_t flag_20ms = 0;         // Bandera para generar pulso cada 20ms

// Configuración del Timer0 para generar interrupciones cada ~1ms
// Con Fosc=20MHz, Prescaler=256, TMR0=131 -> ~1ms
#define TMR0_RELOAD 131

// Función de interrupción
void __interrupt() ISR(void) {
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;  // Limpiar bandera
        TMR0 = TMR0_RELOAD;      // Recargar Timer0
        
        contador_ms++;
        
        // Cada 20ms activar bandera para generar pulso
        if (contador_ms >= 20) {
            contador_ms = 0;
            flag_20ms = 1;
        }
    }
}

// Función para inicializar Timer0 y configuraciones
void Sistema_Init() {
    // Configurar puerto C
    TRISCbits.TRISC0 = 0;  // RC0 como salida
    SERVO_PIN = 0;          // Iniciar en bajo
    
    // Configurar Timer0
    OPTION_REGbits.T0CS = 0;    // Usar reloj interno (Fosc/4)
    OPTION_REGbits.PSA = 0;     // Asignar prescaler a Timer0
    OPTION_REGbits.PS = 0b111;  // Prescaler 1:256
    
    TMR0 = TMR0_RELOAD;  // Cargar valor inicial
    
    // Habilitar interrupciones
    INTCONbits.TMR0IE = 1;  // Habilitar interrupción de Timer0
    INTCONbits.GIE = 1;     // Habilitar interrupciones globales
}

// Función para generar un pulso PWM
void Generar_Pulso() {
    SERVO_PIN = 1;  // Poner pin en alto
    
    // Generar retardo del ancho de pulso en microsegundos
    uint16_t tiempo = pulso_us;
    while(tiempo > 0) {
        __delay_us(1);
        tiempo--;
    }
    
    SERVO_PIN = 0;  // Poner pin en bajo
}

// Función para establecer el ángulo del servo
void Servo_SetAngulo(uint8_t angulo) {
    // Limitar el ángulo entre 0 y 180
    if (angulo > 180) angulo = 180;
    
    // Mapear el ángulo a microsegundos (1000us = 0°, 2000us = 180°)
    pulso_us = 1000 + ((uint32_t)angulo * 1000) / 180;
}

// Función para mantener el servo en una posición por X segundos
void Servo_Mantener(uint8_t angulo, uint8_t segundos) {
    // Establecer el ángulo deseado
    Servo_SetAngulo(angulo);
    
    // Calcular cuántos pulsos necesitamos (50 pulsos por segundo a 20ms cada uno)
    uint16_t pulsos_totales = segundos * 50;
    
    // Generar los pulsos
    for(uint16_t i = 0; i < pulsos_totales; i++) {
        // Esperar la bandera de 20ms
        while(flag_20ms == 0);
        flag_20ms = 0;
        
        // Generar el pulso
        Generar_Pulso();
    }
}

void main(void) {
    // Inicializar el sistema
    Sistema_Init();
    
    while(1) {
        // Mover a 90 grados y mantener por 2 segundos
        Servo_Mantener(90, 2);
        
        // Mover a 0 grados y mantener por 2 segundos
        Servo_Mantener(0, 2);
    }
}