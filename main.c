#include <xc.h>
#include <stdint.h>
#pragma config WDTE=OFF, FOSC=XT, CPD=OFF, LVP=OFF
#define _XTAL_FREQ 4000000
#include <stdio.h>
//#include <htc.h>
#include "usart.h"


char bandera;           //Variable global indica que lleg� un caracter por el puerto serial
char recibido;          //caracter recibido
int inhabilitado;

/* A simple demonstration of serial communications which
 * incorporates the on-board hardware USART of the Microchip
 * PIC16Fxxx series of devices. */



// Pines del puente H
#define IN1 RD4
#define IN2 RD5
#define IN3 RD6
#define IN4 RD7

#define LED RD3


// === CONFIGURACIÓN SERVO ===
#define SERVO_TRIS TRISDbits.TRISD2
#define SERVO_PIN  PORTDbits.RD2

#define PULSE_US_0DEG   1000u
#define PULSE_US_90DEG  2000u
#define PERIOD_US       20000u




// Velocidades del PWM
int vel_giro = 28;
int vel_derecho = 20;
int vel_curva = 15;


unsigned int valorADC;
float voltaje;
const float UMBRAL = 2.0;  // Voltaje de referencia para encender el LED


// --- Funciones de control del servo ---
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

// --- Movimiento de servo 90°→0°→90° ---
void mover_servo(void)
{
    SERVO_TRIS = 0;
    SERVO_PIN = 0;

    // Configurar Timer1 (Fosc/4 = 1 MHz, 1 tick = 1 µs)
    T1CON = 0x00;
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 1;

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
}
//-------------------------------------------------------------



// --- Inicializaci�n del PWM ---
void PWM_Init() {
    // Configurar CCP1 y CCP2 como PWM
    TRISC2 = 0;   // CCP1 -> RC2 (EN1)
    TRISC1 = 0;   // CCP2 -> RC1 (EN2)

    T2CON=0x07;		// tmr2 on, pre divisor 16
    PR2 = 0xFF;   // Periodo PWM

    CCP1CON=0x0C;		//config. PWM ccp1	
    CCP2CON=0x0C;		// config. PWM ccp2

    // Ciclo inicial
    CCPR1L = 0x00;
    CCPR2L = 0x00;

}

// --- Configurar ciclo de trabajo del PWM ---
void PWM_SetDuty1_Porcentaje(int dutyPorcentaje) {
    if (dutyPorcentaje < 0) dutyPorcentaje = 0;     // Limite inferior
    if (dutyPorcentaje > 100) dutyPorcentaje = 100; // Limite superior

    // Escalar 0?100% a 0?255
    int valor = (dutyPorcentaje * 255) / 100;

    CCPR1L = valor;  // Cargar en registro de ciclo de trabajo
}
void PWM_SetDuty2_Porcentaje(int dutyPorcentaje) {
    if (dutyPorcentaje < 0) dutyPorcentaje = 0;     // Limite inferior
    if (dutyPorcentaje > 100) dutyPorcentaje = 100; // Limite superior

    // Escalar 0?100% a 0?255
    int valor = (dutyPorcentaje * 255) / 100;

    CCPR2L = valor;  // Cargar en registro de ciclo de trabajo
}

unsigned int leerADC(uint8_t canal) {
    if (canal > 7) return 0;  // Solo AN0?AN7 disponibles

    ADCON0 &= 0b11000101;         // Limpiar bits de selecci�n de canal
    ADCON0 |= (canal << 3);       // Establecer nuevo canal
    __delay_us(10);               // Peque�o tiempo de adquisici�n

    ADCON0bits.GO_nDONE = 1;      // Iniciar conversi�n
    while (ADCON0bits.GO_nDONE);  // Esperar a que termine

    return ((ADRESH << 8) + ADRESL);  // Retornar resultado (10 bits)
}




// --- Inicializaci�n general ---
void setup() {
    // Inicializar PWM
    PWM_Init();
    TMR2ON = 1;

    // Motores salida
    TRISD4 = 0;
    TRISD5 = 0;
    TRISD6 = 0;
    TRISD7 = 0;

    TRISC6 = 0;
    TRISC7 = 1;

    TRISD3 = 0; // LED

    TRISA0 = 1;
    



    ADCON1 = 0b10000000;  // ADFM=1, PCFG=0000
    ADCON0 = 0b01000001;  // Canal AN0 seleccionado (CHS=000), ADC encendido

    
    __delay_ms(10);
}

// --- Control motores ---
void motor_adelante() {
    PWM_SetDuty1_Porcentaje(vel_derecho);  
    PWM_SetDuty2_Porcentaje(vel_derecho);
    IN1 = 0; IN2 = 1;   // Motor izquierdo adelante
    IN3 = 1; IN4 = 0;   // Motor derecho adelante
}
void motor_atras(){
    PWM_SetDuty1_Porcentaje(vel_derecho);  
    PWM_SetDuty2_Porcentaje(vel_derecho);
    IN1 = 1; IN2 = 0;   // Motor izquierdo atras
    IN3 = 0; IN4 = 1;   // Motor derecho atras
}

void motor_giro_izquierda() {
    IN1 = 1; IN2 = 0;   // Motor izquierdo atr�s
    IN3 = 1; IN4 = 0;   // Motor derecho adelante
}

void motor_giro_derecha() {
    IN1 = 0; IN2 = 1;   // Motor izquierdo adelante
    IN3 = 0; IN4 = 1;   // Motor derecho atr�s
}

void motor_parar() {
    IN1 = 0; IN2 = 0;
    IN3 = 0; IN4 = 0;
}



void main(void){
    bandera=0;      //no lleg� datos
    recibido=0;     //se inicializa recibido
    GIE=1;          //Habilita interrupciones
    PEIE=1;         //Habilita interrupciones de perif�ricos
    RCIE=1;         //Habilita las interrupciones de datos recibidos por puerto serie
    setup();
    init_comms();	// set up the USART - settings defined in usart.h

    while(1){
    // Solo actualizar estado si el servo NO está en movimiento
        valorADC = leerADC(0);
        voltaje = (valorADC * 5.0) / 1023.0;

        if (voltaje > UMBRAL) {
            LED = 1;
            inhabilitado = 1;
            recibido = 'S'; // Fuerza detener motores
        } else {
            LED = 0;
            inhabilitado = 0;
        }
        


        // --- Si se recibió un dato ---
        if (bandera) {
            bandera = 0;  // limpiar bandera

            // 🔹 Permitir siempre mover el servo con 'X', incluso si está inhabilitado
            if (recibido == 'X') {
                mover_servo();   // Ejecutar movimiento de 90°→0°→90°
                continue;
            }

            // 🔹 Control de movimiento solo si no está inhabilitado
            if (!inhabilitado) {
                if (recibido == 'A') {
                    motor_adelante();
                } else if (recibido == 'R') {
                    motor_atras();
                } else if (recibido == 'I') {
                    PWM_SetDuty1_Porcentaje(vel_giro);
                    PWM_SetDuty2_Porcentaje(vel_giro);
                    motor_giro_izquierda();
                } else if (recibido == 'D') {
                    PWM_SetDuty1_Porcentaje(vel_giro);
                    PWM_SetDuty2_Porcentaje(vel_giro);
                    motor_giro_derecha();
                } else if (recibido == 'S') {
                    PWM_SetDuty1_Porcentaje(0);
                    PWM_SetDuty2_Porcentaje(0);
                    motor_parar();
                }
            } else {
                // Si está inhabilitado, siempre detener motores
                PWM_SetDuty1_Porcentaje(0);
                PWM_SetDuty2_Porcentaje(0);
                motor_parar();
            }
        }

        __delay_ms(50);
    }


}

void interrupt Interrupciones (void){
    if(RCIF){                   //si RCIF=1
        recibido=RCREG;         //lee dato recibido
        bandera=1;              //bandera indica dato recibido
    }

}