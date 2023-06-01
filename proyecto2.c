/*
 * File:   Proyecto 2
 * Creado pro Estuardo Castillo 
 * Creado 30 de mayo de 2023
 * Programa para manejo de servo motores mediante 3 modos, ADC, EEPROM y Serial por EUSART 
 * Se fabrican dos PWM de forma manual para controlar un servo de 1 a 2 ms con un período de 20ms
 * Hardware: 4 potenciometros en AN<4:7> (RA6, RE<0:2>) 4 servomotores en RC<0:3> dos dip switch en RB0 y RB1
		pushbutton en RB2, cuatro leds en RA<0:3> FTDI en los puertos serial RC6, RC7. 	
 */


// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <stdint.h>
#include <xc.h>
#define _XTAL_FREQ 1000000
char data = 'A'; 
int modo;
int mensaje_ser;
int mensaje_completo = 0;
int servo_1, servo_2, servo_sel; 
int pulse_width_1, pulse_width_2 = 64936;
int toggle= 1;
int secuencia = 1;
int serial_send, eeprom_send = 0;
//------PROTOTIPOS DE FUNCIONES-----
void cadena(char *str);

void __interrupt() isr(void){
    //rutinas de  interrupción para generar pulsos en RC3 y RC0
        if(T0IF) {	            // TIMER0 Interrupt Flag
            if(toggle == 1){
                PORTCbits.RC3 = 1;
                TMR1H = 0xFE;
                TMR1L = pulse_width_1;
                toggle = 2;
            }else if(toggle == 2){
                PORTCbits.RC0 = 1;
                TMR1H = 0xFE;
                TMR1L = pulse_width_2;
                toggle = 1;
            }
            T1CONbits.TMR1ON = 1; //deja correr tmr1
            TMR0 = 216;             // Valor inicial para interrupciones cada 0.1ms
            T0IF = 0;	// se limpia bandera para siguiente int
        }
        if(TMR1IF){
            if(toggle == 2){
                PORTCbits.RC3 = 0;
                T1CONbits.TMR1ON = 0; //apaga tmr1 mientras no se ejecute y espera a siguiente int de tmr0
            }
            else if(toggle == 1){
                PORTCbits.RC0 = 0;
                T1CONbits.TMR1ON = 0; 
            }
            TMR1IF = 0; //se limpia bandera para siguiente int
        }
    if(PIR1bits.RCIF){
        if(secuencia == 1 && PIR1bits.RCIF){ //sigue solo si RCIF sigue en 1 y hay cola en FIFO
            mensaje_ser += ((int)(RCREG - 48))*100; //casteo a int
            secuencia = 2;
        }
        if(secuencia == 2 && PIR1bits.RCIF){ //sigue solo si RCIF sigue en 1 y hay cola en FIFO
            mensaje_ser += ((int)(RCREG - 48))*10;
            secuencia = 3;
            }
        if(secuencia == 3 && PIR1bits.RCIF){
            mensaje_ser += ((int)(RCREG - 48))*1;
            secuencia = 4;
        }
        if(secuencia == 4 && PIR1bits.RCIF){
            servo_sel = (RCREG);
            mensaje_completo = 1;
        }
        if(mensaje_completo == 1){
            switch(servo_sel){
                case '1':
                    servo_1 = mensaje_ser;
                    secuencia = 1; //reinicia secuencia
                    mensaje_completo = 0; //no permite que se ejecute bloque hasta que se complete otro mensaje
                    mensaje_ser = 0; //reiniciar una vez ya se copio al registro
                    break;
                case '2':
                    servo_2 = mensaje_ser;
                    secuencia = 1; //reinicia secuencia
                    mensaje_completo = 0; //no permite que se ejecute bloque hasta que se complete otro mensaje
                    mensaje_ser = 0; //reiniciar una vez ya se copio al registro
                    break; 
                case '3':
                    pulse_width_1 = mensaje_ser;
                    secuencia = 1; //reinicia secuencia
                    mensaje_completo = 0; //no permite que se ejecute bloque hasta que se complete otro mensaje
                    mensaje_ser = 0; //reiniciar una vez ya se copio al registro
                    break;
                case '4':
                    pulse_width_2 = mensaje_ser;
                    secuencia = 1; //reinicia secuencia
                    mensaje_completo = 0; //no permite que se ejecute bloque hasta que se complete otro mensaje
                    mensaje_ser = 0; //reiniciar una vez ya se copio al registro
                    break;
                default:
                    secuencia = 1; //un mensaje incorrecto de todas formas reinicia
                    mensaje_completo = 0;
                    TXREG = 'x';
                    break;
            }
        }   
    }
    //interrupcion por cambio en B
    if(INTCONbits.RBIF){
        if(PORTBbits.RB2){
            //PORTA = 1;
            eeprom_send = 1;
            //PORTD = ~PORTD;
        }
        else if(PORTBbits.RB0 == 0 && PORTBbits.RB1 ==0){
            modo = 1;
            PORTA = 0b0001;
        }
        else if(PORTBbits.RB0 == 1 && PORTBbits.RB1 ==0){
            modo = 2;
            PORTA = 0b0010;
        }
        else if(PORTBbits.RB0 == 0 && PORTBbits.RB1 ==1){
            modo = 3;
            PORTA = 0b0100;
        }
        else if(PORTBbits.RB0 == 1 && PORTBbits.RB1 ==1){
            modo = 4;
            PORTA = 0b1000;
        }
        INTCONbits.RBIF = 0;
    }
    //interrupcion de ADC
}

void setup(void){
    //configuracion IO
    ANSEL = 0b11110000;
    ANSELH = 0;
    
    TRISE = 0b0111;
    PORTE = 0;
    
    TRISD = 0;
    PORTD = 0x00;
    
    TRISA = 0x10;
    PORTA = 0x00;
    //configuracion puertos B 
    TRISB = 0b0111;
    PORTB = 0;
    
    //posicion inicial en valores guardados de eeprom
    servo_1 = Read_EEPROM(0x01);
    __delay_ms(10);
    servo_2 = Read_EEPROM(0x01);
    __delay_ms(10);
    pulse_width_1 = Read_EEPROM(0x03);
    __delay_ms(10);
    pulse_width_2 = Read_EEPROM(0x04);
    __delay_ms(10);
    //configuracion para interrupciones, WPUB y IOCB de puerto B
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB = 0b0011;
    //interrupciones en cambio de estado en B
    INTCONbits.RBIE = 1;
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB1 = 1;
    IOCBbits.IOCB2 = 1;
    
    //configuracion reloj
    OSCCONbits.IRCF = 0b100;
    OSCCONbits.SCS = 1;
    //configuracion de ADC
    ADCON1bits.ADFM = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    
    ADCON0bits.ADCS = 0b00;
    ADCON0bits.CHS = 0b0101;
    ADCON0bits.ADON = 1;
    __delay_us(50);
    
    //configuraciones para PWM
        //entradas para PWM con duty cycle en RC2 y RC1
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1;
    CCP1CONbits.P1M = 0;
    CCP1CONbits.CCP1M = 0b1100;
    CCP2CONbits.CCP2M = 0b1100;
        //valores iniciales
    CCPR1L = 0x01;
    CCP1CONbits.DC1B = 0;
    CCPR2L = 0x01;
    CCP2CONbits.DC2B1 = 0;
    CCP2CONbits.DC2B0 = 0;
      //periodo del PWM para CCPX con TMR2
    PR2 = 255;
    T2CONbits.T2CKPS = 0b10;
    T2CONbits.TMR2ON = 1;
    PIR1bits.TMR2IF = 0;
    while(PIR1bits.TMR2IF ==0);
    PIR1bits.TMR2IF = 0;
        //clear para salidas 
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    //TMR0
    //Timer0 Registers Prescaler= 1 - TMR0 Preset = 243 - Freq = 9615.38 Hz - Period = 0.000104 seconds
    OPTION_REGbits.T0CS = 0;  // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;  // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;   // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the WDT
    OPTION_REGbits.PS2 = 1;   // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 1;
    TMR0 = 216;             // preset for timer register
    //TMR1
    //Timer1 Registers Prescaler= 1 - TMR1 Preset = 63313 - Freq = 112.46 Hz - Period = 0.008892 seconds
    T1CONbits.T1CKPS1 = 0;   // bits 5-4  Prescaler Rate Select bits
    T1CONbits.T1CKPS0 = 0;   // bit 4
    T1CONbits.T1OSCEN = 0;   // bit 3 Timer1 Oscillator Enable Control bit 1 = on
    T1CONbits.T1SYNC = 1;    // bit 2 Timer1 External Clock Input Synchronization Control bit...1 = Do not synchronize external clock input
    T1CONbits.TMR1CS = 0;    // bit 1 Timer1 Clock Source Select bit...0 = Internal clock (FOSC/4)
    T1CONbits.TMR1ON = 1;    // bit 0 enables timer
    TMR1H = 247;             // preset for timer1 MSB register
    TMR1L = 81; 
    //TX y RX
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    
    BAUDCTLbits.BRG16 = 1;
    //calculo con anexo de manual:
    SPBRG = 25;
    SPBRGH = 0;
    
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    
    TXSTAbits.TXEN = 1;
    //configuracion interrupcion
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 1;
    INTCONbits.TMR0IE = 1;        // bit5 TMR0 Overflow Interrupt Enable bit...1 = Enables the TMR0 interrupt
    INTCONbits.TMR0IF = 0;        // bit2 clear timer 0 interrupt flag
    PIR1bits.TMR1IF = 0;            // clear timer1 interupt flag TMR1IF
    PIE1bits.TMR1IE  =   1;         // enable Timer1 interrupts
    PIR1bits.ADIF = 0;
    RBIF = 0;
    RCIF = 0;
    //perifericas y globales
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    TRISC = 0xF0;
    PORTC = 0;
    return;
}
unsigned int map(unsigned int au32_IN, unsigned int au32_INmin, unsigned int au32_INmax, unsigned int au32_OUTmin, unsigned int au32_OUTmax){
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

unsigned int lectura_ADC(unsigned char canal) {
    ADCON0bits.CHS = canal; //canal para conversion
    __delay_us(50);
    // Iniciar conversion ADC
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO);
    //__delay_us(100);  
    return ADRESH; //regresar valor convertido
    //return conversion; //descomentar para 10 bits
}
int absv(int num) {
    if (num < 0) {
        return -num;
    } else {
        return num;
    }
}
int servoWrite(int value, int duty_cycle_prev, int selector){
    int lower_limit; 
    int upper_limit;
    if(selector == 1){
        lower_limit = 55;
        upper_limit = 140;
    }else{
        lower_limit = 45;
        upper_limit = 165;
    }
    int value_map = map(value, 0, 255, lower_limit, upper_limit);
    int duty_cycle = (char)(value_map);
    int diferencia = 1;
    int diferencia_abs = absv(diferencia);
        while(diferencia_abs>0){
            if(diferencia<0){
                duty_cycle_prev--;
            }else{
                duty_cycle_prev++;
            }
            switch(selector){
                case 1:
                    CCPR1L = duty_cycle_prev>>2;
                    CCP1CON = CCP1CON&(0xCF);
                    CCP1CON = (CCP1CON |(0x30&(duty_cycle_prev>>4)));
                    diferencia_abs--;
                    __delay_us(20);
                    break;
                case 2:
                    CCPR2L = duty_cycle_prev>>2;
                    CCP2CON = CCP2CON&(0xCF);
                    CCP2CON = (CCP2CON |(0x30&(duty_cycle_prev>>4)));
                    diferencia_abs--;
                    __delay_us(20);
                    break;
                default:
                    break;
            }
        }
    duty_cycle_prev = duty_cycle;
    //__delay_ms(20);     // Delay for servo response time
    return duty_cycle_prev;
}
int conversion(int adc_val, int* centenas, int* decenas, int* unidades){
    *centenas= adc_val/100;
    int residuo_cen = adc_val%100;
    *decenas = residuo_cen/10;
    int residuo_dec = adc_val%10;
    *unidades = residuo_dec;
    return 0;
}
int ser_out (int selector, int adc_val){
    int centenas, decenas, unidades;
    char str[5] = "xaaa";
    conversion(adc_val, &centenas, &decenas, &unidades);
    *(str+3) = (char)(unidades+48);
    *(str+2) = (char)(decenas+48);
    *(str+1) = (char)(centenas+48); 
    *(str) = (char)(selector+48);
    int done = 0;
    int i = 0;
    while(done ==0){
        while(PIR1bits.TXIF){
            if (str[i] != '\0'){
                TXREG = str[i];
            }else {
                //TXREG = 13;
                done = 1;
            }
            i++;
        }
  }
    //convertir bytes leidos de 0 a 255 en strings con el formato axxx/
    //donde a es el slider que recibe pyhton y xxx el valor de 0-255 '/' terminador
    
}
int Read_EEPROM(int address){
    EEADR = address;
    EECON1bits.EEPGD =0;
    EECON1bits.RD = 1;
    return EEDAT;
}
void write_EEPROM(int data, int address){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;
    
    INTCONbits.GIE = 0;
    INTCONbits.PEIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;
    
    EECON1bits.WREN = 0;
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1; //rehabilitar interrupciones 
    INTCONbits.PEIE = 1;
}
void main(void) {
    //loop indefinido
    setup();
    int duty_cycle_prev_1, duty_cycle_prev_2 = 44;
    unsigned int adc_value_1, adc_value_2, adc_value_3, adc_value_4;
    RBIF = 1; //llamar una vez para saber en que modo estan los switches
    while(1){
        switch(modo){
            case 1:
                RCSTAbits.CREN = 0; //deshabilitar la recepcion de datos
                //lectura ADC
                adc_value_1 = lectura_ADC(6);
                adc_value_2 = lectura_ADC(7);
                adc_value_3 = lectura_ADC(5);
                adc_value_4 = lectura_ADC(4);
                //escribir servos
                duty_cycle_prev_1 = servoWrite(adc_value_1, duty_cycle_prev_1, 1);
                duty_cycle_prev_2 = servoWrite(adc_value_2, duty_cycle_prev_2, 2);
                servo_1 = adc_value_1;
                servo_2 = adc_value_2;
                pulse_width_1 = adc_value_3;
                pulse_width_2 = adc_value_4;
                if(eeprom_send){
                    write_EEPROM(adc_value_1, 0x01);
                    __delay_ms(10);
                    PORTD = ~PORTD;
                    write_EEPROM(adc_value_2, 0x02);
                    __delay_ms(10);
                    PORTD = ~PORTD;
                    write_EEPROM(adc_value_3, 0x03);
                    __delay_ms(10);
                    PORTD = ~PORTD;
                    write_EEPROM(adc_value_4, 0x04);
                    __delay_ms(10);
                    PORTD = ~PORTD;
                    eeprom_send = 0;
                }
                break;
            case 2:
                //recupera los valores guardados en EEPROM
                RCSTAbits.CREN = 0; //deshabilitar la recepcion de datos
                servo_1 = Read_EEPROM(0x01);
                __delay_ms(10);
                servo_2 = Read_EEPROM(0x02);
                __delay_ms(10);
                pulse_width_1 = Read_EEPROM(0x03);
                __delay_ms(10);
                pulse_width_2 = Read_EEPROM(0x04);
                __delay_ms(10);
                while(modo == 2){
                    duty_cycle_prev_1 = servoWrite(servo_1, duty_cycle_prev_1, 1);
                    duty_cycle_prev_2 = servoWrite(servo_2, duty_cycle_prev_2, 2);
                }
                break;
            case 3: 
                //cargar valores actuaales a PC
                ser_out(1, servo_1);
                ser_out(2, servo_2);
                ser_out(3, pulse_width_1);
                ser_out(4, pulse_width_2);
                __delay_ms(100); //delay para no saturar entrada en PC
                RCSTAbits.CREN = 1; //habilitar la recepcion de datos
                //bloque de escritura desde PC
                    while(modo == 3){
                        PORTD = pulse_width_1;
                        duty_cycle_prev_1 = servoWrite(servo_1, duty_cycle_prev_1, 1);
                        duty_cycle_prev_2 = servoWrite(servo_2, duty_cycle_prev_2, 2);
                        if(eeprom_send){
                            RCSTAbits.CREN = 0;
                            write_EEPROM(servo_1, 0x01);
                            __delay_ms(10);
                            PORTD = ~PORTD;
                            write_EEPROM(servo_2, 0x02);
                            __delay_ms(10);
                            PORTD = ~PORTD;
                            write_EEPROM(pulse_width_1, 0x03);
                            __delay_ms(10);
                            PORTD = ~PORTD;
                            write_EEPROM(pulse_width_2, 0x04);
                            __delay_ms(10);
                            PORTD = ~PORTD;
                            eeprom_send = 0;
                            RCSTAbits.CREN = 1;
                        }
                    }
                break;
            default: 
                break;
        }
    }    
    return;
}