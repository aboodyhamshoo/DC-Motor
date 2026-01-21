// Abdulrahman HAMCHO 62200061
// Hala RAHHAL 62200078
// Abdul Rahman KOUZAM 62210129

#include <xc.h>
#include <stdint.h>

// Configuration Bits
#pragma config FOSC = HS        // High-speed Crystal
#pragma config WDTE = OFF       // Watchdog Timer Disabled
#pragma config PWRTE = OFF      // Power-up Timer Disabled
#pragma config BOREN = OFF      // Brown-out Reset Disabled
#pragma config LVP = OFF        // Low-Voltage Programming Disabled
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection Disabled
#pragma config WRT = OFF        // Flash Program Memory Write Enable Disabled
#pragma config CP = OFF         // Flash Program Memory Code Protection Disabled

#define _XTAL_FREQ 8000000      // Required for __delay_ms()

// Function Prototypes
void ADC_Init();
uint16_t ADC_Read(uint8_t channel);
void PWM_Init();
void Set_PWM1_Duty(uint16_t duty);
void Set_PWM2_Duty(uint16_t duty);

void main() {
    // Port Configuration
    OPTION_REGbits.nRBPU = 0;   // Enable PORTB Pull-ups
    TRISB = 0x07;               // RB0, RB1, RB2 as inputs
    TRISC = 0x00;               // PORTC as outputs
    
    PORTB = 0x00;               // Initial state
    PORTC = 0x00;               // Initial state

    ADC_Init();
    PWM_Init();
    
    __delay_ms(100);
    
    uint16_t i;
    uint8_t dir1_active = 0;
    uint8_t dir2_active = 0;

    while(1) {
        i = ADC_Read(0);        // Read speed from AN0

        // Update PWM duty cycles if directions are active
        if(dir1_active) Set_PWM1_Duty(i);
        if(dir2_active) Set_PWM2_Duty(i);

        __delay_ms(10);

        // --- Direction 1 Button (RB0) ---
        if(PORTBbits.RB0 == 0) {
            if(!dir1_active) {
                PORTB = 0;              // LEDs off
                CCP1CON = 0x00;         // PWM1 Off
                CCP2CON = 0x00;         // PWM2 Off
                PORTC = 0;              // Stop motor
                __delay_ms(100);
                
                CCP1CON = 0x0C;         // Configure CCP1 as PWM
                PORTBbits.RB3 = 1;      // LED1 On
                dir1_active = 1;
                dir2_active = 0;
            }
        }

        // --- Direction 2 Button (RB1) ---
        if(PORTBbits.RB1 == 0) {
            if(!dir2_active) {
                PORTB = 0;              // LEDs off
                CCP1CON = 0x00;         // PWM1 Off
                CCP2CON = 0x00;         // PWM2 Off
                PORTC = 0;              // Stop motor
                __delay_ms(100);
                
                CCP2CON = 0x0C;         // Configure CCP2 as PWM
                PORTBbits.RB4 = 1;      // LED2 On
                dir2_active = 1;
                dir1_active = 0;
            }
        }

        // --- Stop Button (RB2) ---
        if(PORTBbits.RB2 == 0) {
            CCP1CON = 0x00;             // PWM1 Off
            CCP2CON = 0x00;             // PWM2 Off
            PORTC = 0;                  // All H-bridge pins Low
            PORTB = 0;                  // All LEDs off
            dir1_active = 0;
            dir2_active = 0;
        }
    }
}

// --- Peripheral Initialization Functions ---

void ADC_Init() {
    ADCON1 = 0x8E; // Right justified, AN0 analog, others digital, Fosc/32
    ADCON0 = 0x81; // Fosc/32, Channel 0, ADC On
}

uint16_t ADC_Read(uint8_t channel) {
    ADCON0 &= 0xC5;           // Clear channel selection bits
    ADCON0 |= (channel << 3); // Set channel
    __delay_ms(2);            // Acquisition time
    GO_nDONE = 1;             // Start conversion
    while(GO_nDONE);          // Wait for conversion to finish
    return ((ADRESH << 8) + ADRESL); // Return 10-bit result
}

void PWM_Init() {
    PR2 = 250;                // Sets PWM frequency to ~500Hz at 8MHz
    T2CON = 0x06;             // Timer 2 ON, Prescaler 1:16
    // CCP1CON and CCP2CON are handled in the main loop to toggle direction
}

void Set_PWM1_Duty(uint16_t duty) {
    CCPR1L = duty >> 2;                // 8 MSBs
    CCP1CONbits.CCP1X = (duty >> 1) & 1; // Bit 1
    CCP1CONbits.CCP1Y = duty & 1;        // Bit 0
}

void Set_PWM2_Duty(uint16_t duty) {
    CCPR2L = duty >> 2;                // 8 MSBs
    CCP2CONbits.CCP2X = (duty >> 1) & 1; // Bit 1
    CCP2CONbits.CCP2Y = duty & 1;        // Bit 0
}