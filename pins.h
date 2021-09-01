#ifndef _pins_h_
#define _pins_h_

void pinModeInput(const uint8_t, const bool = false);
void pinModeOutput(const uint8_t);
bool digitalReadExt(const uint8_t);
void digitalWriteExt(const uint8_t, const bool);



// #ifndef CRITICAL_SECTION_START
//   #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
//   #define CRITICAL_SECTION_END    SREG = _sreg;
// #endif //CRITICAL_SECTION_START
//
// #define DIO59_PIN		PINF5
// #define DIO59_RPORT	PINF
// #define DIO59_WPORT	PORTF
// #define DIO59_DDR		DDRF
// #define DIO59_PWM		NULL
//
// #define DIO61_PIN		PINF7
// #define DIO61_RPORT	PINF
// #define DIO61_WPORT	PORTF
// #define DIO61_DDR		DDRF
// #define DIO61_PWM		NULL
//
// #define DIO70_PIN		PING4
// #define DIO70_RPORT	PING
// #define DIO70_WPORT	PORTG
// #define DIO70_DDR		DDRG
// #define DIO70_PWM		NULL
//
// #define DIO71_PIN		PING3
// #define DIO71_RPORT	PING
// #define DIO71_WPORT	PORTG
// #define DIO71_DDR		DDRG
// #define DIO71_PWM		NULL
//
// #define DIO72_PIN		PINJ2
// #define DIO72_RPORT	PINJ
// #define DIO72_WPORT	PORTJ
// #define DIO72_DDR		DDRJ
// #define DIO72_PWM		NULL
//
// #define DIO73_PIN		PINJ3
// #define DIO73_RPORT	PINJ
// #define DIO73_WPORT	PORTJ
// #define DIO73_DDR		DDRJ
// #define DIO73_PWM		NULL
//
// #define DIO74_PIN		PINJ7
// #define DIO74_RPORT	PINJ
// #define DIO74_WPORT	PORTJ
// #define DIO74_DDR		DDRJ
// #define DIO74_PWM		NULL
//
// #define DIO75_PIN		PINJ4
// #define DIO75_RPORT	PINJ
// #define DIO75_WPORT	PORTJ
// #define DIO75_DDR		DDRJ
// #define DIO75_PWM		NULL
//
// #define DIO76_PIN		PINJ5
// #define DIO76_RPORT	PINJ
// #define DIO76_WPORT	PORTJ
// #define DIO76_DDR		DDRJ
// #define DIO76_PWM		NULL
//
// #define DIO77_PIN		PINJ6
// #define DIO77_RPORT	PINJ
// #define DIO77_WPORT	PORTJ
// #define DIO77_DDR		DDRJ
// #define DIO77_PWM		NULL
//
// #define DIO78_PIN		PINE2
// #define DIO78_RPORT	PINE
// #define DIO78_WPORT	PORTE
// #define DIO78_DDR		DDRE
// #define DIO78_PWM		NULL
//
// #define DIO79_PIN		PINE6
// #define DIO79_RPORT	PINE
// #define DIO79_WPORT	PORTE
// #define DIO79_DDR		DDRE
// #define DIO79_PWM		NULL
//
// #define DIO80_PIN		PINE7
// #define DIO80_RPORT	PINE
// #define DIO80_WPORT	PORTE
// #define DIO80_DDR		DDRE
// #define DIO80_PWM		NULL
//
// #define DIO81_PIN		PIND4
// #define DIO81_RPORT	PIND
// #define DIO81_WPORT	PORTD
// #define DIO81_DDR		DDRD
// #define DIO81_PWM		NULL
//
// #define DIO82_PIN		PIND5
// #define DIO82_RPORT	PIND
// #define DIO82_WPORT	PORTD
// #define DIO82_DDR		DDRD
// #define DIO82_PWM		NULL
//
// #define DIO83_PIN		PIND6
// #define DIO83_RPORT	PIND
// #define DIO83_WPORT	PORTD
// #define DIO83_DDR		DDRD
// #define DIO83_PWM		NULL
//
// #define DIO84_PIN		PINH2
// #define DIO84_RPORT	PINH
// #define DIO84_WPORT	PORTH
// #define DIO84_DDR		DDRH
// #define DIO84_PWM		NULL
//
// #define DIO85_PIN		PINH7
// #define DIO85_RPORT	PINH
// #define DIO85_WPORT	PORTH
// #define DIO85_DDR		DDRH
// #define DIO85_PWM		NULL
//
//
// #ifndef MASK
// /// MASKING- returns \f$2^PIN\f$
// #define MASK(PIN)  (1 << PIN)
// #endif
//
// #define _READ(IO) ((bool)(DIO ## IO ## _RPORT & MASK(DIO ## IO ## _PIN)))
// #define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }; } while (0)
// #define _WRITE_C(IO, v)   do { if (v) { \
//                                          CRITICAL_SECTION_START; \
//                                          {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); }\
//                                          CRITICAL_SECTION_END; \
//                                        }\
//                                        else {\
//                                          CRITICAL_SECTION_START; \
//                                          {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }\
//                                          CRITICAL_SECTION_END; \
//                                        }\
//                                      }\
//                                      while (0)
//
// #define _WRITE(IO, v)  do {  if (&(DIO ##  IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)
// #define	_SET_INPUT(IO) do {DIO ##  IO ## _DDR &= ~MASK(DIO ## IO ## _PIN); } while (0)
// #define	_SET_OUTPUT(IO) do {DIO ##  IO ## _DDR |=  MASK(DIO ## IO ## _PIN); } while (0)
// #define READ(IO)  _READ(IO)
// #define WRITE(IO, v)  _WRITE(IO, v)
// #define SET_INPUT(IO)  _SET_INPUT(IO)
// #define SET_OUTPUT(IO)  _SET_OUTPUT(IO)







// #define SWI2C_SDA      20 //SDA on P3
// #define SWI2C_SCL      21 //SCL on P3
//
//
#define X_TMC2130_CS           41
#define X_TMC2130_DIAG         64 // !!! changed from 40 (EINY03)
#define X_STEP_PIN             37
#define X_DIR_PIN              49
#define X_MIN_PIN            0 // 12
//#define X_MAX_PIN            30
//#define X_MIN_PIN              X_TMC2130_DIAG
#define X_MAX_PIN              X_TMC2130_DIAG
#define X_ENABLE_PIN           29
#define X_MS1_PIN           -1
#define X_MS2_PIN           -1
//
// #define Y_TMC2130_CS        39
// #define Y_TMC2130_DIAG      69
// #define Y_STEP_PIN          36
// #define Y_DIR_PIN           48
// #define Y_MIN_PIN           0 // 11
// //#define Y_MAX_PIN           24
// //#define Y_MIN_PIN           Y_TMC2130_DIAG
// #define Y_MAX_PIN           Y_TMC2130_DIAG
// #define Y_ENABLE_PIN        28
// #define Y_MS1_PIN           -1
// #define Y_MS2_PIN           -1
//
// #define Z_TMC2130_CS        67
// #define Z_TMC2130_DIAG      68
// #define Z_STEP_PIN          35
// #define Z_DIR_PIN           47
// #define Z_MIN_PIN           0 // 10
// #define Z_MAX_PIN           23
// //#define Z_MAX_PIN           Z_TMC2130_DIAG
// #define Z_ENABLE_PIN        27
// #define Z_MS1_PIN           -1
// #define Z_MS2_PIN           -1
//
// // #define RELAY_PIN   12
//
// #define HEATER_BED_PIN       4 //PG5
// #define TEMP_BED_PIN         2 //A2
//
// #define HEATER_0_PIN         3 // 12 // 3 //PE5
// #define TEMP_0_PIN           0 //A0
//
// #define HEATER_1_PIN        -1
// #define TEMP_1_PIN           1 //A1
//
// #define HEATER_2_PIN        -1
// #define TEMP_2_PIN          -1
//
// #define TEMP_AMBIENT_PIN     6 //A6
//
// #define TEMP_PINDA_PIN       3 //A3
//
// #define VOLT_PWR_PIN         4 //A4
// #define VOLT_BED_PIN         9 //A9
// #define VOLT_IR_PIN          8 //A8
//
//
// #define E0_TMC2130_CS       66
// #define E0_TMC2130_DIAG     65
// #define E0_STEP_PIN         34
// #define E0_DIR_PIN          43
// #define E0_ENABLE_PIN       26
// #define E0_MS1_PIN          -1
// #define E0_MS2_PIN          -1
//
// #define SDPOWER             -1
// #define SDSS                77
// #define LED_PIN             13
// #define FAN_PIN              6
// #define FAN_1_PIN           -1
// #define PS_ON_PIN           -1
// #define KILL_PIN            -1  // 80 with Smart Controller LCD
// #define SUICIDE_PIN         -1  // PIN that has to be turned on right after start, to keep power flowing.
//
//
// //#define KILL_PIN            32
//
// #define LCD_BL_PIN          5   //backlight control pin
#define LCD_PWM_PIN          5   //backlight control pin

// Beeper on AUX-4
#define BEEPER              84

#define LCD_PINS_RS         82
// !!! changed from 18 (EINY03)
#define LCD_PINS_ENABLE     61
// !!! changed from 19 (EINY03)
#define LCD_PINS_D4	        59
#define LCD_PINS_D5         70
#define LCD_PINS_D6         85
#define LCD_PINS_D7         71

// //buttons are directly attached using AUX-2
#define BTN_EN1                72
#define BTN_EN2                14
#define BTN_ENC                 9  // the click
//
// #define SDCARDDETECT           15
//
// #define TACH_0                 79 // !!! changed from 81 (EINY03)
// #define TACH_1                 80
//
// #define IR_SENSOR_PIN 62 //idler sensor @PK0 (digital pin 62/A8)
//
// // Support for an 8 bit logic analyzer, for example the Saleae.
// // Channels 0-2 are fast, they could generate 2.667Mhz waveform with a software loop.
// #define LOGIC_ANALYZER_CH0		X_MIN_PIN		// PB6
// #define LOGIC_ANALYZER_CH1		Y_MIN_PIN		// PB5
// #define LOGIC_ANALYZER_CH2		53				// PB0 (PROC_nCS)
// // Channels 3-7 are slow, they could generate
// // 0.889Mhz waveform with a software loop and interrupt locking,
// // 1.333MHz waveform without interrupt locking.
// #define LOGIC_ANALYZER_CH3 		73				// PJ3
// // PK0 has no Arduino digital pin assigned, so we set it directly.
// #define WRITE_LOGIC_ANALYZER_CH4(value) if (value) PORTK |= (1 << 0); else PORTK &= ~(1 << 0) // PK0
// #define LOGIC_ANALYZER_CH5		16				// PH0 (RXD2)
// #define LOGIC_ANALYZER_CH6		17				// PH1 (TXD2)
// #define LOGIC_ANALYZER_CH7 		76				// PJ5
//
// #define LOGIC_ANALYZER_CH0_ENABLE do { SET_OUTPUT(LOGIC_ANALYZER_CH0); WRITE(LOGIC_ANALYZER_CH0, false); } while (0)
// #define LOGIC_ANALYZER_CH1_ENABLE do { SET_OUTPUT(LOGIC_ANALYZER_CH1); WRITE(LOGIC_ANALYZER_CH1, false); } while (0)
// #define LOGIC_ANALYZER_CH2_ENABLE do { SET_OUTPUT(LOGIC_ANALYZER_CH2); WRITE(LOGIC_ANALYZER_CH2, false); } while (0)
// #define LOGIC_ANALYZER_CH3_ENABLE do { SET_OUTPUT(LOGIC_ANALYZER_CH3); WRITE(LOGIC_ANALYZER_CH3, false); } while (0)
// #define LOGIC_ANALYZER_CH4_ENABLE do { DDRK |= 1 << 0; WRITE_LOGIC_ANALYZER_CH4(false); } while (0)
// #define LOGIC_ANALYZER_CH5_ENABLE do { cbi(UCSR2B, TXEN2); cbi(UCSR2B, RXEN2); cbi(UCSR2B, RXCIE2); SET_OUTPUT(LOGIC_ANALYZER_CH5); WRITE(LOGIC_ANALYZER_CH5, false); } while (0)
// #define LOGIC_ANALYZER_CH6_ENABLE do { cbi(UCSR2B, TXEN2); cbi(UCSR2B, RXEN2); cbi(UCSR2B, RXCIE2); SET_OUTPUT(LOGIC_ANALYZER_CH6); WRITE(LOGIC_ANALYZER_CH6, false); } while (0)
// #define LOGIC_ANALYZER_CH7_ENABLE do { SET_OUTPUT(LOGIC_ANALYZER_CH7); WRITE(LOGIC_ANALYZER_CH7, false); } while (0)
//
// // Async output on channel 5 of the logical analyzer.
// // Baud rate 2MBit, 9 bits, 1 stop bit.
// #define LOGIC_ANALYZER_SERIAL_TX_ENABLE do { UBRR2H = 0; UBRR2L = 0; UCSR2B = (1 << TXEN2) | (1 << UCSZ02); UCSR2C = 0x06; } while (0)
// // Non-checked (quicker) variant. Use it if you are sure that the transmit buffer is already empty.
// #define LOGIC_ANALYZER_SERIAL_TX_WRITE_NC(C) do { if (C & 0x100) UCSR2B |= 1; else UCSR2B &= ~1; UDR2 = C; } while (0)
// #define LOGIC_ANALYZER_SERIAL_TX_WRITE(C) do { \
// 	/* Wait for empty transmit buffer */ \
// 	while (!(UCSR2A & (1<<UDRE2))); \
// 	/* Put data into buffer, sends the data */ \
// 	LOGIC_ANALYZER_SERIAL_TX_WRITE_NC(C); \
// } while (0)

#endif
