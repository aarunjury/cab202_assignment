#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

// --== WIRING ==--
// LCD GND  -> GND
// LCD VCC  -> 5V
// LCD V0   -> GND
// LCD RW   -> GND
// LCD LED Anode    -> 220 Ohm -> 5V
// LCD LED Cathode  -> GND

// Change the values in these defines to reflect 
// how you've hooked up the screen
// In 4-pin mode only DATA4:7 are used

#define LCD_USING_4PIN_MODE (1)

// #define LCD_DATA0_DDR (DDRD)
// #define LCD_DATA1_DDR (DDRD)
// #define LCD_DATA2_DDR (DDRD)
// #define LCD_DATA3_DDR (DDRD)
#define LCD_DATA4_DDR (DDRD)
#define LCD_DATA5_DDR (DDRD)
#define LCD_DATA6_DDR (DDRD)
#define LCD_DATA7_DDR (DDRD)


// #define LCD_DATA0_PORT (PORTD)
// #define LCD_DATA1_PORT (PORTD)
// #define LCD_DATA2_PORT (PORTD)
// #define LCD_DATA3_PORT (PORTD)
#define LCD_DATA4_PORT (PORTD)
#define LCD_DATA5_PORT (PORTD)
#define LCD_DATA6_PORT (PORTD)
#define LCD_DATA7_PORT (PORTD)

// #define LCD_DATA0_PIN (0)
// #define LCD_DATA1_PIN (1)
// #define LCD_DATA2_PIN (2)
// #define LCD_DATA3_PIN (3)
#define LCD_DATA4_PIN (2)
#define LCD_DATA5_PIN (3)
#define LCD_DATA6_PIN (4)
#define LCD_DATA7_PIN (5)


#define LCD_RS_DDR (DDRD)
#define LCD_ENABLE_DDR (DDRD)

#define LCD_RS_PORT (PORTD)
#define LCD_ENABLE_PORT (PORTD)

#define LCD_RS_PIN (7)
#define LCD_ENABLE_PIN (6)

//DATASHEET: https://s3-us-west-1.amazonaws.com/123d-circuits-datasheets/
//uploads%2F1431564901240-mni4g6oo875bfbt9-6492779e35179defaf4482c7ac4f9915%2FLCD-WH1602B-TMI.pdf

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

void lcd_init(void);
void lcd_write_string(uint8_t x, uint8_t y, char string[]);
void lcd_write_char(uint8_t x, uint8_t y, char val);


void lcd_clear(void);
void lcd_home(void);

void lcd_createChar(uint8_t, uint8_t[]);
void lcd_setCursor(uint8_t, uint8_t); 

void lcd_noDisplay(void);
void lcd_display(void);
void lcd_noBlink(void);
void lcd_blink(void);
void lcd_noCursor(void);
void lcd_cursor(void);
void lcd_leftToRight(void);
void lcd_rightToLeft(void);
void lcd_autoscroll(void);
void lcd_noAutoscroll(void);
void scrollDisplayLeft(void);
void scrollDisplayRight(void);

size_t lcd_write(uint8_t);
void lcd_command(uint8_t);

void lcd_send(uint8_t, uint8_t);
void lcd_write4bits(uint8_t);
void lcd_write8bits(uint8_t);
void lcd_pulseEnable(void);

uint8_t _lcd_displayfunction;
uint8_t _lcd_displaycontrol;
uint8_t _lcd_displaymode;
/* ********************************************/
// START LIBRARY FUNCTIOMNS

void lcd_init(void){
  //dotsize
  if (LCD_USING_4PIN_MODE){
    _lcd_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  } else {
    _lcd_displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
  }
  
  _lcd_displayfunction |= LCD_2LINE;

  // RS Pin
  LCD_RS_DDR |= (1 << LCD_RS_PIN);
  // Enable Pin
  LCD_ENABLE_DDR |= (1 << LCD_ENABLE_PIN);
  
  #if LCD_USING_4PIN_MODE
    //Set DDR for all the data pins
    LCD_DATA4_DDR |= (1 << 4);
    LCD_DATA5_DDR |= (1 << 5);
    LCD_DATA6_DDR |= (1 << 6);    
    LCD_DATA7_DDR |= (1 << 7);

  #else
    //Set DDR for all the data pins
    LCD_DATA0_DDR |= (1 << LCD_DATA0_PIN);
    LCD_DATA1_DDR |= (1 << LCD_DATA1_PIN);
    LCD_DATA2_DDR |= (1 << LCD_DATA2_PIN);
    LCD_DATA3_DDR |= (1 << LCD_DATA3_PIN);
    LCD_DATA4_DDR |= (1 << LCD_DATA4_PIN);
    LCD_DATA5_DDR |= (1 << LCD_DATA5_PIN);
    LCD_DATA6_DDR |= (1 << LCD_DATA6_PIN);
    LCD_DATA7_DDR |= (1 << LCD_DATA7_PIN);
  #endif 

  // SEE PAGE 45/46 OF Hitachi HD44780 DATASHEET FOR INITIALIZATION SPECIFICATION!

  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  _delay_us(50000); 
  // Now we pull both RS and Enable low to begin commands (R/W is wired to ground)
  LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  
  //put the LCD into 4 bit or 8 bit mode
  if (LCD_USING_4PIN_MODE) {
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms

    // second try
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms
    
    // third go!
    lcd_write4bits(0b0111); 
    _delay_us(150);

    // finally, set to 4-bit interface
    lcd_write4bits(0b0010); 
  } else {
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(4500);  // wait more than 4.1ms

    // second try
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(150);

    // third go
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
  }

  // finally, set # lines, font size, etc.
  lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);  

  // turn the display on with no cursor or blinking default
  _lcd_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  lcd_display();

  // clear it off
  lcd_clear();

  // Initialize to default text direction (for romance languages)
  _lcd_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}


/********** high level commands, for the user! */
void lcd_write_string(uint8_t x, uint8_t y, char string[]){
  lcd_setCursor(x,y);
  for(int i=0; string[i]!='\0'; ++i){
    lcd_write(string[i]);
  }
}

void lcd_write_char(uint8_t x, uint8_t y, char val){
  lcd_setCursor(x,y);
  lcd_write(val);
}

void lcd_clear(void){
  lcd_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  _delay_us(2000);  // this command takes a long time!
}

void lcd_home(void){
  lcd_command(LCD_RETURNHOME);  // set cursor position to zero
  _delay_us(2000);  // this command takes a long time!
}


// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  lcd_command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    lcd_write(charmap[i]);
  }
}


void lcd_setCursor(uint8_t col, uint8_t row){
  if ( row >= 2 ) {
    row = 1;
  }
  
  lcd_command(LCD_SETDDRAMADDR | (col + row*0x40));
}

// Turn the display on/off (quickly)
void lcd_noDisplay(void) {
  _lcd_displaycontrol &= ~LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_display(void) {
  _lcd_displaycontrol |= LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor(void) {
  _lcd_displaycontrol &= ~LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_cursor(void) {
  _lcd_displaycontrol |= LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink(void) {
  _lcd_displaycontrol &= ~LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_blink(void) {
  _lcd_displaycontrol |= LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// These commands scroll the display without changing the RAM
void scrollDisplayLeft(void) {
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void scrollDisplayRight(void) {
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_leftToRight(void) {
  _lcd_displaymode |= LCD_ENTRYLEFT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft(void) {
  _lcd_displaymode &= ~LCD_ENTRYLEFT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This will 'right justify' text from the cursor
void lcd_autoscroll(void) {
  _lcd_displaymode |= LCD_ENTRYSHIFTINCREMENT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This will 'left justify' text from the cursor
void lcd_noAutoscroll(void) {
  _lcd_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

/*********** mid level commands, for sending data/cmds */

inline void lcd_command(uint8_t value) {
  //
  lcd_send(value, 0);
}

inline size_t lcd_write(uint8_t value) {
  lcd_send(value, 1);
  return 1; // assume sucess
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void lcd_send(uint8_t value, uint8_t mode) {
  //RS Pin
  LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
  LCD_RS_PORT |= (!!mode << LCD_RS_PIN);

  if (LCD_USING_4PIN_MODE) {
    lcd_write4bits(value>>4);
    lcd_write4bits(value);
  } else {
    lcd_write8bits(value); 
  } 
}

void lcd_pulseEnable(void) {
  //Enable Pin
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  _delay_us(1);    
  LCD_ENABLE_PORT |= (1 << LCD_ENABLE_PIN);
  _delay_us(1);    // enable pulse must be >450ns
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  _delay_us(100);   // commands need > 37us to settle
}

void lcd_write4bits(uint8_t value) {
  //Set each wire one at a time

  LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
  LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
  value >>= 1;

  LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
  LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
  value >>= 1;

  LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
  LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
  value >>= 1;

  LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
  LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);

  lcd_pulseEnable();
}

void lcd_write8bits(uint8_t value) {
  //Set each wire one at a time

  #if !LCD_USING_4PIN_MODE
    LCD_DATA0_PORT &= ~(1 << LCD_DATA0_PIN);
    LCD_DATA0_PORT |= ((value & 1) << LCD_DATA0_PIN);
    value >>= 1;

    LCD_DATA1_PORT &= ~(1 << LCD_DATA1_PIN);
    LCD_DATA1_PORT |= ((value & 1) << LCD_DATA1_PIN);
    value >>= 1;

    LCD_DATA2_PORT &= ~(1 << LCD_DATA2_PIN);
    LCD_DATA2_PORT |= ((value & 1) << LCD_DATA2_PIN);
    value >>= 1;

    LCD_DATA3_PORT &= ~(1 << LCD_DATA3_PIN);
    LCD_DATA3_PORT |= ((value & 1) << LCD_DATA3_PIN);
    value >>= 1;

    LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
    LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
    value >>= 1;

    LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
    LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
    value >>= 1;

    LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
    LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
    value >>= 1;

    LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
    LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);
    
    lcd_pulseEnable();
  #endif
}

#define SET_BIT(reg, pin)           (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin)         (reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)  (reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)         (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)        (BIT_VALUE((reg),(pin))==1)
#define FREQ 16000000.0
#define PRESCALE 8.0
#define DURATION 15624;
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
//threshold used for debouncing 
#define THRESHOLD (1000)

/*  ****** Global Variable Declarations ****** */
//For converting cm to a string
char buff[20];
//For uart_printf
char buffer[50];
double cm = 170;
double led_threshold = 150;
int led_brightness = 1;
volatile uint32_t overflow_counter = 0;
volatile uint32_t piezo_counter = 0;
volatile uint8_t switch_closed = 0;
volatile uint8_t switch_counter = 0;

/*  ****** Function Declarations ****** */
//UART Functions
void uart_put_byte(unsigned char data);
void uart_putstring(unsigned char* s);
void uart_printf(const char * fmt, ...);
void uart_init(unsigned int ubrr);
void uart_putchar(unsigned char data);
unsigned char uart_getchar(void);
//String Related Functions
void reverse(char* str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char* res, int afterpoint);
//ISR
ISR (TIMER0_OVF_vect);
ISR (TIMER1_COMPA_vect);
//Ultrasonic Sensor Functions
void trigger_pulse();
void pulse_mode();
//Button Press Function
void button_pressed();
void button_press_loop();
//Adjust Brightness with PWM Function
void adjust_brightness();
void adjust_led();
//ADC Functions
void set_threshold();
uint16_t adc_read(uint8_t channel);
void adc_init();
//Timer Related Functions
double msToCm(double seconds);
double get_current_time();
//Setup
void setup();

/*  ****** MAIN ****** */
int main() {
  setup();
  lcd_init();
  while (1){
    adjust_led();
    button_pressed();
    adjust_brightness();
    set_threshold();
  	trigger_pulse();
  	pulse_mode();
  	_delay_ms(250);
  }
  return 0;
}

/*  ****** SETUP ****** */
void setup() {
  uart_init(MYUBRR);
  //Set up button for output on PB2
  CLEAR_BIT(DDRB, 2);
  //Set up RGB LED (Green PB3, Red PB4)
  //and piezo buzzer (PB5)
  SET_BIT (DDRB, 3);
  SET_BIT (DDRB, 4);
  SET_BIT (DDRB, 5);
  //Set Timer 0 for Ultrasonic Sensor Pulse Measurement
  TCCR0A = 0;
  TCCR0B = (1 << WGM01);
  TIMSK0 = (1 << TOIE1);
  //Timer 1 Set up for Piezo
  OCR1A = DURATION;
  TCCR1B |= (1 << WGM12);
  TIMSK1 |= (1 << OCIE1A);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  //Set Timer 2 for PWM for LED Brightness
  TCCR2A |= (1 << COM2A1);
  TCCR2B = (1 << CS21);
  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  //Setup ADC for the Potentiometer
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADMUX = (1 << REFS0);
  //Enable Interrupts
  sei();
}

/*  ****** UART TEXT FUNCTIONS ****** */
//The following 3 functions are credited to Luis Mejias
//https://blackboard.qut.edu.au/bbcswebdav/pid-8394135-dt-content-rid-31013118_1/courses/CAB202_20se1/Topic08/CAB202-Topic8-Notes.html
void uart_put_byte(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0))) { /* Wait */ }
    UDR0 = data;
}

void uart_putstring(unsigned char* s)
{
    // transmit character until NULL is reached
    while(*s > 0) uart_putchar(*s++);
}

void uart_printf(const char * fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    for (int i = 0; buffer[i]; i++) {
        uart_put_byte(buffer[i]);
    }
}

/*  ****** INTERRUPTS ****** */
//ISR for Timer 0. This serves 2 purposes:
//For measuring length of ultrasonic pulse return time
//and also for interrupt-based debouncing of the switch
ISR(TIMER0_OVF_vect){
    overflow_counter++;
    uint8_t mask = 0b00111111;

    switch_counter = ((switch_counter<<1) & mask) | BIT_VALUE (PINB, 2);
    if (switch_counter == 0){
        switch_closed = 0;
    }
    else if (switch_counter == mask){
        switch_closed = 1;
    }

    TCNT0 = 0;
}

//ISR for Timer 1 for piezo buzzer timer to count number of seconds
//the distance threshold is breached
ISR (TIMER1_COMPA_vect){
    piezo_counter++;
    TCNT1 = 0;
}

/*  ****** ULTRASONIC SENSOR FUNCTIONS ****** */
//This function pulses the Ultrasonic sensor according to spec:
//(To get a clean read, send a quick high pulse, followed by low
//followed by a longer high pulse)
void trigger_pulse(){
  SET_BIT (DDRB, 0);
  CLEAR_BIT(PORTB,0);
  _delay_us(2);
  SET_BIT(PORTB,0);
  _delay_us(5);
  CLEAR_BIT(PORTB,0);
}

void pulse_mode(){
  double time_before, elapsed_time;

   //Set Ultrasonic sensor pin to input
   CLEAR_BIT (DDRB, 0);
  
   while (!BIT_VALUE(PINB, 0)){}
  
   time_before = get_current_time();
  
   while (BIT_VALUE(PINB, 0)){}
  
   elapsed_time = get_current_time() - time_before;
  
   cm = msToCm(elapsed_time);
   ftoa(cm, buff, 1);
   //Can uncomment out the next line to see on the serial monitor
   //the elapsed time in ticks for debugging
   //uart_printf("%d \n", elapsed_time);
   lcd_write_string(0,0, "Distance:");
   lcd_write_string(11,0, buff);
   lcd_write_string(14,0, "cm");
}

/*  ****** ADC FUNCTION ****** */
//The following function is credited to Luis Mejias
//https://blackboard.qut.edu.au/bbcswebdav/pid-8394134-dt-content-rid-31389818_1/courses/CAB202_20se1/Topic10/CAB202-Topic10-Notes.html
void set_threshold(){
char temp_buff[64];
   // Start single conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1 << ADSC);
   // Wait for ADSC bit to clear, signalling conversion complete.
	 while ( ADCSRA & (1 << ADSC) ) {}
  	for (int i = 0; i < sizeof(temp_buff); i++){
      temp_buff[i] = 0;
  }
	//Result now available.
	uint16_t pot = ADC;
    // Divide pot by 4 to give a range in cm to adjust the threshold by
    led_threshold = pot / 4;
  	ftoa(led_threshold, temp_buff, 1);
  	lcd_write_string(0,1, "Threshold:");
  	lcd_write_string(11,1, temp_buff);
  	lcd_write_string(14,1, "cm");
}

/*  ****** LED BRIGHTNESS PWM ROUTINE ****** */
void adjust_brightness(){
  if (led_brightness == 1){
    //50% duty cycle
    OCR2A = 128;
  }
  else if (led_brightness == 2){
    //75% duty cycle
    OCR2A = 192;
  }
  else if (led_brightness  == 3){
    //99.6% duty cycle (near enough to 100% for our intents and purposes)
    OCR2A = 255;
  }
  else{
    //Only three stages of brightness so
    //reset the counter back to 1
    led_brightness = 1;
  }
}

void adjust_led(){
double seconds;
  // if distance in cm is greater than threshold, then we are good
  if (cm > led_threshold){//set green LED and clear red LED
    SET_BIT (DDRB, 3);
    SET_BIT (PORTB, 3);
    CLEAR_BIT (PORTB, 4);
    //Stop Piezo when threshold is not breached
    CLEAR_BIT (PORTB, 5);
    //While the threshold is not breached, we don't want the piezo timer to be counting either
    piezo_counter = 0;
  }
  //then cm must be < led_threshold, red LED on, green off and start reading timer for Piezo     
  else{
    SET_BIT (PORTB, 4);
    CLEAR_BIT (PORTB, 3);
    CLEAR_BIT (DDRB, 3);
    //Sound the alarm if the threshold is breached longer than
    //approx. 10 seconds
    if (piezo_counter >= 10){
        SET_BIT(PORTB, 5);
      }
  }
}

/*  ****** BUTTON PRESS DETECTION ****** */
//Adapted from AMS 9.3 but the answers from which are my own work
void button_pressed(){
uint8_t prevState = 0;
  if (switch_closed != prevState) {
    prevState = switch_closed;
    //This is included for debugging, as the Tinkercad simulation runs so slowly,
    //this is useful to confirm the switch is working
    uart_printf("Switch is %s.\r\n",  prevState ? "closed" : "open");
    led_brightness++; 
  }
}

/*  ****** CONVERT TIME TO DISTANCE ****** */
double msToCm(double seconds) {
  // The speed of sound is 343 m/s.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the totaldistance travelled. TinkerCad also introduces a 
  // margin of error which is compensated by about 5.9% 
  return 343 * seconds / 2 * 100 * 1.059;
}

/*  ****** GET TIME ****** */
//Adapated from AMS Exercise 9.2 (the solution for which was my own work)
double get_current_time(){
    double time = (overflow_counter * 256.0 + TCNT0) * PRESCALE / FREQ;
    return time;
}

/*  ****** MORE UART FUNCTIONS ****** */
//The following 6 functions are credited to Luis Mejias
//https://blackboard.qut.edu.au/bbcswebdav/pid-8394135-dt-content-rid-31013118_1/courses/CAB202_20se1/Topic08/CAB202-Topic8-Notes.html
// Initialize the UART
void uart_init(unsigned int ubrr){
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)(ubrr);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C =(3 << UCSZ00);
    }

//transmit data
void uart_putchar(unsigned char data){
   while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/
     UDR0 = data;            /* Put data into buffer, sends the data */
}

//receive data
unsigned char uart_getchar(void){  
  /* Wait for data to be received */ 
    while ( !(UCSR0A & (1<<RXC0)) );  
  return UDR0;	
}

/*  ****** STRING CONVERSION FUNCTIONS ****** */
// Reverses a string 'str' of length 'len' 
void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 
  
// Converts a given integer x to string str[].  
// d is the number of digits required in the output.  
// If d is more than the number of digits in x,  
// then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
// Converts a floating-point/double number to a string. 
void ftoa(float n, char* res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) { 
        res[i] = '.'; // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter  
        // is needed to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
}