#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>



/*
 * I²C (Two Wire) Communication
 */

// CONSTANTS
// used for communication protocol

#define TW_HOUSE_DEMO 0x20
#define TW_HOUSE_RGB 0x21
#define TW_HOUSE_R 0x22
#define TW_HOUSE_G 0x23
#define TW_HOUSE_B 0x24

#define TW_FERRIS_ENABLED 0x30

#define TW_METRO_DEMO 0x40
#define TW_METRO_1_DRIVE 0x41
#define TW_METRO_2_DRIVE 0x42

#define TW_DISABLE 0x00
#define TW_ENABLE 0x01

#define TW_INTO_STATION 0x00
#define TW_INTO_TUNNEL 0x01



// DATA

// Size of the TW receive buffer. Bytes beyond this limit will
// be discarded

#define TW_BUFFER_SIZE 4

volatile uint8_t g_tw_buffer [TW_BUFFER_SIZE];
volatile uint8_t g_tw_buffer_ix = 0;



// Bit field that stores which demo modes are enabled.

#define DEMO_HOUSE 0x01
#define DEMO_METRO 0x02

volatile uint8_t g_demo = 0xff;



/*
 * Hardware layout
 */

// Pin configuration

#define FERRIS_DDR DDRD
#define FERRIS_PORT PORTD
#define FERRIS_PORT_ENABLED PORTD4
#define FERRIS_PORT_ON PORTD5

// Town hall (house) LED PWM outputs
#define HOUSE_RED OCR0A
#define HOUSE_GREEN OCR2A
#define HOUSE_BLUE OCR2B

// Metro below the town hall
// sensor pins
#define METRO_1_PIN_DDR DDRC
#define METRO_1_PIN PINC
#define METRO_1_PIN_PORT PORTC
#define METRO_1_PIN_STATION PINC6 // station endstop
#define METRO_1_PIN_TUNNEL PINC7 // tunnel endstop

// Output motor ports
#define METRO_1_PORT_DDR DDRA
#define METRO_1_PORT PORTA
#define METRO_1_PORT_ENABLED PORTA1 // enable the motor driver
#define METRO_1_PORT_IN PORTA2 // drive train into the station
#define METRO_1_PORT_OUT PORTA3 // drive train out of the station

// Metro below the ferris wheel
#define METRO_2_PIN_DDR DDRC
#define METRO_2_PIN PINC
#define METRO_2_PIN_PORT PORTC
#define METRO_2_PIN_STATION PINC3
#define METRO_2_PIN_TUNNEL PINC2

#define METRO_2_PORT_DDR DDRA
#define METRO_2_PORT PORTA
#define METRO_2_PORT_ENABLED PORTA4
#define METRO_2_PORT_IN PORTA6
#define METRO_2_PORT_OUT PORTA5

// Internal constants for easier handling.
// Used for commands and bit field states.
#define METRO_STOP 0
#define METRO_IN_STATION 1
#define METRO_IN_TUNNEL 2
#define METRO_INTO_STATION 3
#define METRO_INTO_TUNNEL 4



// Each train will drive until it hits an endstop button. It
// further features a security timer. Each train will be shut
// off when the timer reaches zero.

// Maximum time after which each train will be shut off in
// milliseconds.
#define METRO_MAX_DRIVE_TIME_MILLIS 2000

volatile int16_t g_turn_off_motor_1_after_millis = 0;
volatile int16_t g_turn_off_motor_2_after_millis = 0;



/*
 * Returns a bit field describing the metros state.
 * It is either driving or stopped (METRO_STOP)
 * and it is at one of the endpoints or it is going
 * to one of the endpoints when driving.
 *
 * Possible bits set:
 * METRO_STOP
 * METRO_IN_STATION
 * METRO_IN_TUNNEL
 */
uint8_t
metro_1_state (void) {
    uint8_t d_state = 0x00;
    uint8_t d_pin = METRO_1_PIN;
    uint8_t d_port = METRO_1_PORT;

    if (d_port & (1 << METRO_1_PORT_ENABLED)) {
        // Train is currently powered == driving.
        // In this case it is on the move and therefore
        // in between stations and the target endpoint
        // is returned.
        if (d_port & (1 << METRO_1_PORT_IN)) {
            // It is moving into the station
            d_state |= (1 << METRO_IN_STATION);
        } else if (d_port & (1 << METRO_1_PORT_OUT)) {
            // It is moving into the tunnel
            d_state |= (1 << METRO_INTO_TUNNEL);
        } else {
            // It does not move at all
            d_state |= (1 << METRO_STOP);
        }
    } else {
        // Train is not powered -> it is stopped
        d_state |= (1 << METRO_STOP);
    }

    if (!(d_pin & (1 << METRO_1_PIN_STATION))) {
        // Station endstop is triggered
        d_state |= (1 << METRO_IN_STATION);
    }

    if (!(d_pin & (1 << METRO_1_PIN_TUNNEL))) {
        // Tunnel endstop is triggered
        d_state |= (1 << METRO_IN_TUNNEL);
    }

    return d_state;
}



/*
 * Control the movement of the first train.
 *
 * Possible commands are:
 * METRO_STOP
 * METRO_INTO_STATION
 * METRO_INTO_TUNNEL
 */
void
metro_1_control (uint8_t p_cmd) {
    uint8_t d_state = metro_1_state ();

    // reset all bits
    METRO_1_PORT
        &= ~( (1 << METRO_1_PORT_ENABLED)
            | (1 << METRO_1_PORT_IN)
            | (1 << METRO_1_PORT_OUT)
            );

    switch (p_cmd) {
        case METRO_STOP:
            break;

        case METRO_INTO_STATION:
            if (!(d_state & (1 << METRO_IN_STATION))) {
                METRO_1_PORT
                    |= (1 << METRO_1_PORT_ENABLED)
                    | (1 << METRO_1_PORT_IN)
                    ;

                g_turn_off_motor_1_after_millis = METRO_MAX_DRIVE_TIME_MILLIS;
            }

            break;

        case METRO_INTO_TUNNEL:
            if (!(d_state & (1 << METRO_IN_TUNNEL))) {
                METRO_1_PORT
                    |= (1 << METRO_1_PORT_ENABLED)
                    | (1 << METRO_1_PORT_OUT)
                    ;

                g_turn_off_motor_1_after_millis = METRO_MAX_DRIVE_TIME_MILLIS;
            }
            break;
    }
}



uint8_t
metro_2_state (void) {
    uint8_t d_state = 0x00;
    uint8_t d_pin = METRO_2_PIN;
    uint8_t d_port = METRO_2_PORT;

    if (d_port & (1 << METRO_2_PORT_ENABLED)) {
        if (d_port & (1 << METRO_2_PORT_IN)) {
            d_state |= (1 << METRO_IN_STATION);
        } else if (d_port & (1 << METRO_2_PORT_OUT)) {
            d_state |= (1 << METRO_INTO_TUNNEL);
        } else {
            d_state |= (1 << METRO_STOP);
        }
    } else {
        d_state |= (1 << METRO_STOP);
    }

    if (!(d_pin & (1 << METRO_2_PIN_STATION))) {
        d_state |= (1 << METRO_IN_STATION);
    }

    if (!(d_pin & (1 << METRO_2_PIN_TUNNEL))) {
        d_state |= (1 << METRO_IN_TUNNEL);
    }

    return d_state;
}



void
metro_2_control (uint8_t p_cmd) {
    uint8_t d_state = metro_2_state ();

    // reset all bits
    METRO_2_PORT
        &= ~( (1 << METRO_2_PORT_ENABLED)
            | (1 << METRO_2_PORT_IN)
            | (1 << METRO_2_PORT_OUT)
            );

    switch (p_cmd) {
        case METRO_STOP:
            break;

        case METRO_INTO_STATION:
            if (!(d_state & (1 << METRO_IN_STATION))) {
                METRO_2_PORT
                    |= (1 << METRO_2_PORT_ENABLED)
                    | (1 << METRO_2_PORT_IN)
                    ;

                g_turn_off_motor_2_after_millis = METRO_MAX_DRIVE_TIME_MILLIS;
            }

            break;

        case METRO_INTO_TUNNEL:
            if (!(d_state & (1 << METRO_IN_TUNNEL))) {
                METRO_2_PORT
                    |= (1 << METRO_2_PORT_ENABLED)
                    | (1 << METRO_2_PORT_OUT)
                    ;

                g_turn_off_motor_2_after_millis = METRO_MAX_DRIVE_TIME_MILLIS;
            }
            break;
    }
}



int
main (void) {
    // enable JTAG
    MCUCR = (1 << JTD);
    MCUCR = (1 << JTD);



    /* PORT SETUP */

    // set SS & MOSI & SCP to output
    DDRB
        |= (1 << PORTB4)
        | (1 << PORTB5)
        | (1 << PORTB7)
        ;



    // Town hall leds
    DDRB
        |= (1 << PORTB3)
        ;

    DDRD
        |= (1 << PORTD6)
        | (1 << PORTD7)
        ;



    FERRIS_DDR
        |= (1 << FERRIS_PORT_ENABLED)
        |  (1 << FERRIS_PORT_ON)
        ;



    /* METRO SETUP */

    // Metro ports
    METRO_1_PORT_DDR
        |= (1 << METRO_1_PORT_ENABLED)
        | (1 << METRO_1_PORT_IN)
        | (1 << METRO_1_PORT_OUT)
        ;

    METRO_2_PORT_DDR
        |= (1 << METRO_2_PORT_ENABLED)
        | (1 << METRO_2_PORT_IN)
        | (1 << METRO_2_PORT_OUT)
        ;



    // inputs
    METRO_1_PIN_DDR
        |= (0 << METRO_1_PIN_STATION)
        | (0 << METRO_1_PIN_TUNNEL)
        ;

    METRO_2_PIN_DDR
        |= (0 << METRO_2_PIN_STATION)
        | (0 << METRO_2_PIN_TUNNEL)
        ;

    // pull up
    METRO_1_PIN_PORT
        |= (1 << METRO_1_PIN_STATION)
        | (1 << METRO_1_PIN_TUNNEL)
        ;

    METRO_2_PIN_PORT
        |= (1 << METRO_2_PIN_STATION)
        | (1 << METRO_2_PIN_TUNNEL)
        ;



    // enable interrupt scan on portc
    PCICR
        = (1 << PCIE2);

    // enable interrupts for metro endstops
    PCMSK2
        = (1 << PCINT23)
        | (1 << PCINT22)
        | (1 << PCINT19)
        | (1 << PCINT18)
        ;



    /* TIMER */

    // set none-inverting mode
    TCCR1A
        // disable OCA0A pin
        = (0 << COM1A0)
        | (0 << COM1A1)

        // disable OCA0B pin
        | (0 << COM1B0)
        | (0 << COM1B1)

        // mode 12 interrupt model, compare with ICR1
        | (0 << WGM10)
        | (0 << WGM11)
        ;



    TCCR1B
        // mode 12 interrupt model, compare with ICR1
        = (1 << WGM12)
        | (1 << WGM13)

        // set prescale to 1
        | (1 << CS10)
        | (0 << CS11)
        | (0 << CS12)
        ;

    TCNT1 = 0;


    TIMSK1
        // activate CTC interrupt for mode 12
        = (1 << ICIE1)
        ;

    // interrupt every 20k cycles -> 1 milli second
    // used for metro safeguard shut down (METRO_MAX_DRIVE_TIME_MILLIS)
    ICR1 = 20000;



    /* Town hall PWM setup */

    TCCR0A
        = (1 << WGM00)
        | (1 << WGM01)

        | (0 << COM0A0)
        | (1 << COM0A1)
        ;

    TCCR0B
        = (1 << CS00)
        | (0 << CS01)
        | (0 << CS02)

        | (0 << WGM02)
        ;



    TCCR2A
        = (1 << WGM20)
        | (1 << WGM21)

        | (0 << COM2A0)
        | (1 << COM2A1)

        | (0 << COM2B0)
        | (1 << COM2B1)
        ;

    TCCR2B
        = (1 << CS20)
        | (0 << CS21)
        | (0 << CS22)

        | (0 << WGM22)
        ;



    /* I²C (Two Wire) Setup */

    // set 7 bit address on bits 7-1, bit 0 = TWGCE
    TWAR = 111 << 1;

    TWCR
        = (1 << TWEN) // enable TWI
        | (1 << TWEA) // enable ACK
        | (1 << TWIE) // enable TWI interrupts
        | (1 << TWINT) // clear interrput (INT) flag
        ;



    // turn on interrupts
    sei();



    /* INIT */

    // set town hall LEDs two red
    HOUSE_RED = 0xff;


    _delay_ms (2000);


    // drive both trains into station
    metro_1_control (METRO_INTO_TUNNEL);
    metro_2_control (METRO_INTO_TUNNEL);


    _delay_ms (5000);


    // start ferris wheel
    FERRIS_PORT |= (1 << FERRIS_PORT_ENABLED);
    FERRIS_PORT &= ~(1 << FERRIS_PORT_ON);


    // disable town hall LEDs
    HOUSE_RED = 0x00; // red color channel
    HOUSE_GREEN = 0x00; // green color channel
    HOUSE_BLUE = 0x00; // blue color channel



    while (1) {
        // move metro train
        if (g_demo & (1 << DEMO_METRO)) {
            metro_1_control (METRO_INTO_STATION);
        }

        // fade town hall color channels up and down
        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_RED < 255) {
                HOUSE_RED ++;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_BLUE > 0) {
                HOUSE_BLUE --;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_GREEN < 255) {
                HOUSE_GREEN ++;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_RED > 0) {
                HOUSE_RED --;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_METRO)) {
            metro_1_control (METRO_INTO_TUNNEL);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_BLUE < 255) {
                HOUSE_BLUE ++;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_GREEN > 0) {
                HOUSE_GREEN --;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

            /////////////

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_RED < 255) {
                HOUSE_RED ++;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_BLUE > 0) {
                HOUSE_BLUE --;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_METRO)) {
            metro_2_control (METRO_INTO_STATION);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_GREEN < 255) {
                HOUSE_GREEN ++;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_RED > 0) {
                HOUSE_RED --;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_BLUE < 255) {
                HOUSE_BLUE ++;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_GREEN > 0) {
                HOUSE_GREEN --;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_METRO)) {
            metro_2_control (METRO_INTO_TUNNEL);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_RED < 255) {
                HOUSE_RED ++;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_BLUE > 0) {
                HOUSE_BLUE --;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_GREEN < 255) {
                HOUSE_GREEN ++;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_RED > 0) {
                HOUSE_RED --;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_BLUE < 255) {
                HOUSE_BLUE ++;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }

        if (g_demo & (1 << DEMO_HOUSE)) {
            while (HOUSE_GREEN > 0) {
                HOUSE_GREEN --;
                _delay_ms (10);
            }
        } else {
            _delay_ms (2560);
        }
    }
}



/*
 * Metro endstop interrupts.
 * Stop a train whenever it hits an endstop button.
 */

// save PINC last state in here to determine what has changed
volatile uint8_t portc_history = 0xFF;

ISR (PCINT2_vect) {
    uint8_t changedbits;

    changedbits = PINC ^ portc_history;
    portc_history = PINC;

    // find the offending train and stop it
    if ( (changedbits & (1 << METRO_1_PIN_STATION))
            && !(PINC & (1 << METRO_1_PIN_STATION))
        ) {
        metro_1_control (METRO_STOP);
    }

    if ( (changedbits & (1 << METRO_1_PIN_TUNNEL))
            && !(PINC & (1 << METRO_1_PIN_TUNNEL))
        ) {
        metro_1_control (METRO_STOP);
    }

    if ( (changedbits & (1 << METRO_2_PIN_STATION))
            && !(PINC & (1 << METRO_2_PIN_STATION))
        ) {
        metro_2_control (METRO_STOP);
    }

    if ( (changedbits & (1 << METRO_2_PIN_TUNNEL))
            && !(PINC & (1 << METRO_2_PIN_TUNNEL))
        ) {
        metro_2_control (METRO_STOP);
    }
}



/*
 * Safeguard timer enforcing METRO_MAX_DRIVE_TIME_MILLIS.
 * This is triggered once every millisecond and counts down
 * the train safeguard timers.
 */
ISR (TIMER1_CAPT_vect) {
    if (g_turn_off_motor_1_after_millis <= 0) {
        // 0 reached -> turn the train off
        metro_1_control (METRO_STOP);
    } else {
        // count down one millisecond
        g_turn_off_motor_1_after_millis --;
    }

    if (g_turn_off_motor_2_after_millis <= 0) {
        metro_2_control (METRO_STOP);
    } else {
        g_turn_off_motor_2_after_millis --;
    }
}



/*
 * I²C interrupt vector
 * All I²C communication is handled here.
 */
ISR (TWI_vect) {
    uint8_t status;

    // Read TWI status register
    status = TWSR & 0xF8;

    switch (status) {
        // Data received & ack returned
        case 0x80:
        case 0x90:
            // write data to the buffer
            if (g_tw_buffer_ix >= 0 && g_tw_buffer_ix < TW_BUFFER_SIZE) {
                g_tw_buffer [g_tw_buffer_ix ++] = TWDR;
            }

            TWCR |= (1 << TWINT);

            break;

        // STOP/REPEATED START received
        case 0xA0:
            // transmission is over, reset buffer index
            g_tw_buffer_ix = 0;

            TWCR |= (1 << TWINT);

            // handle received command
            switch (g_tw_buffer [0]) {
                case TW_HOUSE_DEMO:
                    switch (g_tw_buffer [1]) {
                        case TW_DISABLE:
                            g_demo &= ~(1 << DEMO_HOUSE);
                            break;

                        case TW_ENABLE:
                            g_demo |= (1 << DEMO_HOUSE);
                            HOUSE_RED = 0xff;
                            HOUSE_GREEN = 0x00;
                            HOUSE_BLUE = 0x00;
                            break;
                    }
                    break;

                case TW_HOUSE_RGB:
                    HOUSE_RED = g_tw_buffer [1];
                    HOUSE_GREEN = g_tw_buffer [2];
                    HOUSE_BLUE = g_tw_buffer [3];
                    break;

                case TW_HOUSE_R:
                    HOUSE_RED = g_tw_buffer [1];
                    break;

                case TW_HOUSE_G:
                    HOUSE_GREEN = g_tw_buffer [1];
                    break;

                case TW_HOUSE_B:
                    HOUSE_BLUE = g_tw_buffer [1];
                    break;

                case TW_FERRIS_ENABLED:
                    if (g_tw_buffer [1] == 0x00) {
                        FERRIS_PORT |= (1 << FERRIS_PORT_ON);
                    } else {
                        FERRIS_PORT &= ~(1 << FERRIS_PORT_ON);
                    }

                    break;

                case TW_METRO_DEMO:
                    switch (g_tw_buffer [1]) {
                        case 0x00:
                            g_demo &= ~(1 << DEMO_METRO);
                            break;

                        case 0x01:
                            g_demo |= (1 << DEMO_METRO);
                            break;
                    }
                    break;

                case TW_METRO_1_DRIVE:
                    switch (g_tw_buffer [1]) {
                        case TW_INTO_STATION:
                            metro_1_control (METRO_INTO_STATION);
                            break;

                        case TW_INTO_TUNNEL:
                            metro_1_control (METRO_INTO_TUNNEL);
                            break;
                    }
                    break;

                case TW_METRO_2_DRIVE:
                    switch (g_tw_buffer [1]) {
                        case TW_INTO_STATION:
                            metro_2_control (METRO_INTO_STATION);
                            break;

                        case TW_INTO_TUNNEL:
                            metro_2_control (METRO_INTO_TUNNEL);
                            break;
                    }
                    break;
            }

            break;

        default:
            // Clear interrupt flag
            // This gives control back to the I²C processor
            TWCR |= (1 << TWINT);
    }
}
