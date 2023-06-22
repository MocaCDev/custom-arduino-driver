#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <util/delay.h>
#include <string.h>
#include <avr/eeprom.h>

#pragma warning( push )
#pragma warning( disable : 4100 )

#define F_CPU 16000000
#define BLINK_DELAY_MS 500

enum ATMEGA328P_APINS
{
    A0      = DDC0,
    A1      = DDC1,
    A2      = DDC2,
    A3      = DDC3,
    A4      = DDC4,
    A5      = DDC5,
    A6      = DDC6
};

enum ATMEGA328P_DPINS
{
    /* D0 - D1 are RX/TX. Leave them alone. */
    D2      = DDD2,
    D3      = DDD3,
    D4      = DDD4,
    D5      = DDD5,
    D6      = DDD6,
    D7      = DDD7,
    D8      = DDB0,
    D9      = DDB1,
};

enum pin_type
{
    PIN_READ,
    PIN_WRITE
};

typedef struct used_apins
{
    enum ATMEGA328P_APINS apin_used;
    enum pin_type RW_access;
} _used_apins;

typedef struct used_dpins
{
    enum ATMEGA328P_DPINS dpin_used;
    enum pin_type RW_access;
} _used_dpins;

uint8_t DB = 0B0000000;
uint8_t DC = 0B0000000;

#define invert(byte)    byte & (~byte)

/* "Add on" to the binary.
 * Or-ing the binary with itself makes sure whatever values
 * existed before still exist.
 * */
#define add_on(byte, new_byte)    byte | new_byte

uint8_t set_bit(uint8_t k)
{
    return (DB | (1 << (k - 1)));
}
uint8_t clear_bit(uint8_t k)
{
    return (DB | (1 << (k - 1)));
}

/* TODO: Add support for PORTD (pins 2-7, PD0-PD7). */
class ATMEGA328P
{
private:
    _used_apins apins_used[7];
    _used_dpins dpins_used[6];

public:

    ATMEGA328P(enum ATMEGA328P_APINS used_apins[], enum ATMEGA328P_DPINS used_dpins[])
    {
        for(uint8_t i = 0; used_apins[i]; i++)
            apins_used[i].apin_used = used_apins[i];
        
        for(uint8_t i = 0; used_dpins[i]; i++)
            dpins_used[i].dpin_used = used_dpins[i];
    }

    template<typename pt>
    void set_write_access(pt ptype, uint8_t type)
    {
        printf("Yes");
        uint8_t index = 0;

        if(type == 'a')
        {
            for(; apins_used[index].apin_used != ptype; index++);

            apins_used[index].RW_access = PIN_WRITE;
            return;
        }

        if(type == 'd')
        {
            for(; dpins_used[index].dpin_used != ptype; index++);

            dpins_used[index].RW_access = PIN_WRITE;
            return;
        }
        
        return;
    }

    template<typename pt>
    void set_read_access(pt pin, uint8_t type)
    {
        uint8_t index = 0;

        if(type == 'a')
        {
            for(; apins_used[index].apin_used != pin; index++);

            apins_used[index].RW_access = PIN_READ;
            return;
        }

        if(type == 'd')
        {
            for(; dpins_used[index].dpin_used != pin; index++);

            dpins_used[index].RW_access = PIN_READ;
            return;
        }

        return;
    }

    void init_driver()
    {
        DDRB = add_on(DDRB, DB);
        DDRC = invert(DDRC);

        return;
    }

    template<typename pt>
    void init_pin(pt pin, uint8_t type)
    {
        uint8_t index = 0;

        if(type == 'a')
        {
            for(; apins_used[index].apin_used != pin; index++);

            if(apins_used[index].RW_access == PIN_READ)
                DC = set_bit(apins_used[index].apin_used);
            else
                DC = clear_bit(apins_used[index].apin_used);
        }

        if(type == 'd')
        {
            for(; dpins_used[index].dpin_used != pin; index++);

            if(dpins_used[index].RW_access == PIN_READ)
                DB = set_bit(dpins_used[index].dpin_used);
            else
                DB = clear_bit(dpins_used[index].dpin_used);
        }

        return;
    }

    template<typename pt>
    int32_t read_pin(pt pin, uint8_t type)
    {
        if(type == 'a')
            return PINC & _BV(pin);

        if(type == 'd')
            return PINB & _BV(pin);
        
        return -1;
    }

    ~ATMEGA328P()
    {}
};

int main(void)
{
    /* A1 - On high, the sensor sensed something. 
     * A0 - Just a test to make sure `ATMEGA328P` can handle more than 1 pin.
     * */
    enum ATMEGA328P_APINS apins[] = {A1, A0};
    enum ATMEGA328P_DPINS dpins[] = {D8, D9};

    /* Init the ATMEGA328P Arduino Nano driver. */
    ATMEGA328P driver(apins, dpins);

    /* Set access for each pin. */
    driver.set_read_access<enum ATMEGA328P_APINS> (A1, 'a');
    driver.set_read_access<enum ATMEGA328P_APINS> (A0, 'a');
    driver.set_write_access<enum ATMEGA328P_DPINS> (D8, 'd');
    driver.set_write_access<enum ATMEGA328P_DPINS> (D9, 'd');

    /* Initialize each pin. */
    driver.init_pin<enum ATMEGA328P_DPINS> (D8, 'd');
    driver.init_pin<enum ATMEGA328P_DPINS> (D9, 'd');
    driver.init_pin<enum ATMEGA328P_APINS> (A1, 'a');
    driver.init_pin<enum ATMEGA328P_APINS> (A0, 'a');
    
    /* Initialize the driver so we can read/write to the according pins. */
    driver.init_driver();

    for(;;)
    {
        /* See if the sensor sent a `HIGH` value to pin A1. */
        if(driver.read_pin<enum ATMEGA328P_APINS> (A1, 'a') != 0)
        {
            PORTB |= _BV(PORTB0);
        } else
        {
            PORTB &= ~_BV(PORTB0);
        }

        /* See if the sensor sent a `HIGH` value to pin A0. */
        if(driver.read_pin<enum ATMEGA328P_APINS> (A0, 'a') != 0)
        {
            PORTB |= _BV(PORTB1);
        } else
        {
            PORTB &= ~_BV(PORTB1);
        }

        _delay_ms(BLINK_DELAY_MS);
    }
}
