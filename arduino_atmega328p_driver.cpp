#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <util/delay.h>

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

class ATMEGA328P
{
private:
    _used_apins apins_used[7];
    uint8_t apins_used_size = 0;

    _used_dpins dpins_used[6];
    uint8_t dpins_used_size = 0;

    template<typename pt>
    pt *get_pin(pt pin, enum pin_type RW_access, uint8_t type)
    {
        uint8_t i = 0;

        if(type == 'd')
        {
            for(; dpins_used[i].dpin_used != pin; i++);

            if(dpins_used[i].RW_access != RW_access) goto return_empty_slate;
            return &dpins_used[i];
        }

        if(type == 'a')
        {
            for(; apins_used[i].apin_used != pin; i++);

            if(apins_used[i].RW_access != RW_access) goto return_empty_slate;
            return &apins_used[i];
        }

        return_empty_slate:
        return 0;
    }

public:

    ATMEGA328P(enum ATMEGA328P_APINS used_apins[], enum ATMEGA328P_DPINS used_dpins[])
    {
        for(uint8_t i = 0; used_apins[i]; i++)
            { apins_used[i].apin_used = used_apins[i]; apins_used_size++; }
        
        for(uint8_t i = 0; used_dpins[i]; i++)
            { dpins_used[i].dpin_used = used_dpins[i]; dpins_used_size++; }
    }

    template<typename pt>
    void set_write_access(pt ptype, uint8_t type)
    {
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

            apins_used[index].RW_access = PIN_WRITE;
            return;
        }

        if(type == 'd')
        {
            for(; dpins_used[index].dpin_used != pin; index++);

            dpins_used[index].RW_access = PIN_WRITE;
            return;
        }

        return;
    }

    void init_driver()
    {
        uint8_t index = 0;

        for(; index <= apins_used_size; index++)
        {
            if(apins_used[index].RW_access == PIN_READ)
                DDRC &= ~_BV(apins_used[index].apin_used);
            if(apins_used[index].RW_access == PIN_WRITE)
                DDRC |= _BV(apins_used[index].apin_used);
        }

        index = 0;
        for(; index <= dpins_used_size; index++)
        {
            if(dpins_used[index].RW_access == PIN_READ)
                DDRB &= ~_BV(dpins_used[index].dpin_used);
            if(dpins_used[index].RW_access == PIN_WRITE)
                DDRB |= _BV(dpins_used[index].dpin_used);
        }

        //DDRB |= _BV(DDB0);
        //DDRC &= ~_BV(DDC1);

        for(;;)
        {
            int v = PINC & _BV(PC1);

            if(v != 0)
            {
                PORTB |= _BV(PORTB0);
            } else
            {
                PORTB &= ~_BV(PORTB0);
            }

            _delay_ms(BLINK_DELAY_MS);
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
    enum ATMEGA328P_APINS apins[] = {A1};
    enum ATMEGA328P_DPINS dpins[] = {D8};

    ATMEGA328P driver(apins, dpins);

    driver.set_read_access<enum ATMEGA328P_APINS> (A1, 'a');
    driver.set_write_access<enum ATMEGA328P_DPINS> (D8, 'd');

    driver.init_driver();

    //DDRB |= _BV(DDB0);
    //DDRC &= ~_BV(DDC1);

    for(;;)
    {
        int v = PINC & _BV(PC1);

        if(v != 0)
        {
            PORTB |= _BV(PORTB0);
        } else
        {
            PORTB &= ~_BV(PORTB0);
        }

        _delay_ms(BLINK_DELAY_MS);
    }
}
