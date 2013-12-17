/**
 *   Copyright (C) 2013 Ben Martin
 *
 *   Code for doing interesting things on Arduino.
 *
 *   This arduino code is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   libferris is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with libferris.  If not, see <http://www.gnu.org/licenses/>.
 *
 *   For more details see the COPYING file in the root directory of this
 *   distribution.
 *
 ********************************************************************
 *
 * This library allows entire "messages" to be passed to the attiny84.
 *
 * It handles:
 * 1) the USI SPI slave stuff
 * 2) will set timedOut() to true after a preiod of no SPI messages
 *    from the SPI master. You can use the to turn off things like attached
 *    screen from the library client.
 *   
 */
#include "Arduino.h"
#include "SimpleMessagePassing5.h"

volatile byte SPIreply = 0;

SimpleMessagePassing5* singleton = 0;
volatile int chipSelected = 0;
byte ISR_chipSelectPin=0;

volatile byte TimedOut = 0;
volatile unsigned long chipDeselectTime = 0;

const byte screenOffIntervals = 3;
byte WDT_Count = 0;

ISR(WDT_vect)
{
    if( chipSelected )
        return;
    if( !chipDeselectTime )
        return;

    WDT_Count++;
    if( WDT_Count >= screenOffIntervals )
    {
        TimedOut = 1;
        // disable the watchdog.
        WDTCSR &= ~(1<<WDIE);
    }
}

ISR(PCINT0_vect) 
{
    chipSelected = !chipSelected;

    byte disable = (HIGH == digitalRead( ISR_chipSelectPin ));
    if( disable )
    {
        USICR &= ~(1 << USIOIE);
        chipDeselectTime = millis();

        // reset watchdog count
        WDT_Count = 0;
        // enable the watchdog.
        WDTCSR |= (1<<WDIE);
    }
    else
    {
        TimedOut = 0;
        USICR |= (1 << USIOIE);
    }
 
    // Toggle the overflow ISR
    //    USICR = USICR ^ (1 << USIOIE);
}

void SimpleMessagePassing5::setReply(byte v )
{
    SPIreply = v;
}


SimpleMessagePassing5::SimpleMessagePassing5( int _chipSelectPin )
    :
    chipSelectPin( _chipSelectPin ),
    haveMessage(0),
    hadSpiInterrupt(0)
{
    singleton = this;
}

void SimpleMessagePassing5::init()
{
    spiData.init( SPIDATA_CAPACITY );
    
    pinMode(chipSelectPin, INPUT);
    pinMode(MOSI, INPUT);  // DI   ---> MOSI of ATmega328 (Pin 11)
    pinMode(MISO, OUTPUT); // DO   ---> MISO of ATmega328 (Pin 12)
    pinMode(SCK,  INPUT);  // USCK ---> SCK  of ATmega328 (Pin 13)

    USICR = (1 << USIWM0) // Three wire mode
        | (1 << USICS1)  // USI is clocked from the wire itself
        ;//| (1 << USIOIE);

    // always watching
//    USICR |= (1 << USIOIE);
    
    // Set the range of pcint 0-7 in the general interupt mask register
    GIMSK |= (1<<PCIE0);
    // PCint 7 is DP7 and is pin 6 on the chip
    PCMSK0 |= (1<<PCINT7);

    ISR_chipSelectPin = chipSelectPin;

    // watchdog: 4 second timeout. Page 47.
    WDTCSR = (1<<WDP3); // | (1<<WDP0);
}

enum 
{
    STATE_UNKNOWN = 1,
    STATE_STARTED_MESSAGE,
    STATE_STARTED_MESSAGE_HAVE_SIZE,
    STATE_WAITING_MESSAGE_END,
    STATE_TERMINAL_END
};

// support ascii sizes for testing
byte SimpleMessagePassing5::maybeConvertAsciiDigitToNumber( byte b ) const
{
    if( b >= '0' && b <= '9' )
        b -= '0';
    return b;
}

void SimpleMessagePassing5::serviceInput( char c )
{
    static int state = STATE_UNKNOWN;
    static int messageSize = 0;

    if( state == STATE_STARTED_MESSAGE )
        c = maybeConvertAsciiDigitToNumber( c );
    if( state == STATE_UNKNOWN && c != 'm' )
        return;
    
    spiData.put( c );

    switch( state )
    {
        case STATE_UNKNOWN:
            if( c == 'm' )
                state = STATE_STARTED_MESSAGE;
            break;
                    
        case STATE_STARTED_MESSAGE:
            state = STATE_STARTED_MESSAGE_HAVE_SIZE;
            // we already read the byte containing the length
            // so we have size-1 characters left to read for this message.
            messageSize = c-1;
            break;

        case STATE_STARTED_MESSAGE_HAVE_SIZE:
            state = STATE_WAITING_MESSAGE_END;
            if( c == METH_internal_ping )
                SPIreply = spiData.getSize() > SPIDATA_PING_BYTES_TO_HAVE_FREE;
            messageSize--;
            if( !messageSize )
            {
                state = STATE_UNKNOWN;
                haveMessage++;
            }
            break;
            
        case STATE_WAITING_MESSAGE_END:
            messageSize--;
            if( !messageSize )
            {
                state = STATE_UNKNOWN;
                haveMessage++;
            }
            break;
    }
}
    

byte
SimpleMessagePassing5::getAvailableMessageCount() const
{
    return haveMessage;
}

boolean
SimpleMessagePassing5::takeMessage()
{
    //
    // The work of checking if we have a message has been moved
    // here from the ISR
    //
    if( spiData.getSize() < 3 )
        return false;

    byte c;
    //
    // keep reading until the 'm' start of message sentinal
    //
    while( true )
    {
        c = spiData.peek(0);
        if( c == 'm' )
            break;

        // eat the garbage
        spiData.get();
    }
    if( spiData.getSize() < 3 )
        return false;

    // We know that at index zero we have:
    // index0 == 'm'
    // index1, size is available
    // index2, meth is available
    byte messageSize = maybeConvertAsciiDigitToNumber( spiData.peek(1) );
    byte meth        = spiData.peek(2);

    if( messageSize > 2 )
    {
        // The 'm' is not included in the messageSize number
        // but the messageSize byte itself is included.
        // so we need 1+messageSize to cover the leading 'm' sentinal.
        if( spiData.getSize() < 1 + messageSize )
            return false;
    }
    

    //
    // If we get here then we know that there is a full message ready!
    //
    return true;
    
    // byte localHaveMessage = 0;
    // noInterrupts();
    // if( haveMessage )
    // {
    //     localHaveMessage = 1;
    //     --haveMessage;
    // }
    // interrupts();
    // return localHaveMessage;
}

ByteBuffer&
SimpleMessagePassing5::buffer()
{
    return spiData;
}

void
SimpleMessagePassing5::breath( int ledline )
{
    static byte lc = 0;
    
    if(lc) { 
        lc=0;
        digitalWrite(ledline, HIGH);
    } else {
        lc=1;
        digitalWrite(ledline, LOW);
    }  
    
}

bool SimpleMessagePassing5::shouldSleep()
{
  return !chipSelected;
}


bool SimpleMessagePassing5::timedOut() const
{
    return TimedOut;
}

// USI overflow intterupt
ISR(USI_OVF_vect)
{
    if( ! chipSelected )
        return;

    // get the data
    byte b = USIDR;
    if( singleton )
        singleton->spiData.put( b );
    // if( singleton )
    //     singleton->serviceInput( b );
   
    // Clear counter overflow flag,
    // don't call the ISR again until you
    // really have more data for me!
    USISR = (1<<USIOIF);

    // This is the byte that will be sent
    // to the SPI master *next* transfer
    //byte SPIreply = 0;
    //if( singleton )
      //  SPIreply = singleton->SPIreply;
    USIDR = SPIreply;
} 
