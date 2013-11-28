/**
 *
 * Build on SimpleMessagePassing4 by moving the serviceInput()
 * call out of the ISR. The ISR just pushes the byte to the circular
 * array, handling of "do we have a message" state machine is done
 * in takeMessage() now.
 */

#include "ByteBuffer.h"

#ifndef SPIDATA_CAPACITY
#define SPIDATA_CAPACITY 100
#endif

#ifndef SPIDATA_PING_BYTES_TO_HAVE_FREE
#define SPIDATA_PING_BYTES_TO_HAVE_FREE 80
#endif

#ifndef METH_internal_base
#define METH_internal_base 240
#endif

enum
{
    METH_internal_ping = METH_internal_base + 1
};


class SimpleMessagePassing5
{
    int chipSelectPin;
    
  public:

    enum {
        MOSI = 6,  // D6, pin 7  Data In  (MOSI)
        MISO = 5,  // D5, pin 8  Data Out (MISO) 
        SCK  = 4   // D4, pin 9  Universal Serial Interface clock
    } pin_definition;
    
    volatile bool hadSpiInterrupt;
    // Tells us how many complete messages are buffered
    volatile byte haveMessage;

    
    void setReply(byte v );
    bool shouldSleep();


    SimpleMessagePassing5( int _chipSelectPin );

    void init();
    

    /**
     * If we are debugging then turn the ascii char 8 into the byte value 0x8.
     */
    byte maybeConvertAsciiDigitToNumber( byte b ) const;
    

    /**
     * Get the number of messages that are buffered for action
     */
    byte getAvailableMessageCount() const;

    /**
     * If there are no messages buffered for action return false.
     *
     * Otherwise decrement the number of messages ready for action in
     * an interrupt safe way and return true;
     */
    boolean takeMessage();

    /**
     * Get the circular buffer to allow clients to buffer().get()
     * bytes from.
     */
    ByteBuffer& buffer();

    bool timedOut() const;

    void breath( int ledline );

    // This should only be called by the ISR
    void serviceInput( char c );
    // This should only be called by the ISR
    ByteBuffer spiData;
    
};

