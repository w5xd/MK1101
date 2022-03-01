/* Copyright (C) 2018 by Wayne E. Wright, W5XD
 *  Round Rock, Texas
 *  
 * MK-1101 / MK-1102 / MK-1103 multi-keyer SO2R controller.
 * 
 * This sketch is an arduino implementation of the MK-1100 keyer originally
 * implemented on a Motorola 68HC705P microprocessor.
 * It is as close as I could think of to a literal port. The timer
 * is set to interrupt at 512usec intervals to match that used
 * in the MK-1100. This firmware works on the MK-1101, MK-1102, MK-1103
 * variants.
 * 
 * For programming novices that might be reading this:
 * This sketch is NOT a good example of much of anything other than
 * a port of 68HC705 assembly language to C++. The comments
 * have snippets of the original MK-1100 assembly language to
 * provide reference so that this port can behave identically to
 * the original. The names of variables and functions are those
 * used in the original firmware. Those kinds of things don't 
 * belong in C++ anymore, but they were useful for this particular
 * project.
 */
 
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

#define MK1101_BUILD_DEFINITION 1
#define MK1102_BUILD_DEFINITION 1 // MK1101/MK1102 have identical code
#define MK1103_BUILD_DEFINITION 3 // different because the serial out on SPI has one more byte

#define MK110X_BUILD MK1103_BUILD_DEFINITION

#if (MK110X_BUILD == MK1101_BUILD_DEFINITION)
#define SPI_MK110x_HZ  10000000L // The line driver chips are rated to 10MHz
#else
#define SPI_MK110x_HZ   1000000L // The low pass on the MK1103 is about 4MHz
#endif

namespace {
/* The Ardunio Pro Mini has 20 I/O pins. ALL are allocated.
** This code will require work to run on other Arduino boards.
** Of course the 20 I/O pins defined here would have to be
** worked out. Also the 512usec timer interrupt code below
** is only operable on certain Atmega processors. */

// connections from Adafruit breakout to Arduino
    const int RTS_PIN = 4;
    const int CTS_PIN = 5;
    const int DTR_PIN = 6;
    const int DSR_PIN = 7;
    const int DCD_PIN = A3;

    //SPI to talk to aux BOARD
    const int SPI_SCK = 13;
    const int SPI_MOSI = 11;
// The MK-1102/1103 aux board is on SPI on select pin 10
    const int AUX_SELECT_PIN = 10;
    const int FOOTRING_PIN = A7;

    const int CWL_PIN = A4;
    const int CWR_PIN = 15;
    const int PTTL_PIN = 14;
    const int PTTR_PIN = 16;
    const int CW_SIDETONE_PIN = 3;

    const int CWDOT_PIN = 9;
    const int CWDASH_PIN = 8;
    const int FOOTTIP_PIN = 12; // MISO we're reading footswitch
    const int RIGCTS_PIN = A5;
    const int LRSWITCH_PIN = 2;
    const int SPEEDPOT_PIN = A6;

    // In order to run on hardware with no pot attached to SPEEDPOT_PIN...
    const int KNOB_EPROM_IDX = 0;
    const int KNOB_REVERSE_IDX = 1;
    const int PTT_DELAY_IDX=2;
    const int LR_SWITCH_REVERSE_IDX = 3;

    void MANLRCHECK();
    void ARSCLR();
}

namespace host {
    void ProcPc();
    void setup();
}

namespace aux {
    void setup();
    void loop();

    uint8_t copyAux();
    void clearInAux(uint8_t toClear);
    void setInAux(uint8_t toSet);
    void toggleInAux(uint8_t toToggle);

    // these outputs on the aux board were physical pins on the 6805
    const uint8_t PRIRADR_M =  0x02u; // High means Radio LEFT is primary now
    const uint8_t RXAUDL_M =  0x40u; // Left headphone relay
    const uint8_t RXAUDR_M =  0x01u; // Right headphone relay
    const uint8_t PTTOUTL_M = 0x04u; // Left PTT
    const uint8_t PTTOUTR_M = 0x08u; // Right PTT
    const uint8_t CWOUTL_M =  0x20u; // Left CW
    const uint8_t CWOUTR_M =  0x10u; // Right CW
    const uint8_t PRIRADL_M = 0x80u; // on other lpt port, but matches PRIRADR_M
    // ... in order to match MK-1100 behavior
}

namespace cw {
    const uint8_t CW_MARK = 0;
    const uint8_t CW_SPACE = 1;
    const uint8_t CW_IDLE = 2;
    const uint8_t CW_WAIT = 3;
    const uint8_t CW_TUNE = 4;
    const uint8_t CW_BUG = 5;
    const uint8_t CW_NUMSTATES = 6;

    // the ALL CAPS variable names to match those in the 6805 assembler.
    volatile uint8_t CWSTATE;
    volatile uint8_t CWPTRIN;
    volatile uint8_t CWPTROU;
    volatile uint8_t CWBUFF[8];
    volatile uint8_t DOTLEN; // morse dot
    volatile uint16_t LTRLEN; // normally twice dot, but farnsworth lengthens
    volatile int8_t WEIGHT; // WEIGHT is SIGNED
    volatile uint8_t PTTIDL;

    /* volatile
     * its necessary to declare variables that are touched by both mainline
     * and interrupt service routines as "volatile". That keeps the optimizer
     * from emitting mainline code that misses changes made by interrupt service
     * routines. But volatile is a really brutal de-optimization. Every single
     * reference in source code to a volatile memory location goes to memory
     * with no use of machine registers.
     *
     * The code below reintroduces the optimization by using variables to hold
     * and modify an otherwise "volatile" memory location, and then store
     * the result back after a series of modifications. This means, of course,
     * that changes by an interrupt service routine over that sequence
     * will be lost. So this coding technique can only be used in routines
     * running with noInterrupts, or as an interrupt service routine.
     */

    void setup();
    void loop();
    unsigned long getCount();
    void SWI(); // was "software interrupt" in 68HC705 code
    // SWI ran out-of-order on the 6805. That is, it didn't
    // actually run until interrupts were next enabled.
    // That difference, though, does not affect the match
    // between MK-1100 and MK-1101 behavior.
    void CLRCWBUF();
    void OutputCw(bool Mark);
}

namespace ptt {
    void loop();
    void SetPttOn();
    void SetPttOff();
    void SETUPPTTDELAY();
    void setup();
    volatile uint16_t PttCntHiLo;
    volatile uint8_t PTTDELAY;
}

namespace diag {
    void CheckSerial();
    void incomingChar(uint8_t);
    bool printCommands = false;
}

// globals carried over from PCKY_P1.ASM
namespace {
    const uint8_t BUFNOTETY = 1 << 0; // CW buffer not empty
    const uint8_t AUDLATCH = 1 << 1;    // Audio latch
    const uint8_t PADDACTI = 1 << 2;    // Paddles active
    // Below appear in host shifted three bits lower in byte (bits 0 through 4)

    // Sidetone bits are negative logic: ON means supress sidetone
    const uint8_t STMACHINE = 1 << 3;   // Sidetone on machine sent
    const uint8_t STONPDLS = 1 << 4;    // Sidetone on paddles
    const uint8_t PRIRADDLOAD = 1 << 5;  // Primary radio. ON is Right
    const uint8_t HDPHSPLIT = 1 << 6;   // Headphones split
    const uint8_t AUTOLTR = 1 << 7;     // Auto letter space

    uint8_t KNOBSPD;    // CW knob speed
    //the bit layout of ARSLATCH match 68hc705 code 
    volatile uint8_t ARSLATCH;
}

//The setup function is called once at startup of the sketch
void setup()
{
    Serial.begin(9600); // Serial I/O is for debugging only
    Serial.println(
#if (MK110X_BUILD == MK1101_BUILD_DEFINITION)
        F("MK1101/MK1102")
#else
        F("MK1103")
#endif
    );

    // These pins have nothing to do with Serial library
    // The MK-1101 is the Data Set (not Data Terminal)
    pinMode(CTS_PIN, OUTPUT);
    pinMode(DSR_PIN, OUTPUT);
    pinMode(DCD_PIN, OUTPUT);
    pinMode(RTS_PIN, INPUT);
    pinMode(DTR_PIN, INPUT);

    digitalWrite(SPEEDPOT_PIN, LOW);
    pinMode(SPEEDPOT_PIN, INPUT);

    pinMode(CWDOT_PIN, INPUT_PULLUP);
    pinMode(CWDASH_PIN, INPUT_PULLUP);
    pinMode(FOOTTIP_PIN, INPUT_PULLUP);
    pinMode(LRSWITCH_PIN, INPUT_PULLUP);

    digitalWrite(AUX_SELECT_PIN, HIGH);
    pinMode(AUX_SELECT_PIN, OUTPUT);
    digitalWrite(CWL_PIN, LOW);
    pinMode(CWL_PIN, OUTPUT);
    digitalWrite(CWR_PIN, LOW);
    pinMode(CWR_PIN, OUTPUT);
    digitalWrite(PTTL_PIN, LOW);
    pinMode(PTTL_PIN, OUTPUT);
    digitalWrite(PTTR_PIN, LOW);
    pinMode(PTTR_PIN, OUTPUT);
    digitalWrite(CW_SIDETONE_PIN, LOW);
    pinMode(CW_SIDETONE_PIN, OUTPUT);

    digitalWrite(CTS_PIN, LOW);
    digitalWrite(DSR_PIN, LOW);
    digitalWrite(DCD_PIN, HIGH);

    analogReference(DEFAULT);

    SPI.begin();
    cw::setup();
    aux::setup();
    host::setup();
    ptt::setup();
}

// The loop function is called in an endless loop
void loop()
{
     /*
    POLLLOOP:
    RSP
    BSET    OCIE,TCSR       ;ensure 512 usec interrupt
    CLI                     ;Enable interrupts

    CLRA
    STA     COP             ;tell watchdog to go back to sleep
    LDX     CWSPDST
    ASLX
    JSR     CWSPDISP,X      ;dispatch on CW speed state

    SEI
    BSR     PROCPC
    CLI     ;allow interrupt routines to bust in here...

    SEI                       ;check the CW paddles and respond accordingly
    BSR     PDLCHECK
    CLI     ;allow interrupt routines to bust in here...

    SEI
    BSR     PROCPC
    CLI     ;allow interrupt routines to bust in here...

    BSR     MANLRCHECK

    BSR     FTPTTCHECK

    SEI
    JSR     ARSCLR

    BSET    CANDOBUGTUNE,                                                                 ;Identifies this ROM to other side
    BRA     POLLLOOP
*/

/*interrupts and noInterupts
** The 68HC705 code ran the main loop with interrupts OFF, turning them ON inside the loop
** and turning them back off. The arduino code can't quite do the same as it uses SPI and analogRead
** which require interrupts to be turned ON to work.
** 
** In this sketch, the responsibility for cli/sei is inside the routines themselves.
** struct LockInterrupts, below, helps.
*/
    aux::loop();
    host::ProcPc();
    cw::loop();
    MANLRCHECK();
    ptt::loop();
    ARSCLR();
    diag::CheckSerial();
}

// instance one of these to turn off interrupts. The compiler
// will find all the exit paths and turn interrupts back on.
struct LockInterrupts {
    LockInterrupts()
    {  noInterrupts();}
    ~LockInterrupts()
    { interrupts();   }
};

namespace aux {
    volatile uint8_t AuxBoardOutput;
    uint8_t AuxBoardAnt; // not touched by interrupt service

    inline uint8_t copyAux()
    {        return AuxBoardOutput;    }

    inline void clearInAux(uint8_t toClear)
    {        AuxBoardOutput &= ~toClear;    }

    inline void setInAux(uint8_t toSet)
    {        AuxBoardOutput |= toSet;    }

    inline void toggleInAux(uint8_t toToggle)
    {       AuxBoardOutput ^= toToggle;}

     // SPI send of 16 bits to Aux board
    void ToAuxBoard(uint8_t v, uint8_t ant)
    {
        digitalWrite(AUX_SELECT_PIN, LOW);
        SPI.beginTransaction(SPISettings(SPI_MK110x_HZ, MSBFIRST, SPI_MODE3));
        SPI.transfer(v);
        SPI.transfer(ant);
#if (MK110X_BUILD == MK1103_BUILD_DEFINITION)
        // the MK1103 circuit has 3 bytes in the shift register, and the closest
        // one to this MCU has K11/K12 control in same bits as farthest one in MK1102
        SPI.transfer(v);
#endif
        SPI.endTransaction();
        digitalWrite(AUX_SELECT_PIN, HIGH);
    }

    void setup()  {  ToAuxBoard(copyAux(), AuxBoardAnt);    }

    void loop()
    {
        static uint8_t lastUpdated;
        static uint8_t lastAnt;
        uint8_t aux = copyAux();
        if ((aux != lastUpdated) || (AuxBoardAnt != lastAnt))
        {
            lastUpdated = aux;
            lastAnt = AuxBoardAnt;
            ToAuxBoard(aux, AuxBoardAnt);
        }
    }
}

namespace host {
    const int PCREC0 = RTS_PIN;
    const int PCREC1 = DTR_PIN;
    const int PCXMT0 = DSR_PIN;
    const int PCXMT1 = CTS_PIN;
 
    typedef uint8_t HostState_t;
    const HostState_t PC_DONE = 0;
    const HostState_t PC_BIT1 = 1;
    const HostState_t PC_BIT0 = 2;
    const HostState_t PC_SHIFT = 3;

    /* status bits in STSTATUS*/
    const uint8_t XMITTING = 1 << 7;
    const uint8_t FIFOFULL = 1 << 6;
    const uint8_t USINGMANLR = 1 << 5;
    // BITS 4 & 3 unassigned
    const uint8_t CANDOBUGTUNE = 1 << 2;
    const uint8_t MANLRSEL = 1 << 1;
    // const uint8_t STONBIT = 1; MK-1101 does sidetone differently

    // status byte to send to host on request
    volatile uint8_t STSTATUS;
    // volatile generates a lot more memory accesses...but some of them might be needed.
    // defeat volatile by operating on a variable copied from here, and then write back.

    volatile uint8_t STONCOUNT; // WriteLog default: 1
    volatile uint8_t STOFFCOUNT;    // WriteLog default: 4

    uint32_t PCSPEED;

    const uint8_t STAT2TUNE = 1 << 7;
    const uint8_t HPTOGGLE = 1 << 6;
    const uint8_t STATABUG = 1 << 5;
    volatile uint8_t STATUS2;

    const uint8_t     PCM_DATA     = 0b11100000; //5 bits of DATA
    const uint8_t     PCM_ABORT    = 0b11000000;// ABORT COMMAND
    const uint8_t     PCM_LATCH    = 0b11000001; //SET AUDIO LATCH
    const uint8_t     PCM_XFRSP    = 0b11000010; //TRANSFER SPEED NOW
    const uint8_t     PCM_XFRWT    = 0b11000011; //transfer weight now
    const uint8_t     PCM_DEFSPE   = 0b11000100; //transfer DATA byte to morse buffer
        //; if the data byte is $C0 or higher,
        //; then it is shifted
        //; left two bits and used as the speed,
        //; and this is done synchronously
        //; The fact that the high byte of ltrlen
        //; must be zero limits this technique to
        //; 20 WPM and above
    const uint8_t     PCM_UNLOCK   = 0b11000101; //unlock paddles active lock
    const uint8_t     PCM_XFAUTO   = 0b11000110;// XFR AUTOLTR bit to ARSLATCH
        //; incoming bit mask is :
        //; #7 is AUTO LETTER SPACE
        //; #6 is Headphone mode(1 is split, 0 is same)
        //; #5 is PRIRAD(0 gives CW on right)
        //; #4, #3

    const uint8_t     PCM_ANTRLY   = 0b11000111; //8 bits to the antenna relay
    const uint8_t     PCM_PTTON    = 0b11001000; //PTT ON
    const uint8_t     PCM_PTTOFF   = 0b11001001; //PTT OFF
    const uint8_t     PCM_XFSTONE  = 0b11001010; //XFER SIDETONE COUNTS
    const uint8_t     PCM_XFWAVEST = 0b11001011; //XFER WAVE STATUS
    const uint8_t     PCM_DELETE   = 0b11001100; //DELETE NEWEST CHARACTER
    const uint8_t     PCM_PTTDELAY = 0b11001101; //Transfer PTT Delay value
    const uint8_t     PCM_XFTUNEETC= 0b11001110; //Transfer TUNE bit and headphones reverse bit
    const uint8_t     PCM_UNLATCH  = 0b11001111; //opposite of PCM_LATCH

    void incomingChar(uint8_t);
    void MORSE(uint8_t pcChar);
    void CMDDISPATCH(uint8_t pcChar);

    void setup()
    {        STSTATUS = CANDOBUGTUNE;    }

    void ProcPc()
    {   // examine modem control lines for host commands
        static HostState_t prevVal;
        HostState_t val = static_cast<HostState_t>((
                digitalRead(PCREC0) == HIGH ? 1 : 0)
                + (digitalRead(PCREC1) == HIGH ? 2 : 0));
        if (val == prevVal)
        {	// require two consecutive polls to give same result
            static HostState_t state;
            static uint8_t pcCount;
            static uint8_t pcChar;
            static uint8_t pcXmit;
            if (val != state)
            {	// state change processing
                if (val != PC_SHIFT)
                {
                    if (val == PC_DONE)
                    {
                        digitalWrite(PCXMT0, LOW);
                        digitalWrite(PCXMT1, LOW);
                        if (pcCount == 7)
                        {
                            pcChar <<= 1;
                            pcChar |= (state & 1);
                            incomingChar(pcChar);
                        } // else its an error.
                        pcCount = 0;
                    } else // bit0 and bit1 processing
                    {
                        // SENDXMITBIT
                        if (pcCount == 0) // what we send depends on the first bit we receive
                        {
                            pcXmit = (val == PC_BIT1) ? STSTATUS : KNOBSPD ;
                        }
                        if (0 == (pcXmit & 0x80))
                        {
                            digitalWrite(PCXMT0, HIGH);
                            digitalWrite(PCXMT1, LOW);
                        }
                        else
                        {
                           digitalWrite(PCXMT1, HIGH);
                           digitalWrite(PCXMT0, LOW);
                        }
                    }
                } else
                { // val == PC_SHIFT
                    if (state != PC_DONE)
                    {
                        pcCount += 1;
                        pcChar <<= 1;
                        pcChar |= (state & 1);
                        digitalWrite(PCXMT0, HIGH);
                        digitalWrite(PCXMT1, HIGH);
                        pcXmit <<= 1;
                    } else
                        // illegal state transition
                        pcCount = 0;
                }
                state = val;
            }
        } else
            prevVal = val;
    }

    void incomingChar(uint8_t pcChar)
    {
        {
            LockInterrupts lock;
            // no interrupts while processing command
            if (pcChar < 0xC0)
                MORSE(pcChar);
            else if (pcChar < PCM_DATA)
                CMDDISPATCH(pcChar);
            else
            {
                PCSPEED >>= 5;
                PCSPEED |= uint32_t(pcChar & 0x1F) << 19;
             }
        }

        diag::incomingChar(pcChar);
    }

    void MORSE(uint8_t pcChar)
    {
        // add a character to the processing queue at CWPTRIN/CWPTROUT

        /*
        MORSE:   ;ACC is morse character
        BRSET   PADDACTI,ARSLATCH,MORSERET
        LDX     CWPTRIN         ;Index into CW buffer
        STA     CWBUFF,X        ;Store new character
        BSET    XMITTING,STSTATUS ;Tell PC we're transmitting for it
        ;update buffer pointer
        TXA                    ;go to next byte next time
        INCA
        AND     #$7             ;Buffer circulates through 8 bytes
        STA     CWPTRIN
        INCA                    ;Check next index for feedback loop
        AND     #$7             ;only these bit
        EOR     CWPTROU         ;in/out ptr equal?
        BNE     NOFEEDB
        BSET    FIFOFULL,STSTATUS    ;Tell PC we're full up

        NOFEEDB:BSET    BUFNOTETY,ARSLATCH
        MORSERET:
        BRA     NOGOOD          ;Actually, it IS good, but need clr count
        */
        if (0 == (PADDACTI & ARSLATCH))
        {
            auto tempstat = host::STSTATUS;
            tempstat |= host::XMITTING;
            auto cwptrin = cw::CWPTRIN;
            cw::CWBUFF[cwptrin++] = pcChar;
            cwptrin &= 7;
            cw::CWPTRIN = cwptrin++;
            cwptrin &= 7;
            if (cwptrin == cw::CWPTROU)
                tempstat |= host::FIFOFULL;
            ARSLATCH |= BUFNOTETY;
            host::STSTATUS = tempstat;
        }
    }

    const unsigned NUM_COMMANDS = 16;
    typedef void (*CmdDispatch_t)();
    extern CmdDispatch_t const COMTABL[NUM_COMMANDS];

    void CMDDISPATCH(uint8_t pcChar)
    {   // range is 0xC0 through 0xDF (1 less than PCM_DATA)
        pcChar &= 0x1F; // 0 through 31
        if (pcChar < NUM_COMMANDS)
            (*COMTABL[pcChar])();
    }

    void CHKLATCH()
    {        ARSLATCH |= AUDLATCH;    }

    void CHKXFRSP()
    {
        /*
        CHKXFRSP:
        ;        LDA     PCSPEED
        STA     DOTLEN
        CLR     KNOBSPD
        LDA     PCSPEED+1
        STA     LTRLEN
        LDA     PCSPEED+2
        STA     LTRLEN+1
        RTS

        */
        KNOBSPD = 0;
        cw::DOTLEN = (uint8_t)(PCSPEED >> 16);
        cw::LTRLEN = (uint16_t)(PCSPEED);
    }

    void CHKWEIGHT()
    {        cw::WEIGHT = (int8_t)(PCSPEED >> 16);    }

    void DEFSPE()
    {        MORSE((uint8_t)(PCSPEED >> 16));    }

    void CHECKUNLOC()
    {
        /*
        CHECKUNLOC:
        BCLR    PADDACTI,ARSLATCH       ;Paddles not active
        RTS
        */
        ARSLATCH &= ~PADDACTI;
    }
    
    void XFRAUTOLTR()
    {
        /*
XFRAUTOLTR:
        ;replace top 5 bits of ARSLATCH with top 5 bits of acc...
        ;bit 0, 1, 2 are also significant in 10.31 and higher
        AND     #$F8
        TAX
        LDA     ARSLATCH
        AND     #7
        STA     ARSLATCH
        TXA
        ORA     ARSLATCH
        STA     ARSLATCH
        BCLR    USINGMANLR,STSTATUS     ;show NOT using manual LR
        EOR     PORTA                   ;ASSUMES PRIRADDLOAD == PRIRAD!!!!!
        AND     #PRIRADDLOAD_M          ;see if new bit represents a change
        BEQ     XFRAUTODONE             ;branch on no change
        SWI                             ;Stop any xmit in progress--interrupts are OFF now.
        BRCLR   PRIRADDLOAD,ARSLATCH,GORADB
        BSET PRIRAD,PORTA
        BRA  XFRAUTODONE
GORADB: BCLR PRIRAD,PORTA
XFRAUTODONE:
        RTS
        */
        uint8_t temp = PCSPEED >> 16;
        temp &= 0xF8;
        uint8_t temp2 = ARSLATCH;
        temp2 &= 0x7;
        temp2 |= temp;
        ARSLATCH = temp2;
        host::STSTATUS &= ~USINGMANLR;
        bool newIsLeft = (0 != (temp2 & PRIRADDLOAD));
        uint8_t oldAux = aux::copyAux();
        bool oldIsLeft = (0 != (oldAux & aux::PRIRADL_M));
        if (newIsLeft != oldIsLeft)
        {
            cw::SWI();
            aux::toggleInAux(aux::PRIRADR_M | aux::PRIRADL_M);
        }
    }

    void XFRRELAY()
    {
        /*
        XFRRELAY:    ;top 5 bits of acc are (in order)
        ; radio l/r flag
        ; 4 bits of relay output
        ;
        ;load the 4 bits into the high/low of ANTRELAY
        ;and then send all of ANTRELAY to SDR
        LDX  ANTRELAY ;old value of antenna relay

        LSLA    ;l/r flag to c bit and other 4 bits to top
        BCS  TOPANTBITS
        ;new bits go to LOWER part
        JSR  NIBBLESWAPX
        JSR  NIBBLEAX  ;top of A becomes bottom of X

        BRA  ANTBITSDONE

        TOPANTBITS:    ;top A bits become top X bits
        JSR  NIBBLEAX  ;top of A becomes bottom of X
        JSR  NIBBLESWAPX  ;

        ANTBITSDONE:
        STX  ANTRELAY
        STX    SDR              ;send the bits out...
        COMTRTS:
        RTS
        */
        uint8_t temp = PCSPEED >> 16;
        uint8_t mask;
        if (0 != (temp & 0x80))
        {   //LEFT
            temp <<= 1;
            mask = 0xF0;
        }
        else
        {
            temp >>= 3;
            mask = 0x0F;
        }
        temp &= mask;
        aux::AuxBoardAnt &= ~mask;
        aux::AuxBoardAnt |= temp;
    }

    void SETPTTONRM()
    {
        /*
        SETPTTONRM:
        BRSET PADDACTI,ARSLATCH,PTTOFFRET  ;If paddles running, ignore PTT Off
        SETPTTON:                ;turn ON one of the PTT outputs according to the PRIRAD bit
        */
        if (0 == (ARSLATCH & PADDACTI))
            ptt::SetPttOn();
    }

    void SETPTTOFFRM()
    {
        /*
        SETPTTOFFRM:
        BRSET PADDACTI,ARSLATCH,PTTOFFRET  ;If paddles running, ignore PTT Off
        SETPTTOFF:
        */
        if (0 == (ARSLATCH & PADDACTI))
            ptt::SetPttOff();
    }
    
    void XFRSTONE()
    {
        /* This function is here because of the MK-1100, but
        ** sidetone on the MK-1101 is done with a buzzer device
        ** and does not use STONCOUNT/STOFFCOUNT */

    /*
    XFRSTONE:
        STA     STONCOUNT
        LDA     PCSPEED+1
        STA     STOFFCOUNT
        RTS

*/
        STONCOUNT = (uint8_t) (PCSPEED >> 16);
        STOFFCOUNT = (uint8_t) (PCSPEED >> 8);
    }

    void XFRWAVSTAT()   
    {} // Unused in current versions of WriteLog, and there is no REC nor PB pin assigned

    void BACKUPCHAR()
    {
        /*
        BACKUPCHAR:
        ;IF BUFFER IS EMPTY, THEN NOOP
        LDA    CWPTRIN  ;ACC is IN index
        CMP    CWPTROU          ;Compare OUT with IN
        BEQ     MRSEMPTY
        DECA
        AND     #$7
        STA     CWPTRIN

        MRSEMPTY:
        RTS
        */
        auto cwptrin = cw::CWPTRIN;
        if (cwptrin != cw::CWPTROU)
        {
            cwptrin -= 1;
            cwptrin &= 7;
            cw::CWPTRIN = cwptrin;
        }
    }
    
    void LOADPTTDEL()
    {
        /*
        LOADPTTDEL:
        STA     PTTDELAY
        LDA     PCSPEED+1
        STA     PTTIDL
        LSR     PTTIDL  ; throw away bottom bit
        RTS
        */
        ptt::PTTDELAY = (uint8_t)(PCSPEED >> 16);
        cw::PTTIDL = ((uint8_t)(PCSPEED >> 8)) >> 1;
    }
    
    void XFTUNEETC()
    {
        /*
        XFTUNEETC:
        STA     STATUS2
        BMI     DOTUNECMD
        RTS
 DOTUNECMD:
         SWI                    ;break anything in progress
         BCLR   PADDACTI,ARSLATCH      ;paddles not active
        LDA   #CW_TUNE
         STA   CWSTATE
        LDA     #CWOUTR_M
        BRCLR   PRIRAD,PORTA,TUNERADIO_B    ;Input says other radio is primary now?
        LSRA
TUNERADIO_B ;A is bit mask for use in PORTB
        ORA     PORTC
        STA     PORTC
         RTS
       */
        STATUS2 = (uint8_t)(PCSPEED >> 16);
        if (0 != (STAT2TUNE & STATUS2))
        {
            cw::SWI();
            ARSLATCH &= ~PADDACTI;
            cw::CWSTATE = cw::CW_TUNE;
            cw::OutputCw(true);
            // CHANGE IN BEHAVIOR FROM MK1100:
            if (0 == (ARSLATCH & STMACHINE))
                 digitalWrite(CW_SIDETONE_PIN, HIGH);
        }
    }
    
    void CHKUNLATCH()
    {        ARSLATCH &= ~AUDLATCH;    }

    CmdDispatch_t const COMTABL[NUM_COMMANDS] =
    {
        &cw::SWI, &CHKLATCH, &CHKXFRSP, &CHKWEIGHT, &DEFSPE, &CHECKUNLOC, &XFRAUTOLTR, &XFRRELAY,
        &SETPTTONRM, &SETPTTOFFRM, &XFRSTONE, &XFRWAVSTAT, &BACKUPCHAR, &LOADPTTDEL, &XFTUNEETC, &CHKUNLATCH
    };
}

namespace {
    void MANLRCHECK()
    {
        /*
        MANLRCHECK:
        LDA     STSTATUS
        EOR     PORTC           ;ASSUMES same bit in STSTATUS and PORTC!!!
        AND     #MANLRSEL_M    ;Did the MANLRSEL bit change in PORTC?
        BEQ     MANLROK
        BSET    USINGMANLR,STSTATUS
        SWI                             ; shut down any transmission
        BRSET   MANLRSEL,PORTC,MANLRON
        BCLR   PRIRAD,PORTA
        BCLR   MANLRSEL,STSTATUS
        BRA     MANLROK
        MANLRON:        ;LR input bit is ON and changed
        BSET   PRIRAD,PORTA
        BSET   MANLRSEL,STSTATUS
        MANLROK:
        RTS
        */
        LockInterrupts l;
        bool manLr = (HIGH == digitalRead(LRSWITCH_PIN)) ^ (EEPROM.read(LR_SWITCH_REVERSE_IDX) == 0);
        auto tempStat = host::STSTATUS;
        bool stManLr = (0 != (tempStat & host::MANLRSEL));
        if (stManLr != manLr)
        {
            tempStat |= host::USINGMANLR;
            cw::SWI();
            if (manLr)
            {
                tempStat |= host::MANLRSEL;
                // to match MK1100, set/clear both bits identically
                aux::setInAux(aux::PRIRADR_M | aux::PRIRADL_M);
            }
            else
            {
                tempStat &= ~host::MANLRSEL;
                // match MK-1100
                aux::clearInAux(aux::PRIRADR_M | aux::PRIRADL_M);
            }
            host::STSTATUS = tempStat;
        }
    }
    
    void ARSCLR()
    {
        // This routine's name is carried over from the 6805 code.
        // But its function doesn't match its name. Its purpose is to set
        // the headphone relays according to the current state.

        /*
        ARSCLR: LDA     PORTC
        AND     #{PTTOUTL_M | PTTOUTR_M}
        BNE     OUTLAT                  ;Hold the latch if PTT asserted on either radio
        BRSET   BUFNOTETY,ARSLATCH,OUTLAT ;also hold if CW buffer is not empty
        BCLR    AUDLATCH,ARSLATCH          ;Otherwise clear the latch

        OUTLAT:
        ; Job from here to end is to set AUDIO and RXAUDL outputs in
        ; PORTA to reflect the three bits:
        ;  AUDLATCH (in ARSLATCH)
        ;  HDPHSPLIT (in ARSLATCH)
        ;  PRIRAD  (in PORTA)

        BRSET AUDLATCH,ARSLATCH,DOLATCHED  ;then headphones match

        BRSET HDPHSPLIT,ARSLATCH,SPLITB  ;else split, then one each ear

        ;BRSET PRIRAD,PORTA,LATRADB   ;else following PRIRAD
        ;FOLLOWING PRIRAD XOR'D WITH HPTOGGLE
        LDA    PORTA
        ASLA        ;Line up PRIRAD with HPTOGGLE in STATUS2
        EOR     STATUS2
        AND     #HPTOGGLE_M
        BNE     LATRADB

        LATRADA:
        BSET RXAUDR,PORTA
        BSET RXAUDL,PORTA
        BRA OUTLATDON

        SPLITB:
        BCLR RXAUDR,PORTA
        BSET RXAUDL,PORTA
        BRA OUTLATDON

        DOLATCHED:
        BRSET PRIRAD,PORTA,LATRADA   ;OPPOSITE of PRIRAD

        LATRADB:
        BCLR RXAUDR,PORTA
        BCLR RXAUDL,PORTA

        PADLBUG:
        OUTLATDON:
        RTS
        */
        LockInterrupts l;
        uint8_t auxB = aux::copyAux();
        auto arsLatch = ARSLATCH;
        if ((0 == (auxB & (aux::PTTOUTL_M | aux::PTTOUTR_M)))
            && (0 == (arsLatch & BUFNOTETY)))
            arsLatch &= ~AUDLATCH;

        bool isLeft = (0 != (auxB & aux::PRIRADL_M));
        if (0 == (AUDLATCH & arsLatch))
        {
            if (0 == (HDPHSPLIT & arsLatch))
            {
                bool toggleM = (0 != (host::STATUS2 & host::HPTOGGLE));
                if (isLeft == toggleM) //LATRADA
                    aux::clearInAux(aux::RXAUDR_M | aux::RXAUDL_M);
                else//LATRADB
                    aux::setInAux(aux::RXAUDR_M | aux::RXAUDL_M);
            }
            else
            {   // SPLITB
                aux::setInAux(aux::RXAUDR_M);
                aux::clearInAux(aux::RXAUDL_M );
            }
        }
        else
        {   // DOLATCHED
            if (isLeft) //LATRADB
                aux::setInAux(aux::RXAUDR_M | aux::RXAUDL_M);
            else//LATRADA
                aux::clearInAux(aux::RXAUDR_M | aux::RXAUDL_M);
        }
        ARSLATCH = arsLatch;
    }
}

namespace cw {
    void DIDDLECW();
    void MARKAGN();
    void PR_LTR_CWWAIT();

    /*** CW state transitions
    CW_MARK EQU     0
    CW_SPACE EQU    1
    CW_IDLE   EQU    2
    CW_WAIT   EQU    3
    CW_TUNE   EQU    4      ;output is on TUNE
    CW_BUG    EQU    5      ;doing MARK while dash paddle down

    ***************************
    ;STATE MACHINE
    ;
    ;  CW_IDLE
    ;  The CW machine stays in CW_IDLE until either a paddle is pressed or
    ;  CW appears through the PC port. In either case, the polling loop simply
    ;  puts the desired character in the cw buffer and awaits the TOF interrupt
    ;
    ;  CW_WAIT
    ;  Pressing a CW paddle while in CW_IDLE moves the state here where
    ;  paddle pressing is now ignored. The TOFINT processing will deal
    ;  with the paddle that was pressed. This means that the character
    ;  sent by a paddle press while in CW_IDLE is the one for the paddle
    ;  FIRST pressed.
    ;
    ;  TOFINT processing:
    ;  load NEXTHI/NEXTLOW, turn ON cw output, set state to CW_MARK
    ;
    ;  CW_MARK TOFINT Processing:
    ;  load NEXTHI/NEXTLOW, turn OFF cw output, set state to CW_SPACE
    ;
    ;  CW_SPACE:
    ;  TOFINT processing:
    ;     shift CWCHAR to right and check for termination of current character.
    ;     if so, then load NEXTHI/NEXTLO with LTRLEN and goto IDLE.
    ;     otherwise load NEXTHI/LO with dash or dot time and go to CW_MARK.
    ;
    ;
    ;IAMBIC KEYER:
    ;  Paddles are polled in main line loop. When a paddle is detected
    ;  activated, then a jump table is executed for the 3 states above.
    ******/

    /* all these memory locations touched by both
    ** loop routines and by interrupt service routines.*/
    volatile unsigned long count512usec; // diagnostic counter
    volatile uint16_t NEXTHILO; // time until next routine acts
    volatile uint8_t CWCHAR;

    unsigned long getCount()
    {
        uint8_t oldSREG = SREG;
        noInterrupts();
        unsigned long ret = count512usec;
        SREG = oldSREG;
        return ret;
    }

    bool CwIsOn()
    {        return 0 != (aux::copyAux() & (aux::CWOUTL_M | aux::CWOUTR_M));    }

    void OutputCw(bool Mark)
    {
        if (!Mark)
        {
            aux::clearInAux(aux::CWOUTL_M | aux::CWOUTR_M);
            digitalWrite(CWL_PIN, LOW);
            digitalWrite(CWR_PIN, LOW);
            digitalWrite(CW_SIDETONE_PIN, LOW); // Asymmetry: off, but not on below.
        }
        else
        {
            bool isLeft = (aux::copyAux() & aux::PRIRADL_M) != 0;
            if (isLeft)
            {
                aux::setInAux(aux::CWOUTL_M);
                digitalWrite(CWL_PIN, HIGH);
            }
            else
            {
                aux::setInAux(aux::CWOUTR_M);
                digitalWrite(CWR_PIN, HIGH);
            }
        }
    }

    void SWI() 
    {
        /* The only "software interrupt" available in the Atmel 328 is to
        ** configure a pin as OUTPUT and use its pin change interrupt.
        ** But we have no free pins. (PCINT15 looked hopeful as there is
        ** no physical pin, but, alas, the documentation for the devices
        ** says the interrupt enable for PCINT15 cannot be set.
        **
        ** The effect on the port is that calls to SWI() are IMMEDIATE
        ** rather than deferred until interrupts are off.
        */

        /*
        ABORT:
        JSR    CLRCWBUF ;stack pushes down into CWBUFF, but we're clearing it
        CLR     NEXTHI  ;ditto
        LDA     PORTC
        AND     #{(CWOUTR_M | CWOUTL_M) ^ $FF}
        STA     PORTC       ;CW itself is now off
        BCLR   STONBIT,STSTATUS
        BCLR    SIDETONE,PORTD
        LDA     #CW_IDLE
        STA     CWSTATE
        ;        CLR     NEXTLOW ;Set up so next time we pick on on next TOF
        ;        INC     NEXTLOW ;ditto
        DECA
        STA    NEXTLOW  ;puts a 1 in NEXTLOW cuz CW_IDLE is 2
        IRQINT:
        RTI
        */
        CLRCWBUF();
        OutputCw(false);
        CWSTATE = CW_IDLE;
        NEXTHILO = 1;
    }

    void PR_SPACE()
    {
        /*
        PR_SPACE:       ;just finished a space interval--cw lead is OFF
        LSR     CWCHAR
        LDX     #1
        CPX     CWCHAR
        BNE     MARKAGN
        INC     CWSTATE         ;CW_IDLE next
        LDA     LTRLEN
        STA     NEXTHI
        LDA     LTRLEN+1
        STA     NEXTLOW
        RTI
        */

        auto cwchar = CWCHAR;
        cwchar >>= 1;
        CWCHAR = cwchar;
        if (cwchar == 1)
        {
            CWSTATE += 1;
            NEXTHILO = LTRLEN;
        }
        else
            MARKAGN();
    }

    void MARKAGN()
    {
        /*
        MARKAGN:
        BRCLR   PADDACTI,ARSLATCH,NOTBUG        ;if not doing paddles, no bug
        ;If we are in BUG mode, then go to CW_BUG state
        BRCLR   STATABUG,STATUS2,NOTBUG
        BRCLR   0,CWCHAR,NOTBUG
        LDA     #CW_BUG
        STA     CWSTATE
        JMP     DIDDLECW

        NOTBUG:
        CLR     CWSTATE         ;CW_MARK next
        LDX     DOTLEN
        LDA     #3            ;assume dash
        BRSET   0,CWCHAR,DASHNEXT
        ASRA
        DASHNEXT:
        MUL
        ADD    WEIGHT
        STA    NEXTLOW
        BCC    NC1
        INCX   ;add carry from adding weight
        NC1:    BRCLR  7,WEIGHT,NC2
        DECX   ;negative weighting
        NC2:    STX    NEXTHI
        BRA    DIDDLECW
        */
        bool dash = (0 != (1 & CWCHAR));
        if (!((0 == (ARSLATCH & PADDACTI))
                || (0 == (host::STATABUG & host::STATUS2))
                || !dash))
        {
            CWSTATE = CW_BUG;
        }
        else
        {
            CWSTATE = CW_MARK;
            uint16_t nexthilo = DOTLEN;
            if (dash) // dash
                nexthilo += nexthilo * 2; // TIMES 3
            nexthilo += WEIGHT;
            NEXTHILO = nexthilo;
        }
        DIDDLECW();
    }

    void PR_TUNE()
    {
        /*
        PR_TUNE:
        INC     NEXTLOW         ;poll CWPTROU rapidly
        LDX     CWPTROU
        CMPX    CWPTRIN         ;Letters to send?
        BNE     TUNE_OFF        ;YES
        PR_BUG:                         ;NOTHING TO DO
        RTI

        TUNE_OFF:       ;finish the TUNE function and send some letters
        LDA     PORTC
        AND     #{(CWOUTR_M | CWOUTL_M) ^ $FF}
        STA     PORTC       ;CW itself is now off
        BRA     MORELETTERS
        */
        NEXTHILO = 1;
        if (CWPTROU == CWPTRIN)
            return;

        OutputCw(false);
        PR_LTR_CWWAIT(); // close enough to MORELETTERS to work
    }

    void PR_LTR_CWWAIT()
    {
        /*
        PR_CWWAIT:
        PR_LTR:        ;just finished sending an entire character--cw lead is OFF
        LDX     CWPTROU  ;ACC is OUT index
        CMPX    CWPTRIN          ;Compare OUT with IN
        BNE     MORELETTERS
        ;This is happening every 512 usec while keyer is idle
        BCLR    BUFNOTETY,ARSLATCH       ;Flag that buffer is empty
        BCLR    FIFOFULL,STSTATUS    ;Safety: Tell PC we have buffer available
        BCLR XMITTING,STSTATUS ;Tell PC we're not transmitting now
        INC     NEXTLOW         ;result is 1 and we run immediately on TOFINT
        INTDONE:
        RTI                     ;IDLE now


        MORELETTERS:
        LDA     CWBUFF,X        ;get next letter
        STA     CWCHAR
        TXA
        INCA
        AND     #$7             ;Circulate in 8 bytes
        STA     CWPTROU
        BCLR    FIFOFULL,STSTATUS    ;Tell PC we have buffer available
        LDA     CWCHAR
        BEQ     WORDSPACE
        CMP  #1    ;Just a space?
        BEQ  JUSTSPACE
        CMP     #$C0             ;New Speed?
        BCS     MARKAGN
        LSLA                   ;bottom 6 bits of command are speed
        LSLA

        STA     DOTLEN
        CLR     KNOBSPD
        CLR     LTRLEN        ;high order forced zero
        ASLA
        STA     LTRLEN+1      ;double to low order
        ROL     LTRLEN
        BRA     PR_LTR

        JUSTSPACE:
        LDA  DOTLEN
        STA  NEXTLOW
        RTI     ;exit, state unchanged

        WORDSPACE:
        ;Make a wordspace=2.0 * LTRLEN       CWSTATE unchanged
        LDA      LTRLEN+1      ;low order
        LDX      LTRLEN        ;high order
        LSLA
        ROLX
        STX      NEXTHI
        STA      NEXTLOW
        ;exit with state unchanged
        RTI
        */
    AroundAgain:
        {
            auto cwptrou = CWPTROU;
            if (cwptrou == CWPTRIN)
            {
                ARSLATCH &= ~BUFNOTETY;
                host::STSTATUS &= ~(host::FIFOFULL | host::XMITTING);
                NEXTHILO = 1;
                return;
            }
            host::STSTATUS &= ~host::FIFOFULL;
            auto cwchar = CWBUFF[cwptrou++];
            CWCHAR = cwchar;
            cwptrou &= 7;
            CWPTROU = cwptrou;
            if (cwchar == 0)    // wordspace
                NEXTHILO = LTRLEN << 1;
            else if (cwchar == 1)   // letter space
                NEXTHILO = DOTLEN;
            else if (cwchar >= 0xC0)
            {   // deferred speed
                uint16_t dotlen = (uint8_t) (cwchar << 2);
                DOTLEN = (uint8_t)dotlen;
                KNOBSPD = 0;
                LTRLEN = dotlen << 1;
                // this is the only one that goes back around
                goto AroundAgain;
            }
            else
                MARKAGN();
        }
    }
    
    void PR_MARK()
    {
        /*
        PR_MARK:        ;just finished a mark interval--cw lead is ON
        INC     CWSTATE         ;CW_SPACE next
        LDA     DOTLEN        ;pick up dot spacing
        SUB     WEIGHT          ;adjust for character weighting
        STA     NEXTLOW
        CLR     NEXTHI

        DIDDLECW:
        */
        CWSTATE += 1;
        uint16_t nexthilo = DOTLEN;
        nexthilo -= WEIGHT;
        NEXTHILO = nexthilo;
        DIDDLECW();
    }

    void DIDDLECW()
    {
        /*
        DIDDLECW:
        ;get CW port location
        LDA     #CWOUTR_M
        BRCLR   PRIRAD,PORTA,RADIO_B    ;Input says other radio is primary now?
        LSRA
        RADIO_B ;A is bit mask for use in PORTB
        EOR     PORTC
        STA     PORTC

        ;setup sidetone
        AND     #{CWOUTR_M | CWOUTL_M}
        BNE     CWISON
        BCLR    SIDETONE,PORTD
        BCLR    STONBIT,STSTATUS
        RTI

        CWISON: ; branch out here if no sidetone on PC sent code...
        BRCLR    PADDACTI,ARSLATCH,MACHINECW         ;branch if paddles NOT cause of CW
        BRCLR    STONPDLS,ARSLATCH,STONLOAD
        RTI
        MACHINECW:
        BRCLR   STMACHINE,ARSLATCH,STONLOAD    ;branch if ST enabled for machine sent
        RTI

        STONLOAD:
        BSET    SIDETONE,PORTD
        BSET    STONBIT,STSTATUS
        LDA     STONCOUNT
        STA     STCOUNT
        RTI
        */
        bool mark = CwIsOn();
        OutputCw(!mark);
        if (!mark)
        {   // Turn on sidetone if supposed to
            auto arsLatch = ARSLATCH;
            if (0 != (arsLatch & PADDACTI) ?
                0 == (arsLatch & STONPDLS) :
                0 == (arsLatch & STMACHINE))
                digitalWrite(CW_SIDETONE_PIN, HIGH);
        }
    }
  
    void PR_BUG()
    { }

    // CW buffered from PC
    void CLRCWBUF()
    {   // INTERRUPTS MUST BE OFF
        /*
        CLRCWBUF:
        BCLR    FIFOFULL,STSTATUS ;let host download
        BCLR XMITTING,STSTATUS ;tell host not xmitting
        CLR     CWPTRIN  ;Show buffer empty
        CLR     CWPTROU  ; ""
        RTS
        */
        host::STSTATUS &= ~(host::FIFOFULL & host::XMITTING);
        CWPTRIN = CWPTROU = 0;
    }

    typedef void(*CwFunction_t)();

    CwFunction_t const STATEDIS[CW_NUMSTATES] =
    {        &PR_MARK, &PR_SPACE, &PR_LTR_CWWAIT, &PR_LTR_CWWAIT, &PR_TUNE, &PR_BUG    };

    // 512 usec interrupt routine
    void on512usec()
    {
        count512usec += 1;
        if (ARSLATCH & PADDACTI)
        {
            auto pttcnthilo = ptt::PttCntHiLo;
            pttcnthilo -= 1;
            ptt::PttCntHiLo = pttcnthilo;
            if (pttcnthilo == 0)
                ptt::SetPttOff();
         }
        if (--NEXTHILO == 0)
            (*STATEDIS[CWSTATE])();
    }

    void initVars()
    {
       DOTLEN = 117u; // Init to 20WPM. units 512usec
       LTRLEN = 117u << 1; // ditto -- letter length is double dot length
       PTTIDL = 24; // 24/64 second PTT
       SWI();
    }

#if (F_CPU == 16000000ul)
    const uint8_t USEC_512_COUNT = 0x80u; // 256 counts at 2uSec
#elif (F_CPU == 8000000ul)
    const uint8_t USEC_512_COUNT = 0x40u; // 128 counts at 4uSec
#else
#error Only support 16MHz and 8Mhz clocks
#endif

#if defined  (__AVR_ATmega328P__)
// target hardware is Pro Mini, which is 328P
    uint8_t tcnt2Setting()
    {   return static_cast<uint8_t>(0x100u - USEC_512_COUNT); //counts up from here, interrupt at == 0
    }

    ISR(TIMER2_OVF_vect)
    {
        TCNT2 = tcnt2Setting();
        on512usec();
    }

    void setup()
    {
        initVars();
        LockInterrupts lock;
        /* Normal timer operation.*/
        TCCR2A = 0x00;
        /* The timer counter register.         */
        TCNT2 = tcnt2Setting();
        TCCR2B = 0x04; // 64 prescale, which is 2usec per TCNT2 @ 16MHz
        /* Enable the timer overflow interrupt. */
        TIMSK2 = 0x01;
        TIFR2 = 1; // Clear any current overflow flag
    }
#elif defined (__AVR_ATmega32U4__)
// some prototyping done on Pro Micro, 32U4
    ISR(TIMER4_OVF_vect)
    {
        on512usec();
    }

    void setup()
    {
        initVars();
        LockInterrupts lock;
        TCCR4B = 0;
        TCCR4A = 0;
        TCCR4C = 0;
        TCCR4D = 0;
        TCCR4E = 0;
        TCCR4B = (1 << CS42) | (1 << CS41) | (1<< CS40) | (1 << PSR4); // prescale 1/64 and reset
        OCR4C = USEC_512_COUNT;
        TIFR4 = (1 << TOV4);
        TCNT4 = 0;
        TIMSK4 = (1 << TOIE4);
    }
#else 
#error Only support ATmega328P and, some functions on ATmega32U4
#endif

    const int SPD_MIN =  38; //about 60WPM
    const int SPD_MAX =  180; //about 13WPM
    const int SPD_FILTERCOUNT  =  3; //number of consecutive change required
    const int SPD_KNOB_RATIO = 7; // conversion from ADC units to DOTLEN
    const int SPD_MIN_KNOB_CHANGE = 20;

    void checkKnob() // called with interrupts()
    {
        static unsigned long lastUpdate;
        unsigned long update = cw::getCount() / 32; // 32 * 512uSec. about 60Hz
        if (update != lastUpdate)
        {
            lastUpdate = update;
            // read the knob
            analogReference(DEFAULT);
            int SpdCnt = analogRead(SPEEDPOT_PIN);
            if (EEPROM.read(KNOB_REVERSE_IDX) == 0)
                SpdCnt = 1024 - SpdCnt;
            static int SpdPrv; // previous read
            static char SpdFilter;
            int cmp = SpdCnt - SpdPrv;
            if (cmp < 0)
                cmp = -cmp;
            if (cmp >= SPD_MIN_KNOB_CHANGE)
            {
                if (SpdFilter++ >= SPD_FILTERCOUNT)
                {
                    SpdPrv = SpdCnt;
                    SpdCnt /= SPD_KNOB_RATIO; // convert to 68HC705 units--about..
                    SpdCnt += SPD_MIN;
                    if (SpdCnt > SPD_MAX)
                        SpdCnt = SPD_MAX;
                    KNOBSPD = SpdCnt;
                    DOTLEN = SpdCnt;
                    LTRLEN = SpdCnt << 1;
                }
            }
            else
                SpdFilter = 0;
        }
    }

    uint8_t QUERYPDL()
    {
        /*
        QUERYPDL:
        LDA    #4
        BRCLR  DASHPDL,PORTD,LOADDASH ;-- set carry if no branch
        BIL     LOADOPPO                 ; carry unchanged
        RTS          ;False alarm --carry set

        LOADDASH:
        ORA    #2                            ;Load a dash
        LOADOPPO:
        STA    CWCHAR
        CLC    ;Carry is CLEAR if we did a set
        NODICE:
        RTS

        */
        /* porting note: old code alters CWCHAR in place.
        ** C port does not. We instead return proposed CWCHAR:
        ** 0 for no paddle, 4 for dot and 6 for dash,
        ** (which is left-shifted by 1, like the old code)
        */
        if (digitalRead(CWDASH_PIN) != LOW)
        {
            if (digitalRead(CWDOT_PIN) != LOW)
                return 0;
            return 4;
        }
        else
            return 6;
    }

    void PADLMARK(uint8_t pd)
    {
/*
        PADLMARK:
        ;PADDLE(S) DOWN WHILE MARKING
        ;If we're sending a dash, and the dot paddle is down, then load a dot
        ;and the other way around, too.
        ;must preseve the low bit of CWCHAR in the process
        ;guarantee none in type ahead buffer
        JSR    CLRCWBUF
        LDA    #4                ;paddle dispatch routines assume 4 in ACC
        BRCLR  0,CWCHAR,DOINGDOT
        INCA   ;preserve fact we're sending a dash now
        BIL  LOADOPPO      ;check the dot paddle
NOLOAD: RTS
DOINGDOT:
        BRSET  DASHPDL,PORTD,NOLOAD ;Load a DASH
LOADDASH:
        ORA    #2                            ;Load a dash
LOADOPPO:
        STA    CWCHAR
        CLC    ;Carry is CLEAR if we did a set
NODICE:
        RTS
*/
        CLRCWBUF();
        if (CWCHAR & 1)
        {   // doing dash
            if (LOW != digitalRead(CWDOT_PIN))
                return; // dash paddle--do nothing
            CWCHAR = 5; // dot added to send
        }
        else
        {   // doing dot
            if (LOW != digitalRead(CWDASH_PIN))
                return; // dot paddle--do nothing
            CWCHAR = 6; // dash added to send
        }
    }

    void PADLSPAC(uint8_t pd)
    {
        /*
 PADLSPAC:
      ;if we're in the final msec, then set CWCHAR according the closed paddle,
      ;otherwise do the same thing as in mark state
        TST    NEXTHI
        BNE    PADLMARK   ;not in final time
        ;TXA       ; LDA    #2  NEED A BYTE!!!  CW_MARK=1 *2=2  KLUDGE!!!
        CPX    NEXTLOW
        BLO    PADLMARK   ;not in final time
        INCX      ; LDA  #3
        CPX    CWCHAR      ;already loaded following character?
        BCS    NODICE      ;branch on yes
        ;fall through cuz following char not loaded into CWCHAR yet
QUERYPDL:
...
NODICE:
        RTS
       */

        if (NEXTHILO > 1)
            PADLMARK(pd);
        else if (CWCHAR <= 3)
        {
            /* porting note.. old code called QUERYPDL (again.)
            ** We just use its previously just-obtained result  */
            CWCHAR = pd;
        }
    }

    void PADIDLE(uint8_t paddles)
    {
        /*
        PADIDLE:       ;PADDLE DOWN DURING CW_IDLE
        ;are we doing auto letter space?
        BRCLR   AUTOLTR,ARSLATCH,AUTOLTRYES          ;BR on yes
        SWI           ;start time and state NOW (on next timer interrupt)
        AUTOLTRYES:         ;otherwise leave NEXTIME alone
        ;acc is 4 for dot down, 6 for dash down
        LSRA                     ;Into the correct position
        STA    CWBUFF
        JSR    CLRCWBUF          ;clear the RAM CW buffer
        INC    CWPTRIN           ;will go on next timer interrupt
        ;which will be right away (on SWI) or
        ;at end of letter space
        INC     CWSTATE          ;go to CW_WAIT

        SETUPPTTDELAY:
        */
        if (0 != (AUTOLTR & ARSLATCH))
            SWI();
        CWBUFF[0] = paddles >> 1;
        CLRCWBUF();
        CWPTRIN += 1;
        CWSTATE += 1;
        ptt::SETUPPTTDELAY();
    }

    void NOOP(uint8_t)
    {    }

    typedef void(*PadlDisp_t)(uint8_t);
    PadlDisp_t const PADLDISP[CW_NUMSTATES] =
        { &PADLMARK, &PADLSPAC, &PADIDLE, &NOOP, &PADLMARK, &NOOP};

    void PDLCHECK() // called with noInteerupts
    {
        /*
 PDLCHECK:
        LDX     CWCHAR            ;save current cw character
        JSR     QUERYPDL
        BCS     PDLCHECKOFF ;Branch on no paddles down
        ;acc is now 4 for dot down, 6 for dash down
        BRSET   PADDACTI,ARSLATCH,PADSRUN  ;br on paddles not active
        SWI                 ;STOP everything
PADSRUN:
        STX     CWCHAR      ;restore the current character lost by querypdl
        LDX     CWSTATE
        ASLX
        JSR     PADLDISP,X
        BSET    PADDACTI,ARSLATCH
        LDA  PTTIDL     ;Whenever we see paddles down, start PTT count down 512 x half-msec
        STA  PTTCNTHI
        CLR             PTTCNTLO
        LSR             PTTCNTHI
        ROR             PTTCNTLO
        LSR             PTTCNTHI
        ROR             PTTCNTLO
        LSR             PTTCNTHI
        ROR             PTTCNTLO
        RTS

        PDLCHECKOFF:
        LDA     CWSTATE
        CMP     #CW_BUG
        BNE     PDLEXIT
        SWI             ;turn OFF CW on key up
        PDLEXIT:
        RTS
        */

        /* Porting note: old code saves and (sometimes) restores CWCHAR
        ** across call to QUERYPDL. New QUERYPDL only returns proposed CWCHAR
        ** which is passed to the state processing routines, which might or
        ** might not use the current paddle state, and the CWCHAR is
        ** (like in the old code) the same as upon entry.
        */
        uint8_t paddles = QUERYPDL();
        LockInterrupts lock;
        if (paddles != 0)
        {
            if (0 == (PADDACTI & ARSLATCH))
                SWI();
            (*PADLDISP[CWSTATE])(paddles);
            ARSLATCH |= PADDACTI;
            uint16_t pttidl = PTTIDL;
            pttidl <<= 5;
            ptt::PttCntHiLo = pttidl;
        }
        else
        {
            if (CW_BUG == CWSTATE)
                SWI();
        }
    }

    void loop()
    {
        if (EEPROM.read(KNOB_EPROM_IDX) != 0)
            checkKnob();
        PDLCHECK();
    }
}

namespace ptt {
    const int FTPTTONCNT = 10; // debounce--number of times we see FTPTT off before turning it off
    uint8_t FTPTTONST;

    void setup()
    {
        PTTDELAY = 0;
        auto del = EEPROM.read(PTT_DELAY_IDX);
        if (del != 255u)
            PTTDELAY = del;
    }

    void loop()
    {
        /*
FTPTTCHECK:      ;CHECK FOOT PTT SWITCH INPUT
    BRSET FTPTTIN,PORTC,FTPTTISOFF
    ;foot switch PTT is ON
    JSR SETPTTON
    LDA #FTPTTONCNT
    STA FTPTTONST
    RTS
FTPTTISOFF:
    TST FTPTTONST
    BEQ FTPTTDONE
    DEC FTPTTONST
    BNE FTPTTDONE
    JSR SETPTTOFF
FTPTTDONE:
    RTS
         */
        LockInterrupts l;
        if (digitalRead(FOOTTIP_PIN) == LOW)
        {
            SetPttOn();
            FTPTTONST = FTPTTONCNT;
        }
        else if (FTPTTONST != 0)
        {
            if (--FTPTTONST == 0)
                SetPttOff();
        }
    }

    void SetPttOn()
    {
        bool isLeft = (aux::copyAux() & aux::PRIRADL_M) != 0;
        uint8_t toSet = isLeft ?
            aux::PTTOUTL_M : aux::PTTOUTR_M;
        aux::setInAux(toSet);
        digitalWrite(isLeft ? PTTL_PIN : PTTR_PIN, HIGH);
    }

    void SetPttOff()
    {
        aux::clearInAux(aux::PTTOUTL_M | aux::PTTOUTR_M);
        digitalWrite(PTTL_PIN, LOW);
        digitalWrite(PTTR_PIN, LOW);
    }

    void SETUPPTTDELAY()
    {   // call with ********** noInterrupts *****************
        /*
SETUPPTTDELAY:
    ; if we have a PTT delay, and PTT is OFF, then that's how long we must wait
        LDA  PTTDELAY
        BEQ  PTTDELDONE; Don't have any
        LDA PORTC; LOAD BOTH PTT outputs  at once...
        AND #{PTTOUTL_M | PTTOUTR_M}
    BNE  PTTDELDONE; already on
        JSR  SETPTTON
        LDA  PTTDELAY
        STA  NEXTLOW
        CLR  NEXTHI
    PTTDELDONE :
    RTS
    */
        if (PTTDELAY)
        {
            if (0 != (aux::copyAux() & (aux::PTTOUTL_M | aux::PTTOUTR_M)))
                return;
            SetPttOn();
            cw::NEXTHILO = PTTDELAY;
        }
    }
}

namespace diag {
  /* The Arduino serial port in normal MK-1101 operation is wired to nothing,
   *  and unused. The code in this namespace monitors that serial port
   *  (9600 bauds) and, should it be connected, executes certain commands.
   */
    const int HOST_BUFFER_LENGTH = 80;
    char fromHostbuffer[HOST_BUFFER_LENGTH + 1]; // +1 so we can add trailing null
    int charsInBuf = 0;
    bool pauseBetweenTests;
    void SelfTest();
    void SOUNDTEST();
    void SPEEDTEST();

    void pause()
    {
        if (!pauseBetweenTests)
            return;
        Serial.println(F(" ...any key to continue"));
        while (!Serial.available());
        Serial.read();
        return;
    }

    uint8_t asciiToBin(const char *p)
    {
        uint8_t s = 0;
        while (isdigit(*p))
        {
            s *= 10;
            s += *p++ - '0';
        }
        return s;
    }

    void ProcessHostCommand()
    {
        bool echo = true;
        static const char printOn[]  = "printOn";
        static const char printOff[]  = "printOff";
        static const char printCount[] = "printCount";
        static const char potLR[] = "potLR";
        static const char sound[] = "sound";
        static const char knobSpd[]  = "knobSpd=";
        static const unsigned knobSpdLen = -1 + sizeof(knobSpd) / sizeof(knobSpd[0]);
        static const char DCDup[]  = "DCDup";
        static const char DCDdown[]  = "DCDdown";
        static const char selfTest[]  = "selfTest";
        static const char knobOn[] = "knobOn";
        static const char knobOff[] = "knobOff";
        static const char hostDump[] = "hostDump";
        static const char knob[] = "readKnob";
        static const char knobRev[] = "knobRev";
        static const char knobFwd[] = "knowFwd";
        static const char lrNormal[] = "LRnormal";
        static const char lrRev[] = "LRrev";
        static const char pttDel[] = "pttDel=";
        static const char paddles[] = "paddles";
        static const char pauseOn[] = "pauseOn";
        static const char pauseOff[] = "pauseOff";
        static const unsigned pttDelLen = -1 + sizeof(pttDel);
        if (strcmp(fromHostbuffer, printOn) == 0)
            printCommands = true;
        else if (strcmp(fromHostbuffer, printOff) == 0)
            printCommands = false;
        else if (strcmp(fromHostbuffer, printCount) == 0)
        {
            Serial.print(F("512 usec counter: "));
            Serial.println(cw::getCount());
            echo = false;
        }
        else if (strncmp(fromHostbuffer, knobSpd, knobSpdLen) == 0)
        {   // simulate turning the pot
            uint8_t s = asciiToBin(fromHostbuffer + knobSpdLen);
            KNOBSPD = s;
            cw::DOTLEN = s;
            cw::LTRLEN = s << 1;
        }
        else if (strncmp(fromHostbuffer, pttDel, pttDelLen) == 0)
        {
            uint8_t s = asciiToBin(fromHostbuffer + pttDelLen);
            EEPROM.write(PTT_DELAY_IDX, s);
            ptt::PTTDELAY = s;
        }
        else if (strcmp(fromHostbuffer, DCDup) == 0)
            digitalWrite(DCD_PIN, HIGH);
        else if (strcmp(fromHostbuffer, DCDdown) == 0)
            digitalWrite(DCD_PIN, LOW);
        else if (strcmp(fromHostbuffer, selfTest) == 0)
            SelfTest();
        else if (strcmp(fromHostbuffer, knobOn) == 0)
            EEPROM.write(KNOB_EPROM_IDX, 1);
        else if (strcmp(fromHostbuffer, knobOff) == 0)
            EEPROM.write(KNOB_EPROM_IDX, 0);
        else if (strcmp(fromHostbuffer, knobRev) == 0)
            EEPROM.write(KNOB_REVERSE_IDX, 0);
        else if (strcmp(fromHostbuffer, knobFwd) == 0)
            EEPROM.write(KNOB_REVERSE_IDX, 1);
        else if (strcmp(fromHostbuffer, lrNormal) == 0)
            EEPROM.write(LR_SWITCH_REVERSE_IDX, 1);
        else if (strcmp(fromHostbuffer, lrRev) == 0)
            EEPROM.write(LR_SWITCH_REVERSE_IDX, 0);
        else if (strcmp(fromHostbuffer, hostDump) == 0)
        {
            echo = false;
            Serial.print("EEPROM: KnobEnabled=");
            Serial.print((int)EEPROM.read(KNOB_EPROM_IDX));
            Serial.print(" PTT delay="); Serial.print((int)EEPROM.read(PTT_DELAY_IDX));
            Serial.print(" knobRev="); Serial.print((int)EEPROM.read(KNOB_REVERSE_IDX));
            Serial.print(" LRreverse="); Serial.print((int)EEPROM.read(LR_SWITCH_REVERSE_IDX));
            Serial.print(" DOTLEN="); Serial.print(cw::DOTLEN);
            Serial.print(" LTRLEN="); Serial.println(cw::LTRLEN);
        }
        else if (strcmp(fromHostbuffer, knob) == 0)
        {
            analogReference(DEFAULT);
            int spd = analogRead(SPEEDPOT_PIN);
            Serial.print("Pot="); Serial.println(spd);
            echo = false;
        }
        else if (strcmp(fromHostbuffer, potLR) == 0)
        {
            SPEEDTEST();
        }
        else if (strcmp(fromHostbuffer, sound) == 0)
        {
            SOUNDTEST();
        }
        else if (strcmp(fromHostbuffer, paddles) == 0)
        {
            Serial.print("Dot=");
            Serial.print(digitalRead(CWDOT_PIN) == LOW ? "on" : "off");
            Serial.print(" Dash=");
            Serial.println(digitalRead(CWDASH_PIN) == LOW ? "on" : "off");
        }
        else if (strcmp(fromHostbuffer, pauseOn) == 0)
            pauseBetweenTests = true;
        else if (strcmp(fromHostbuffer, pauseOff) == 0)
            pauseBetweenTests = false;
        else
            echo = false; // unknown commands do NOT echo
        // ...so TXD/RXD can be looped back when diag port disconnected.
        if (echo)
            Serial.println(fromHostbuffer);
    }

    void incomingChar(uint8_t c)
    {
        if (printCommands)
        {
            Serial.print(F("incoming Char: 0x"));
            Serial.println(c, HEX);
        }
    }

    void CheckSerial()
    {
        while (Serial.available() > 0)
        {
            char input = Serial.read();
            static bool overflow = false;
            bool inputIsNewline = (input == '\r') || (input == '\n');
            if (!inputIsNewline) // not a carriage return
            {
                if (charsInBuf >= HOST_BUFFER_LENGTH)
                {
                    overflow = true;
                    charsInBuf = 0;
                }
                if (!overflow)
                {
                    fromHostbuffer[charsInBuf] = input;
                    charsInBuf++;
                }
            }

            if (inputIsNewline)
            {
                if (charsInBuf > 0)
                {
                    fromHostbuffer[charsInBuf] = 0;
                    ProcessHostCommand();
                }
                overflow = false;
                charsInBuf = 0;
            }
        }
    }

    uint8_t const P12Lights[3]  = { aux::PRIRADR_M, aux::CWOUTR_M, aux::PTTOUTR_M };
    uint8_t const P13Lights[3]  = { aux::PRIRADL_M, aux::CWOUTL_M, aux::PTTOUTL_M };

    void LIGHTUP_Px(const uint8_t *p, int shift)
    {
        Serial.println(F("L/R, then CW, then PTT"));
        pause();
        for (unsigned i = 0; i < 3; i++)
        {
            aux::toggleInAux(*p);
            aux::loop();
            delay(512);
            aux::toggleInAux(*p++);
        }
        Serial.println(F("A3 down to A0"));
        pause();
        for (unsigned i = 0; i < 4; i++)
        {
            aux::AuxBoardAnt = 1 << ((3 - i) + shift);
            aux::loop();
            delay(512);
        }
    }

    void LoopbackTests();
    void RelayTests();

    void SelfTest()
    {
        interrupts();
        for (;;)
        {
            // initialize
            digitalWrite(CW_SIDETONE_PIN, LOW);
            aux::AuxBoardAnt = 0;
            aux::clearInAux(0xffu);
            aux::loop();
            delay(128);

            Serial.println(F("Sidetone test"));
            pause();
            SOUNDTEST();

            SPEEDTEST();
            pause();
            LoopbackTests();

            Serial.println(F("Aux LED test on LPT right"));
            aux::setInAux(aux::RXAUDL_M); // turn ON L headphone relay (cosmetic)
            LIGHTUP_Px(P12Lights, 0);
            aux::AuxBoardAnt = 0;
            aux::clearInAux(0xff);
            aux::loop();
            delay(1000);

            Serial.println(F("Aux LED test on LPT left"));
            LIGHTUP_Px(P13Lights, 4);
            aux::AuxBoardAnt = 0;
            aux::clearInAux(0xff);
            aux::loop();
            delay(1000);

            Serial.println(F("Aux Relay tests"));
            RelayTests();
        }
    }

    void SOUNDTEST()
    {
        digitalWrite(CW_SIDETONE_PIN, HIGH);
        delay(500);
        digitalWrite(CW_SIDETONE_PIN, LOW);
        delay(500);
     }

    void SPEEDTEST()
    {
        Serial.print("Speed pot is at ");
        analogReference(DEFAULT);
        int s = analogRead(SPEEDPOT_PIN);
        Serial.print(s);
        Serial.println(" of 1024.");
        delay(512);

        int lr = (digitalRead(LRSWITCH_PIN) != LOW) ^ (EEPROM.read(LR_SWITCH_REVERSE_IDX) == 0);
        Serial.print("Left/Right switch is set to ");
        if (lr == HIGH)
            Serial.println("Left");
        else
            Serial.println("Right");
        delay(512);
    }

    int readFootRing()
    {
        analogReference(DEFAULT);
        int fr = analogRead(FOOTRING_PIN);
        if (fr > 1024 / 2)
            return HIGH;
        else
            return LOW;
    }

    void loopback(int pinNumberOut, int pinNumberIn, uint8_t pinInAux)
    {
        const static char CWL_ERROR[]  = "Error: CW-L not OK on footswitch ring. **************************";
        const static char CWR_ERROR[]  = "Error: CW-R not OK on paddle dash. **************************";
        const static char PTTL_ERROR[]  = "Error: PTT-L not OK on footswitch tip. **************************";
        const static char PTTR_ERROR[]  = "Error: PTT-R not OK on paddle dot. **************************";
        // set exactly one output HIGH
        digitalWrite(CWL_PIN, pinNumberOut == CWL_PIN ? HIGH : LOW);
        digitalWrite(CWR_PIN, pinNumberOut == CWR_PIN ? HIGH : LOW);
        digitalWrite(PTTL_PIN, pinNumberOut == PTTL_PIN ? HIGH : LOW);
        digitalWrite(PTTR_PIN, pinNumberOut == PTTR_PIN ? HIGH : LOW);
        if (pinInAux)
            aux::setInAux(pinInAux);
        aux::loop();
        delay(2);

        // Read the 4 inputs. Only one of them should be low (except FOOTRING)

        // FOOTRING_PIN is an analog input pin ONLY and with external pullup.
        int footRing = readFootRing();
        bool error(false);
        if (footRing != ((pinNumberIn == FOOTRING_PIN) ? LOW : HIGH))
        {
            Serial.println(CWL_ERROR);
            error = true;
            pause();
        }
        if (digitalRead(CWDASH_PIN) != ((pinNumberIn == CWDASH_PIN) ? LOW : HIGH))
        {
            Serial.println(CWR_ERROR);
            error = true;
            pause();
        }
        if (digitalRead(FOOTTIP_PIN) != ((pinNumberIn == FOOTTIP_PIN) ? LOW : HIGH))
        {
            Serial.println(PTTL_ERROR);
            error = true;
            pause();
        }
        if (digitalRead(CWDOT_PIN) != ((pinNumberIn == CWDOT_PIN) ? LOW : HIGH))
        {
            Serial.println(PTTR_ERROR);
            error = true;
            pause();
        }
        if (pinNumberOut > 0)
            digitalWrite(pinNumberOut, LOW);
        if (pinInAux)
            aux::clearInAux(pinInAux);
        if (error)
            delay(1000);
        else
            Serial.println("OK");
    }

    void LoopbackTests()
    {
        aux::AuxBoardAnt = 0;
        aux::clearInAux(0xffu);
        aux::loop();

        Serial.println(F("Checking all CW+PTT lines output off"));
        pause();
        loopback(-1, -1, 0);

        Serial.println(F("CW Left"));
        pause();
        loopback(CWL_PIN, FOOTRING_PIN, aux::CWOUTL_M);

        Serial.println(F("PTT Left"));
        pause();
        loopback(PTTL_PIN, FOOTTIP_PIN, aux::PTTOUTL_M);

        Serial.println(F("CW Right"));
        pause();
        loopback(CWR_PIN, CWDASH_PIN, aux::CWOUTR_M);

        Serial.println(F("PTT Right"));
        pause();
        loopback(PTTR_PIN, CWDOT_PIN, aux::PTTOUTR_M);
    }

    void RelayTests()
    {
        aux::AuxBoardAnt = 0;
        aux::clearInAux(0xffu);
        aux::loop();
        Serial.println(F("Relays and RX-L and RX-R LEDs are off."));
        delay(2000);

        Serial.println(F("RX-L LED flash (Right A0+A1 source)"));
        pause();
        aux::AuxBoardAnt = 3;
        for (int i = 0; i < 20; i++)
        {
          aux::loop();
          delay(100);
          aux::AuxBoardAnt ^= 3;
        }

        Serial.println(F("RX-L relay activate. LED OFF"));
        pause();
        aux::setInAux(aux::RXAUDL_M);
        for (int i = 0; i < 20; i++)
        {
          aux::loop();
          delay(100);
          aux::AuxBoardAnt ^= 3;
        }

        Serial.println(F("RX-R relay activate. R LED FLASH (Right A0+A1 source)"));
        pause();
        aux::setInAux(aux::RXAUDR_M);
        for (int i = 0; i < 20; i++)
        {
          aux::loop();
          delay(100);
          aux::AuxBoardAnt ^= 3;
        }

        Serial.println(F("RX-L relay deactivate. both flash (Right A0+A1 source)"));
        pause();
        aux::clearInAux(aux::RXAUDL_M);
        for (int i = 0; i < 20; i++)
        {
          aux::loop();
          delay(100);
          aux::AuxBoardAnt ^= 3;
        }
        aux::AuxBoardAnt = 0;
        aux::clearInAux(0xffu);
        aux::loop();
    }
}
