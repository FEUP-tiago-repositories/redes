// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

#define FLAG 0x7E
#define A_TX 0x03
#define C_SET 0x03
#define C_UA 0x07
#define A_RX 0x01
#define C_DISC 0x0B

// Special byte used for stuffing
#define ESC 0x7D

// Data frames
#define C_I0 0x00
#define C_I1 0x80

// Ready to receive frame 0 or 1
#define C_RR0 0xAA
#define C_RR1 0xAB

// Request frame 0 or 1 again
#define C_REJ0 0x54
#define C_REJ1 0x55


static int alarmCount = 0;
static int alarmEnabled = FALSE;

// Number of the next frame to send
static int txSequence = 0;

// Number of the next frame expected
static int rxSequence = 0;

static LinkLayer params;      // save timeout and nRetransmissions
static int nFramesSent = 0, nRetransmitted = 0, nTimeouts = 0;

static void alarmHandler(int signal)
{
    alarmEnabled = TRUE;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
}

// Calculate BCC2 from the data bytes
static unsigned char calculateBCC2(const unsigned char *buf, int size)
{
    unsigned char bcc2 = 0;

    for (int i = 0; i < size; i++)
    {
        bcc2 = bcc2 ^ buf[i];
    }

    return bcc2;
}

// Copy data and replace special bytes
static int stuffBytes(const unsigned char *buf, int size, unsigned char *stuffed)
{
    int j = 0;

    for (int i = 0; i < size; i++)
    {
        if (buf[i] == FLAG || buf[i] == ESC)
        {
            stuffed[j] = ESC;
            j++;

            stuffed[j] = buf[i] ^ 0x20;
            j++;
        }
        else
        {
            stuffed[j] = buf[i];
            j++;
        }
    }

    return j;
}

// Build an information frame
static int buildInformationFrame(const unsigned char *buf, int size,
                                 unsigned char *frame)
{
    unsigned char data[MAX_PAYLOAD_SIZE + 1];
    unsigned char control;

    // Choose frame number 0 or 1
    if (txSequence == 0)
    {
        control = C_I0;
    }
    else
    {
        control = C_I1;
    }

    // Build the header
    frame[0] = FLAG;
    frame[1] = A_TX;
    frame[2] = control;
    frame[3] = A_TX ^ control;

    // Copy the data
    for (int i = 0; i < size; i++)
    {
        data[i] = buf[i];
    }

    // Add BCC2 after the data
    data[size] = calculateBCC2(buf, size);

    // Apply stuffing to the data and BCC2
    int stuffedSize = stuffBytes(data, size + 1, &frame[4]);

    // Add the final flag
    frame[4 + stuffedSize] = FLAG;

    return 5 + stuffedSize;
}

static int readFrame(unsigned char addr, unsigned char ctrl)
{
    enum { S_START, S_FLAG, S_A, S_C, S_BCC, S_END } state = S_START;
    unsigned char byte;

    while (state != S_END)
    {
        if (alarmEnabled) return 1;
        int r = readByteSerialPort(&byte);
        if (r < 0)
            return alarmEnabled ? 1: -1;
        if (r == 0)
            continue;

        printf("var = 0x%02X\n", byte);

        switch (state)
        {
            case S_START:
                if (byte == FLAG) state = S_FLAG;
                break;
            case S_FLAG:
                if (byte == addr) state = S_A;
                else if (byte != FLAG) state = S_START;
                break;
            case S_A:
                if (byte == ctrl) state = S_C;
                else if (byte == FLAG) state = S_FLAG;
                else state = S_START;
                break;
            case S_C:
                if (byte == (addr ^ ctrl)) state = S_BCC;
                else if (byte == FLAG) state = S_FLAG;
                else state = S_START;
                break;
            case S_BCC:
                if (byte == FLAG) state = S_END;
                else state = S_START;
                break;
            default:
                break;
        }
    }
    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and send a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    params = llParameters;

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    static int readControl(unsigned char addr)
    {
        enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK } state = START;
        unsigned char byte, c = 0;

        while (1)
        {
            if (alarmEnabled) return -2;
            int r = readByteSerialPort(&byte);
            if (r < 0) return alarmEnabled ? -2 : -1;
            if (r == 0) continue;

            switch (state)
            {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == addr) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case A_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else { c = byte; state = C_RCV; }
                    break;
                case C_RCV:
                    if (byte == (addr ^ c)) state = BCC_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG) return c;
                    state = START;
                    break;
            }
        }
    }

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    unsigned char set[5] = {FLAG, A_TX, C_SET, A_TX ^ C_SET, FLAG};
    int connected = FALSE;
    alarmCount = 0;

    while (alarmCount <= llParameters.nRetransmissions && !connected)
    {
        int bytes = writeBytesSerialPort(set, 5);
        printf("%d bytes written to serial port (SET)\n", bytes);

        alarmEnabled = FALSE;
        alarm(llParameters.timeout);
        int r = readFrame(A_TX, C_UA);
        alarm(0);

        if (r < 0)
        {
            perror("readFrame");
            return -1;
        }
        if (r == 0)
            connected = TRUE;
    }

    if (!connected)
    {
        printf("No UA received after %d retransmissions\n", llParameters.nRetransmissions);
        closeSerialPort();
        return -1;
    }
    printf("UA received\n");

    // Keep the port open to send data later.
    return 0; 
}

int llOpenRx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and receive a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    if (readFrame(A_TX, C_SET) < 0)
    {
        perror("readFrame");
        return -1;
    }
    printf("SET received\n");

    unsigned char ua[5] = {FLAG, A_TX, C_UA, A_TX ^ C_UA, FLAG};
    int bytes = writeBytesSerialPort(ua, 5);
    if (bytes != 5)
    {
        perror("writeBytesSerialPort");
        return -1;
    }
    printf("%d bytes written to serial port (UA)\n", bytes);

    // Keep the port open to receive data later.
    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    unsigned char frame[2 * (MAX_PAYLOAD_SIZE + 1) + 5];
    int size = buildInformationFrame(buf, bufSize, frame);
    int expectedRR = (txSequence == 0) ? C_RR1 : C_RR0;

    for (int tries = 0; tries <= params.nRetransmissions; tries++)
    {
        writeBytesSerialPort(frame, size);
        nFramesSent++;
        if (tries > 0) nRetransmitted++;

        alarmEnabled = FALSE;
        alarm(params.timeout);
        int c = readControl(A_TX);
        alarm(0);

        if (c == -1) return -1;
        if (c == expectedRR)
        {
            txSequence = 1 - txSequence;
            return bufSize;
        }
        if (c == -2) nTimeouts++;
        // REJ, RR antigo ou timeout: retransmite
    }
    return -1;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    unsigned char disc[5] = {FLAG, A_TX, C_DISC, A_TX ^ C_DISC, FLAG};
    unsigned char ua[5] = {FLAG, A_RX, C_UA, A_RX ^ C_UA, FLAG};
    int ok = FALSE;

    for (int tries = 0; tries <= params.nRetransmissions && !ok; tries++)
    {
        writeBytesSerialPort(disc, 5);
        alarmEnabled = FALSE;
        alarm(params.timeout);
        int c = readControl(A_RX);
        alarm(0);

        if (c == -1) break;
        if (c == C_DISC) ok = TRUE;
    }

    if (ok) writeBytesSerialPort(ua, 5);
    sleep(1);

    printf("Frames sent: %d | Retransmissions: %d | Timeouts: %d\n",
           nFramesSent, nRetransmitted, nTimeouts);

    if (closeSerialPort() < 0) return -1;
    return ok ? 0 : -1;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}
