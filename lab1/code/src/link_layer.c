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

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);
    /*
    // Create string to send
    unsigned char buf[BUF_SIZE] = {0};

    for (int i = 0; i < BUF_SIZE; i++)
    {
        buf[i] = 'a' + i % 26;
    }

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.
    buf[5] = '\n';

    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
    printf("%d bytes written to serial port\n", bytes);

    // Wait until all bytes have been written to the serial port
    sleep(1);
    */
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

    /* Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);*/

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
    /*
    // Read from serial port until the 'z' char is received.

    // NOTE: This while() cycle is a simple example showing how to read from the serial port.
    // It must be changed in order to respect the specifications of the protocol indicated in the Lab guide.

    // TODO: Save the received bytes in a buffer array and print it at the end of the program.
    volatile int STOP = FALSE;
    int nBytesBuf = 0;

    while (STOP == FALSE)
    {
        // Read one byte from serial port.
        // NOTE: You must check how many bytes were actually read by reading the return value.
        // In this example, we assume that the byte is always read, which may not be true.
        unsigned char byte;
        int bytes = readByteSerialPort(&byte);
        nBytesBuf += bytes;

        printf("Byte received: %c\n", byte);

        if (byte == 'z')
        {
            printf("Received 'z' char. Stop reading from serial port.\n");
            STOP = TRUE;
        }
    }

    printf("Total bytes received: %d\n", nBytesBuf);
    */

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

    /*sleep(1);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);*/

    // Keep the port open to receive data later.
    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
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
    // TODO: Implement this function

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}
