// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

static int buildControl(unsigned char c, long size, const char *name,
                        unsigned char *packet)
{
    int n = strlen(name);
    int len = 0;

    packet[len++] = c;
    packet[len++] = 0;                 // T: tamanho do ficheiro
    packet[len++] = 4;                 // L
    packet[len++] = (size >> 24) & 0xFF;
    packet[len++] = (size >> 16) & 0xFF;
    packet[len++] = (size >> 8) & 0xFF;
    packet[len++] = size & 0xFF;
    packet[len++] = 1;                 // T: nome do ficheiro
    packet[len++] = n;                 // L
    memcpy(&packet[len], name, n);
    return len + n;
}

static void sendFile(const char *filename)
{
    FILE *f = fopen(filename, "rb");
    if (f == NULL) { perror("fopen"); return; }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);

    unsigned char packet[MAX_PAYLOAD_SIZE];
    int len = buildControl(1, size, filename, packet);        // START
    if (llSend(packet, len) < 0) { fclose(f); return; }

    int k;
    while ((k = fread(&packet[3], 1, MAX_PAYLOAD_SIZE - 3, f)) > 0)
    {
        packet[0] = 2;                                         // DATA
        packet[1] = k / 256;
        packet[2] = k % 256;
        if (llSend(packet, k + 3) < 0) { fclose(f); return; }
    }

    len = buildControl(3, size, filename, packet);            // END
    llSend(packet, len);
    fclose(f);
}

// Read the file size from a control packet.
static int getFileSize(const unsigned char *packet, int length,
                       unsigned long *fileSize)
{
    // Check the size field.
    if (length < 7)
    {
        return -1;
    }

    if (packet[1] != 0 || packet[2] != 4)
    {
        return -1;
    }

    unsigned long size = 0;

    for (int i = 3; i < 7; i++)
    {
        size = size * 256 + packet[i];
    }

    *fileSize = size;
    return 0;
}

// Receive packets and save the file.
static int receiveFile(const char *filename)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    unsigned char startPacket[MAX_PAYLOAD_SIZE];

    unsigned long fileSize;
    unsigned long received = 0;

    // Receive START.
    int length = llReceive(packet);

    if (length < 1 || packet[0] != 1)
    {
        printf("Invalid START packet\n");
        return -1;
    }

    if (getFileSize(packet, length, &fileSize) < 0)
    {
        printf("Invalid file size\n");
        return -1;
    }

    // Check the file name field sent by the transmitter.
    if (length < 9 || packet[7] != 1 ||
        length != 9 + packet[8])
    {
        printf("Invalid file name field\n");
        return -1;
    }

    // Save START to compare it with END later.
    int startLength = length;
    memcpy(startPacket, packet, length);

    // Use the output name given in the command line.
    FILE *f = fopen(filename, "wb");

    if (f == NULL)
    {
        perror("fopen");
        return -1;
    }

    while (1)
    {
        length = llReceive(packet);

        if (length < 1)
        {
            printf("Failed to receive packet\n");
            fclose(f);
            return -1;
        }

        if (packet[0] == 2)
        {
            // DATA needs a three-byte header.
            if (length < 3)
            {
                printf("Invalid DATA packet\n");
                fclose(f);
                return -1;
            }

            int dataSize = packet[1] * 256 + packet[2];

            // Check the packet size and remaining file size.
            if (dataSize != length - 3 ||
                (unsigned long)dataSize > fileSize - received)
            {
                printf("Invalid data size\n");
                fclose(f);
                return -1;
            }

            // Write only the file data, without the header.
            if (fwrite(&packet[3], 1, dataSize, f) != (size_t)dataSize)
            {
                perror("fwrite");
                fclose(f);
                return -1;
            }

            received = received + dataSize;
        }
        else if (packet[0] == 3)
        {
            // END must repeat the START information.
            if (length != startLength ||
                memcmp(&packet[1], &startPacket[1], length - 1) != 0 ||
                received != fileSize)
            {
                printf("Invalid END packet or incomplete file\n");
                fclose(f);
                return -1;
            }

            if (fclose(f) != 0)
            {
                perror("fclose");
                return -1;
            }

            printf("File received: %lu bytes\n", received);
            return 0;
        }
        else
        {
            printf("Unexpected packet type\n");
            fclose(f);
            return -1;
        }
    }
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

    if (strcmp(role, "tx") == 0)
    {
        if (llOpenTx(llParameters) < 0) return;
        sendFile(filename);
        llCloseTx();
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0)
        {
            return;
        }

        int result = receiveFile(filename);

        if (result < 0)
        {
            printf("File reception failed\n");
        }

        if (llCloseRx() < 0)
        {
            printf("Connection close failed\n");
        }
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
