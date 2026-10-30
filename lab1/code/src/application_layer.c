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
        llOpenRx(llParameters);
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
