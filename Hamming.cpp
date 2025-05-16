#include "Hamming.h"
#include <cstring>


CHamming::CHamming()
{
    Hamming_7_4_init();
    QR_16_7_6_init();
}

void CHamming::Hamming_7_4_init()
{
    // correctable bit positions given syndrome bits as index (see above)
    memset(Hamming_7_4_m_corr, 0xFF, 8); // initialize with all invalid positions
    Hamming_7_4_m_corr[0b101] = 0;
    Hamming_7_4_m_corr[0b111] = 1;
    Hamming_7_4_m_corr[0b110] = 2;
    Hamming_7_4_m_corr[0b011] = 3;
    Hamming_7_4_m_corr[0b100] = 4;
    Hamming_7_4_m_corr[0b010] = 5;
    Hamming_7_4_m_corr[0b001] = 6;
}


void CHamming::QR_16_7_6_init()
{
    int i1 = 0, i2 = 0, ir = 0, ip = 0;
    int syndromeI = 0, syndromeIP = 0;
    int ip1 = 0, ip2 = 0;
    int syndromeIP1 = 0, syndromeIP2 = 0;

    memset(QR_16_7_6_m_corr, 0xFF, 2*512);

    for (i1 = 0; i1 < 7; i1++)
    {
        for (i2 = i1+1; i2 < 7; i2++)
        {
            // 2 bit patterns
            syndromeI = 0;

            for (ir = 0; ir < 9; ir++)
            {
                syndromeI += ((QR_16_7_6_m_H[16*ir + i1] +  QR_16_7_6_m_H[16*ir + i2]) % 2) << (8-ir);
            }

            QR_16_7_6_m_corr[syndromeI][0] = i1;
            QR_16_7_6_m_corr[syndromeI][1] = i2;
        }

        // single bit patterns
        syndromeI = 0;

        for (ir = 0; ir < 9; ir++)
        {
            syndromeI += QR_16_7_6_m_H[16*ir + i1] << (8-ir);
        }

        QR_16_7_6_m_corr[syndromeI][0] = i1;

        // 1 possible bit flip left in the parity part
        for (ip = 0; ip < 9; ip++)
        {
            syndromeIP = syndromeI ^ (1 << (8-ip));
            QR_16_7_6_m_corr[syndromeIP][0] = i1;
            QR_16_7_6_m_corr[syndromeIP][1] = 7 + ip;
        }
    }

    // no bit patterns (in message) -> all in parity
    for (ip1 = 0; ip1 < 9; ip1++) // 1 bit flip in parity
    {
        syndromeIP1 = (1 << (8-ip1));
        QR_16_7_6_m_corr[syndromeIP1][0] = 7 + ip1;

        for (ip2 = ip1+1; ip2 < 9; ip2++) // 1 more bit flip in parity
        {
            syndromeIP2 = syndromeIP1 ^ (1 << (8-ip2));
            QR_16_7_6_m_corr[syndromeIP2][0] = 7 + ip1;
            QR_16_7_6_m_corr[syndromeIP2][1] = 7 + ip2;
        }
    }
}

bool CHamming::Hamming_7_4_decode(unsigned char *rxBits) // corrects in place
{
    unsigned int syndromeI = 0; // syndrome index
    int is = 0;
    int correction = 0;

    for (is = 0; is < 3; is++)
    {
        syndromeI += (((rxBits[0] * Hamming_7_4_m_H[7*is + 0])
                     + (rxBits[1] * Hamming_7_4_m_H[7*is + 1])
                     + (rxBits[2] * Hamming_7_4_m_H[7*is + 2])
                     + (rxBits[3] * Hamming_7_4_m_H[7*is + 3])
                     + (rxBits[4] * Hamming_7_4_m_H[7*is + 4])
                     + (rxBits[5] * Hamming_7_4_m_H[7*is + 5])
                     + (rxBits[6] * Hamming_7_4_m_H[7*is + 6])) % 2) << (2-is);
    }

    if (syndromeI > 0)
    {
        if (Hamming_7_4_m_corr[syndromeI] == 0xFF)
        {
            return false;
        }
        else
        {
            rxBits[Hamming_7_4_m_corr[syndromeI]] ^= 1; // flip bit
            correction++;
        }
        //not sure of upper limit on what hamming can correct (if any),
        //but will test with 0 and 1 to see how those perform
        if (correction > 1)
        {
            return false;
        }
    }

    return true;
}

bool CHamming::QR_16_7_6_decode(unsigned char *rxBits)
{
    unsigned int syndromeI = 0; // syndrome index
    int is = 0;
    int i = 0;

    for (is = 0; is < 9; is++)
    {
        syndromeI += (((rxBits[0]  * QR_16_7_6_m_H[16*is + 0])
                     + (rxBits[1]  * QR_16_7_6_m_H[16*is + 1])
                     + (rxBits[2]  * QR_16_7_6_m_H[16*is + 2])
                     + (rxBits[3]  * QR_16_7_6_m_H[16*is + 3])
                     + (rxBits[4]  * QR_16_7_6_m_H[16*is + 4])
                     + (rxBits[5]  * QR_16_7_6_m_H[16*is + 5])
                     + (rxBits[6]  * QR_16_7_6_m_H[16*is + 6])
                     + (rxBits[7]  * QR_16_7_6_m_H[16*is + 7])
                     + (rxBits[8]  * QR_16_7_6_m_H[16*is + 8])
                     + (rxBits[9]  * QR_16_7_6_m_H[16*is + 9])
                     + (rxBits[10] * QR_16_7_6_m_H[16*is + 10])
                     + (rxBits[11] * QR_16_7_6_m_H[16*is + 11])
                     + (rxBits[12] * QR_16_7_6_m_H[16*is + 12])
                     + (rxBits[13] * QR_16_7_6_m_H[16*is + 13])
                     + (rxBits[14] * QR_16_7_6_m_H[16*is + 14])
                     + (rxBits[15] * QR_16_7_6_m_H[16*is + 15])) % 2) << (8-is);
    }

    if (syndromeI > 0)
    {
        i = 0;

        for (; i < 2; i++)
        {
            if (QR_16_7_6_m_corr[syndromeI][i] == 0xFF)
            {
                break;
            }
            else
            {
                rxBits[QR_16_7_6_m_corr[syndromeI][i]] ^= 1; // flip bit
            }
        }

        if (i == 0)
        {
            return false;
        }
    }

    return true;
}
