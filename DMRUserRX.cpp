/*
 *   Copyright (C) 2009-2017 by Jonathan Naylor G4KLX
 *   Copyright (C) 2017,2018 by Andy Uribe CA6JAU
 *   Copyright (C) 2025-2026 by Rob Williams M1BGT
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "Config.h"

#include "Globals.h"
#include "DMRUserRX.h"
#include "DMRSlotType.h"
#include "Utils.h"
#include "Debug.h"

const uint8_t MAX_SYNC_BYTES_ERRS = 2U;
const uint8_t MAX_SYNC_LOST_FRAMES = 30U;
const uint16_t NOENDPTR = 9999U;

const uint8_t CONTROL_DATA = 0x40U;
const uint8_t CONTROL_VOICE = 0x20U;
const uint8_t COLOR_CODE_NONE = 0xFFU;
const uint8_t BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

const uint8_t cachInterleave[DMR_CACH_LENGTH_BITS] = {
    0, 7, 8, 9, 1, 10, 11, 12, 2, 13, 14,
    15, 3, 16, 4, 17, 18, 19, 5, 20, 21, 22, 6, 23
};

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])
#define READ_BIT1(p,i)    ((p[(i)>>3] & BIT_MASK_TABLE[(i)&7]) >> (7 - ((i)&7)))

extern bool decodeDMRHamming74(uint8_t *received);

// EMB parity bits for the quadratic residue (16,7,6) code, ETSI TS 102 361-1 Annex B.3.2.
// Taken from OK-DMRlib via rob-uk-hub/MMDVM (GPL-2.0).
const uint16_t EMB_PARITY[7U] = {0x04FU, 0x11EU, 0x1B7U, 0x1E2U, 0x1C9U, 0x0E5U, 0x073U};

static uint8_t hamming74Syndrome(const uint8_t* received)
{
    uint8_t d1 = received[0];
    uint8_t d2 = received[1];
    uint8_t d3 = received[2];
    uint8_t d4 = received[3];

    uint8_t p1 = received[4];
    uint8_t p2 = received[5];
    uint8_t p3 = received[6];

    uint8_t s1 = p1 ^ d1 ^ d2 ^ d3;
    uint8_t s2 = p2 ^ d2 ^ d3 ^ d4;
    uint8_t s3 = p3 ^ d1 ^ d2 ^ d4;

    return (s3 << 2) | (s2 << 1) | s1;
}

static bool decodeEMB(const uint8_t* frame, uint8_t& colorCode)
{
    uint16_t emb = 0U;

    for (uint8_t i = 0U; i < 8U; i++)
        emb = (emb << 1) | READ_BIT1(frame, 8U + 108U + i);

    for (uint8_t i = 0U; i < 8U; i++)
        emb = (emb << 1) | READ_BIT1(frame, 8U + 148U + i);

    uint16_t data = emb >> 9;
    uint16_t parity = 0U;

    for (uint8_t i = 0U; i < 7U; i++) {
        if ((data >> (6U - i)) & 1U)
            parity ^= EMB_PARITY[i];
    }

    if (parity != (emb & 0x1FFU))
        return false;

    colorCode = (data >> 3) & 0x0FU;
    return true;
}

CDMRUserRX::CDMRUserRX() :
m_patternBuffer(0U),
m_buffer(),
m_dataPtr(0U),
m_endPtr(NOENDPTR),
m_startPtr(0U),
m_colorCode(0U),
m_slot(false),
m_dataSync(),
m_SuperFrameIndex(),
m_slotColorCode(),
m_syncThisBurst(false),
m_syncCount(0U)
{
    m_slotColorCode[0U] = COLOR_CODE_NONE;
    m_slotColorCode[1U] = COLOR_CODE_NONE;
}

void CDMRUserRX::reset()
{
    m_patternBuffer = 0U;
    m_dataPtr = 0U;
    m_endPtr = NOENDPTR;
    m_startPtr = 0U;
    m_slot = false;
    m_dataSync[0U] = false;
    m_dataSync[1U] = false;
    m_SuperFrameIndex[0U] = 0U;
    m_SuperFrameIndex[1U] = 0U;
    m_slotColorCode[0U] = COLOR_CODE_NONE;
    m_slotColorCode[1U] = COLOR_CODE_NONE;
    m_syncThisBurst = false;
    m_syncCount = 0U;
}

void CDMRUserRX::databit(bool bit)
{
    bool foundBsDataSync = false;
    bool foundBsVoiceSync = false;

    WRITE_BIT1(m_buffer, m_dataPtr, bit);

    m_patternBuffer <<= 1;
    if (bit)
    {
      m_patternBuffer |= 0x01U;
    }

    // Only base station syncs are accepted in DMR user mode
    foundBsDataSync = countBits64((m_patternBuffer & DMR_SYNC_BITS_MASK) ^ DMR_BS_DATA_SYNC_BITS) <= MAX_SYNC_BYTES_ERRS;
    foundBsVoiceSync = !foundBsDataSync && countBits64((m_patternBuffer & DMR_SYNC_BITS_MASK) ^ DMR_BS_VOICE_SYNC_BITS) <= MAX_SYNC_BYTES_ERRS;

    if (foundBsDataSync || foundBsVoiceSync) {
        m_startPtr = m_dataPtr + DMR_USER_LENGTH_BITS - DMR_SLOT_TYPE_LENGTH_BITS / 2U - DMR_INFO_LENGTH_BITS / 2U - DMR_SYNC_LENGTH_BITS - DMR_CACH_LENGTH_BITS + 1;
        if (m_startPtr >= DMR_USER_LENGTH_BITS)
        {
            m_startPtr -= DMR_USER_LENGTH_BITS;
        }

        m_endPtr = m_dataPtr + DMR_SLOT_TYPE_LENGTH_BITS / 2U + DMR_INFO_LENGTH_BITS / 2U;
        if (m_endPtr >= DMR_USER_LENGTH_BITS)
        {
            m_endPtr -= DMR_USER_LENGTH_BITS;
        }

        uint8_t cach[DMR_CACH_LENGTH_BITS];
        extractBits(m_startPtr, DMR_CACH_LENGTH_BITS, cach);

        bool syncSlot = false;
        if (cachSlot(cach, syncSlot))
        {
            uint8_t slot = syncSlot ? 1U : 0U;
            m_dataSync[slot] = foundBsDataSync;
            m_SuperFrameIndex[slot] = 0U;
        }

        m_syncThisBurst = true;
    }

    if (m_dataPtr == m_endPtr) {
        const bool syncBurst = m_syncThisBurst;
        m_syncThisBurst = false;

        if (syncBurst) {
            m_syncCount = 0U;
        } else if (++m_syncCount >= MAX_SYNC_LOST_FRAMES) {
            m_endPtr = NOENDPTR;
            m_dataSync[0U] = false;
            m_dataSync[1U] = false;
            m_SuperFrameIndex[0U] = 0U;
            m_SuperFrameIndex[1U] = 0U;
            m_slotColorCode[0U] = COLOR_CODE_NONE;
            m_slotColorCode[1U] = COLOR_CODE_NONE;
            m_syncCount = 0U;
        }

        if (m_endPtr != NOENDPTR) {
            uint8_t cach[DMR_CACH_LENGTH_BITS];
            extractBits(m_startPtr, DMR_CACH_LENGTH_BITS, cach);

            bool burstSlot = false;
            if (!cachSlot(cach, burstSlot)) {
                m_slot = !m_slot;
            } else {
                m_slot = burstSlot;
                uint8_t slot = m_slot ? 1U : 0U;

                uint16_t ptr = m_startPtr + DMR_CACH_LENGTH_BITS;
                if (ptr >= DMR_USER_LENGTH_BITS)
                {
                    ptr -= DMR_USER_LENGTH_BITS;
                }

                bitsToBytes(ptr, DMR_FRAME_LENGTH_BYTES, m_dataFrame + 1U);

                if (syncBurst)
                    m_SuperFrameIndex[slot] = 0U;

                if (m_dataSync[slot]) {
                    if (syncBurst) {
                        uint8_t colorCode;
                        uint8_t dataType;
                        CDMRSlotType slotType;
                        slotType.decode(m_dataFrame + 1U, colorCode, dataType);

                        if (colorCode == m_colorCode && dataType != DT_IDLE)
                        {
                            m_dataFrame[0U] = CONTROL_DATA | dataType;
                            serial.writeDMRData(m_slot, m_dataFrame, DMR_FRAME_LENGTH_BYTES + 1U);
                        }
                    }
                } else if (m_SuperFrameIndex[slot] < 6U) {
                    if (!syncBurst) {
                        uint8_t embColorCode = COLOR_CODE_NONE;
                        if (decodeEMB(m_dataFrame, embColorCode))
                            m_slotColorCode[slot] = embColorCode;
                    }

                    if (m_slotColorCode[slot] == m_colorCode) {
                        m_dataFrame[0U] = (m_SuperFrameIndex[slot] == 0U) ? CONTROL_VOICE : m_SuperFrameIndex[slot];
                        serial.writeDMRData(m_slot, m_dataFrame, DMR_FRAME_LENGTH_BYTES + 1U);
                    }

                    m_SuperFrameIndex[slot]++;
                }
            }

            m_startPtr = m_endPtr + 1U;
            if (m_startPtr >= DMR_USER_LENGTH_BITS)
            {
                m_startPtr -= DMR_USER_LENGTH_BITS;
            }

            m_endPtr = m_startPtr + DMR_FRAME_LENGTH_BITS + DMR_CACH_LENGTH_BITS - 1U;
            if (m_endPtr >= DMR_USER_LENGTH_BITS)
            {
                m_endPtr -= DMR_USER_LENGTH_BITS;
            }
        }
    }

    m_dataPtr++;
    if (m_dataPtr >= DMR_USER_LENGTH_BITS)
        m_dataPtr = 0U;
}

void CDMRUserRX::bitsToBytes(uint16_t start, uint8_t count, uint8_t* buffer)
{
    for (uint8_t i = 0U; i < count; i++) {
      buffer[i]  = 0U;
      buffer[i] |= READ_BIT1(m_buffer, start) << 7;
      start++;
      if (start >= DMR_USER_LENGTH_BITS)
          start -= DMR_USER_LENGTH_BITS;
      buffer[i] |= READ_BIT1(m_buffer, start) << 6;
      start++;
      if (start >= DMR_USER_LENGTH_BITS)
          start -= DMR_USER_LENGTH_BITS;
      buffer[i] |= READ_BIT1(m_buffer, start) << 5;
      start++;
      if (start >= DMR_USER_LENGTH_BITS)
          start -= DMR_USER_LENGTH_BITS;
      buffer[i] |= READ_BIT1(m_buffer, start) << 4;
      start++;
      if (start >= DMR_USER_LENGTH_BITS)
          start -= DMR_USER_LENGTH_BITS;
      buffer[i] |= READ_BIT1(m_buffer, start) << 3;
      start++;
      if (start >= DMR_USER_LENGTH_BITS)
          start -= DMR_USER_LENGTH_BITS;
      buffer[i] |= READ_BIT1(m_buffer, start) << 2;
      start++;
      if (start >= DMR_USER_LENGTH_BITS)
          start -= DMR_USER_LENGTH_BITS;
      buffer[i] |= READ_BIT1(m_buffer, start) << 1;
      start++;
      if (start >= DMR_USER_LENGTH_BITS)
          start -= DMR_USER_LENGTH_BITS;
      buffer[i] |= READ_BIT1(m_buffer, start) << 0;
      start++;
      if (start >= DMR_USER_LENGTH_BITS)
          start -= DMR_USER_LENGTH_BITS;
    }
}

bool CDMRUserRX::cachSlot(uint8_t* cach, bool& slot)
{
    bool cachdata[DMR_CACH_LENGTH_BITS];

    for (uint8_t i = 0U; i < DMR_CACH_LENGTH_BITS; i++)
    {
        cachdata[cachInterleave[i]] = cach[i] != 0U;
    }

    uint8_t tactBits[7];

    for (uint8_t i = 0U; i < 7U; i++)
    {
        tactBits[i] = cachdata[i] ? 1U : 0U;
    }

    uint8_t syndrome = hamming74Syndrome(tactBits);

    if (!decodeDMRHamming74(tactBits))
        return false;

    bool tc = tactBits[1] != 0U;

    if (syndrome != 0U && tc == m_slot)
        return false;

    slot = tc;

    return true;
}

void CDMRUserRX::extractBits(uint16_t start, uint8_t count_bits, uint8_t* buffer)
{
    for (uint8_t i = 0U; i < count_bits; i++) {
        buffer[i]  = READ_BIT1(m_buffer, start);
        start++;
        if (start >= DMR_USER_LENGTH_BITS)
        {
            start -= DMR_USER_LENGTH_BITS;
        }
    }
}

void CDMRUserRX::setColorCode(uint8_t colorCode)
{
    m_colorCode = colorCode;
}
