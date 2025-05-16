/*
 *   Copyright (C) 2009-2017 by Jonathan Naylor G4KLX
 *   Copyright (C) 2017,2018 by Andy Uribe CA6JAU
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
 
const uint8_t MAX_SYNC_BYTES_ERRS = 2U; 
const uint16_t NOENDPTR = 9999U;
 
const uint8_t CONTROL_IDLE = 0x80U;
const uint8_t CONTROL_DATA = 0x40U; 
const uint8_t BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

const uint8_t cachInterleave[CACH_SIZE_BITS] = {
    0, 7, 8, 9, 1, 10, 11, 12, 2, 13, 14,
    15, 3, 16, 4, 17, 18, 19, 5, 20, 21, 22, 6, 23
};
 
 #define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])
 #define READ_BIT1(p,i)    ((p[(i)>>3] & BIT_MASK_TABLE[(i)&7]) >> (7 - ((i)&7)))
 
 CDMRUserRX::CDMRUserRX() :
 m_patternBuffer(0U),
 m_buffer(),
 m_dataPtr(0U),
 m_endPtr(NOENDPTR),
 m_colorCode(0U)
 {
 }
 
 void CDMRUserRX::reset()
 {
   m_dataPtr   = 0U;
   m_endPtr    = NOENDPTR;
 }
 
 bool CDMRUserRX::cachCheck(uint8_t* cach)
 {
   uint8_t i;
 
   bool cachdata[CACH_SIZE_BITS];
     
   for (i = 0; i < CACH_SIZE_BITS; i++)
   {
     cachdata[cachInterleave[i]] = cach[i];
   }
 
   uint8_t tactBits[7];
 
   for (i = 0; i < 7; i++)
   {
     tactBits[i] = cachdata[i];
   }
 
   if (!m_hamming.Hamming_7_4_decode(tactBits))
   {
     return false;
   }
 
   // bool at_continuous = tactBits[0];
 
   m_slot = tactBits[1]; // TDMA Channel (TC)
 
   return true;
 }
 
 
 void CDMRUserRX::databit(bool bit)
 {
    bool foundBsSync = false;
    bool foundMsSync = false;
    uint8_t cach[CACH_SIZE_BITS];
    
   // 288 bits per slot / 36 bytes per slot (including CACH)
   // | CACH (24 bits) | TS 1 Payload (98 bits) | Slot Type (10 bits) | Sync (48 bits) | Slot Type (10 bits) | TS 1 Payload (98 bits) 
   // Both slots: 576 bits  / 72 bytes
 
   // sync mask is 0x0000FFFFFFFFFFFFU
   // skip 32 bits
   // 320-264 = 56?

   WRITE_BIT1(m_buffer, m_dataPtr, bit);
 
   m_patternBuffer <<= 1;
   if (bit)
   {
     m_patternBuffer |= 0x01U;
   }

   foundBsSync = countBits64((m_patternBuffer & DMR_SYNC_BITS_MASK) ^ DMR_BS_DATA_SYNC_BITS) <= MAX_SYNC_BYTES_ERRS;
   foundMsSync = !foundBsSync && countBits64((m_patternBuffer & DMR_SYNC_BITS_MASK) ^ DMR_MS_DATA_SYNC_BITS) <= MAX_SYNC_BYTES_ERRS;

   if (foundBsSync || foundMsSync) {
     m_startPtr = m_dataPtr + DMR_USER_LENGTH_BITS - DMR_SLOT_TYPE_LENGTH_BITS / 2U - DMR_INFO_LENGTH_BITS / 2U - DMR_SYNC_LENGTH_BITS - CACH_SIZE_BITS + 1;
     if (m_startPtr >= DMR_USER_LENGTH_BITS)
     {
       m_startPtr -= DMR_USER_LENGTH_BITS;
     }

     m_endPtr = m_dataPtr + DMR_SLOT_TYPE_LENGTH_BITS / 2U + DMR_INFO_LENGTH_BITS / 2U;
     if (m_endPtr >= DMR_USER_LENGTH_BITS)
     {
       m_endPtr -= DMR_USER_LENGTH_BITS;
     }
   }
 
   if (m_dataPtr == m_endPtr) {
        uint16_t ptr;
        ptr = m_endPtr + DMR_USER_LENGTH_BITS - DMR_FRAME_LENGTH_BITS + 1;
        if (ptr >= DMR_USER_LENGTH_BITS)
        {
            ptr -= DMR_USER_LENGTH_BITS;
        }

        bitsToBytes(ptr, DMR_FRAME_LENGTH_BYTES, m_dataFrame + 1U);

        uint8_t colorCode;
        uint8_t dataType;
        CDMRSlotType slotType;
        // Read from byte 12 (bit 96) in m_dataFrame
        slotType.decode(m_dataFrame + 1U, colorCode, dataType);

        if(colorCode == m_colorCode && dataType != 0x09 /* IDLE */)
        {
            // Start with the CACH
            extractBits(m_startPtr, CACH_SIZE_BITS, cach);
            
            if(cachCheck(cach))
            {
                m_dataFrame[0U] = CONTROL_DATA | dataType; // Assume data for now (due to sync)
                serial.writeDMRData(m_slot, m_dataFrame, DMR_FRAME_LENGTH_BYTES + 1U);
            }
        }

        m_endPtr  = NOENDPTR;
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
 
 