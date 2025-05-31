/*
 *   Copyright (C) 2015 by Jonathan Naylor G4KLX
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

 #if !defined(DMRIDLERX_H)
 #define  DMRIDLERX_H
 
 #include "Config.h"
 
 #include "DMRDefines.h"
 
 #define CACH_SIZE_BITS 24U

 const uint16_t DMR_USER_LENGTH_BITS = 320U+CACH_SIZE_BITS; // Why 320?
 
 class CDMRUserRX {
 public:
   CDMRUserRX();
 
   void databit(bool bit);
 
   void setColorCode(uint8_t colorCode);
 
   void reset();
 
 private:
   uint64_t m_patternBuffer;
   uint8_t  m_buffer[DMR_USER_LENGTH_BITS / 8U];
   uint16_t m_dataPtr;
   uint16_t m_endPtr;
   uint16_t m_startPtr;
   uint8_t  m_colorCode;
   bool     m_slot;
   uint8_t m_dataFrame[DMR_FRAME_LENGTH_BYTES + 1U]; //33+1 bytes (Ignores CACH)

 
   void bitsToBytes(uint16_t start, uint8_t count, uint8_t* buffer);
   bool cachCheck(uint8_t* cach);
   void extractBits(uint16_t start, uint8_t count_bits, uint8_t* buffer);
 };
 
 #endif
 