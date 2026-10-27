/*
 *   Copyright (C) 2015 by Jonathan Naylor G4KLX
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

#if !defined(DMRUSERRX_H)
#define DMRUSERRX_H

#include "Config.h"

#include "DMRDefines.h"

const uint16_t DMR_USER_LENGTH_BITS = DMR_FRAME_LENGTH_BITS + DMR_CACH_LENGTH_BITS;

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
  bool     m_dataSync[2];
  uint8_t  m_SuperFrameIndex[2];
  uint8_t  m_slotColorCode[2];
  bool     m_syncThisBurst;
  uint8_t  m_syncCount;

  uint8_t m_dataFrame[DMR_FRAME_LENGTH_BYTES + 1U];

  void bitsToBytes(uint16_t start, uint8_t count, uint8_t* buffer);
  bool cachSlot(uint8_t* cach, bool& slot);
  void extractBits(uint16_t start, uint8_t count_bits, uint8_t* buffer);
};

#endif
