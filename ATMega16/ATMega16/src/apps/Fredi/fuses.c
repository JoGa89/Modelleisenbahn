/****************************************************************************
    Copyright (C) 2006, 2011 Martin Pischky, Olaf Funke

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    $Id: fuses.c,v 1.3 2011/07/25 18:31:16 pischky Exp $
******************************************************************************/

/*
 *  Define fuses in elf file
 */

#define LFUSE __attribute__ ((section ("lfuses")))
#define HFUSE __attribute__ ((section ("hfuses")))
#define EFUSE __attribute__ ((section ("efuses")))

//////////////////////////////////////////////////////////////////////////////////
#if defined(__AVR_ATmega16__)

  //
  //                  Fuse High Byte
  //
  #define BOOTRST     0  // ----------+
  #define BOOTSZ0     1  // ---------+|
  #define BOOTSZ1     2  // --------+||
  #define EESAVE      3  // -------+|||
  #define CKOPT       4  // -----+ ||||
  #define SPIEN       5  // ----+| ||||
  #define WDTON       6  // ---+|| ||||
  #define RSTDISBL    7  // --+||| ||||
  //                          |||| ||||
  //      hfuse:              1101 0001 = 0xD1

  unsigned char hfuse HFUSE =
    ( (1 << RSTDISBL ) | (1 << WDTON    ) | (0 << SPIEN    ) | (1 << CKOPT    )
    | (0 << EESAVE   ) | (0 << BOOTSZ1  ) | (0 << BOOTSZ0  ) | (1 << BOOTRST  ) );

  //
  //                  Fuse Low Byte
  //
  #define CKSEL0      0  // ----------+
  #define CKSEL1      1  // ---------+|
  #define CKSEL2      2  // --------+||
  #define CKSEL3      3  // -------+|||
  #define SUT0        4  // -----+ ||||
  #define SUT1        5  // ----+| ||||
  #define BODEN       6  // ---+|| ||||
  #define BODLEVEL    7  // --+||| ||||
  //                          |||| ||||
  //      lfuse:              1011 1111 = 0xBF

  unsigned char lfuse LFUSE =
    ( (1 << BODLEVEL ) | (0 << BODEN    ) | (1 << SUT1     ) | (1 << SUT0     )
    | (1 << CKSEL3   ) | (1 << CKSEL2   ) | (1 << CKSEL1   ) | (1 << CKSEL0   ) );

//////////////////////////////////////////////////////////////////////////////////
#elif defined(__AVR_ATmega48__)  | defined(__AVR_ATmega48A__)  \
    | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega48PA__)

  //
  //                  Fuse Extended Byte
  //
  #define SELFPRGEN   0  // ----------+
  #define UNUSED1     1  // ---------+|
  #define UNUSED2     2  // --------+||
  #define UNUSED3     3  // -------+|||
  #define UNUSED4     4  // -----+ ||||
  #define UNUSED5     5  // ----+| ||||
  #define UNUSED6     6  // ---+|| ||||
  #define UNUSED7     7  // --+||| ||||
  //                          |||| ||||
  //      efuse:              1111 1111 = 0xFF

  unsigned char efuse EFUSE =
    ( (1 << UNUSED7  ) | (1 << UNUSED6  ) | (1 << UNUSED5  ) | (1 << UNUSED4  )
    | (1 << UNUSED3  ) | (1 << UNUSED2  ) | (1 << UNUSED1  ) | (1 << SELFPRGEN) );

  //
  //                   Fuse High Byte
  //
  #define BODLEVEL0   0  // ----------+
  #define BODLEVEL1   1  // ---------+|
  #define BODLEVEL2   2  // --------+||
  #define EESAVE      3  // -------+|||
  #define WDTON       4  // -----+ ||||
  #define SPIEN       5  // ----+| ||||
  #define DWEN        6  // ---+|| ||||
  #define RSTDISBL    7  // --+||| ||||
  //                          |||| ||||
  //      hfuse:              1101 0101 = 0xD5

  unsigned char hfuse HFUSE =
    ( (1 << RSTDISBL ) | (1 << DWEN     ) | (0 << SPIEN    ) | (1 << WDTON    )
    | (0 << EESAVE   ) | (1 << BODLEVEL2) | (0 << BODLEVEL1) | (1 << BODLEVEL0) );

  //
  //                  Fuse Low Byte
  //
  #define CKSEL0      0  // ----------+
  #define CKSEL1      1  // ---------+|
  #define CKSEL2      2  // --------+||
  #define CKSEL3      3  // -------+|||
  #define SUT0        4  // -----+ ||||
  #define SUT1        5  // ----+| ||||
  #define CKOUT       6  // ---+|| ||||
  #define CKDIV8      7  // --+||| ||||
  //                          |||| ||||
  //      lfuse:              1111 1101 = 0xFD

  unsigned char lfuse LFUSE =
    ( (1 << CKDIV8   ) | (1 << CKOUT    ) | (1 << SUT1     ) | (1 << SUT0     )
    | (1 << CKSEL3   ) | (1 << CKSEL2   ) | (0 << CKSEL1   ) | (1 << CKSEL0   ) );

//////////////////////////////////////////////////////////////////////////////////
#elif defined(__AVR_ATmega88__)  | defined(__AVR_ATmega88A__)  \
    | defined(__AVR_ATmega88P__) | defined(__AVR_ATmega88PA__) \
    | defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) \
    | defined(__AVR_ATmega168P__)| defined(__AVR_ATmega168PA__)

  #define X 0  // avrdude likes unused bits as "0",
               // Atmels opinion is "1" (but stk500.exe also accepts 0)

  //
  //                  Fuse Extended Byte
  //
  #define BOOTRST     0  // ----------+
  #define BOOTSZ0     1  // ---------+|
  #define BOOTSZ1     2  // --------+||
  #define UNUSED3     3  // -------+|||
  #define UNUSED4     4  // -----+ ||||
  #define UNUSED5     5  // ----+| ||||
  #define UNUSED6     6  // ---+|| ||||
  #define UNUSED7     7  // --+||| ||||
  //                          |||| ||||
  //      efuse:              XXXX X001 = 0xF9/0x01

  unsigned char efuse EFUSE =
    ( (X << UNUSED7  ) | (X << UNUSED6  ) | (X << UNUSED5  ) | (X << UNUSED4  )
    | (X << UNUSED3  ) | (0 << BOOTSZ1  ) | (0 << BOOTSZ0  ) | (1 << BOOTRST  ) );

  //
  //                  Fuse High Byte
  //
  #define BODLEVEL0   0  // ----------+
  #define BODLEVEL1   1  // ---------+|
  #define BODLEVEL2   2  // --------+||
  #define EESAVE      3  // -------+|||
  #define WDTON       4  // -----+ ||||
  #define SPIEN       5  // ----+| ||||
  #define DWEN        6  // ---+|| ||||
  #define RSTDISBL    7  // --+||| ||||
  //                          |||| ||||
  //      hfuse:              1101 0101 = 0xD5

  unsigned char hfuse HFUSE =
    ( (1 << RSTDISBL ) | (1 << DWEN     ) | (0 << SPIEN    ) | (1 << WDTON    )
    | (0 << EESAVE   ) | (1 << BODLEVEL2) | (0 << BODLEVEL1) | (1 << BODLEVEL0) );

  //
  //                  Fuse Low Byte
  //
  #define CKSEL0      0  // ----------+
  #define CKSEL1      1  // ---------+|
  #define CKSEL2      2  // --------+||
  #define CKSEL3      3  // -------+|||
  #define SUT0        4  // -----+ ||||
  #define SUT1        5  // ----+| ||||
  #define CKOUT       6  // ---+|| ||||
  #define CKDIV8      7  // --+||| ||||
  //                          |||| ||||
  //      lfuse:              1111 1101 = 0xFD

  unsigned char lfuse LFUSE =
    ( (1 << CKDIV8   ) | (1 << CKOUT    ) | (1 << SUT1     ) | (1 << SUT0     )
    | (1 << CKSEL3   ) | (1 << CKSEL2   ) | (0 << CKSEL1   ) | (1 << CKSEL0   ) );

//////////////////////////////////////////////////////////////////////////////////
#elif defined(__AVR_ATmega328__) | defined(__AVR_ATmega328P__) 

  #define X 0  // avrdude likes unused bits as "0",
               // Atmels opinion is "1" (but stk500.exe also accepts 0)

  //
  //                  Fuse Extended Byte
  //
  // BODLEVEL2..0 = 101 (V_BOT=2.7V)
  //
  #define BODLEVEL0   0  // ----------+
  #define BODLEVEL1   1  // ---------+|
  #define BODLEVEL2   2  // --------+||
  #define UNUSED3     3  // -------+|||
  #define UNUSED4     4  // -----+ ||||
  #define UNUSED5     5  // ----+| ||||
  #define UNUSED6     6  // ---+|| ||||
  #define UNUSED7     7  // --+||| ||||
  //                          |||| ||||
  //      efuse:              XXXX X101 = 0xFD/0x05

  unsigned char efuse EFUSE =
    ( (X << UNUSED7  ) | (X << UNUSED6  ) | (X << UNUSED5  ) | (X << UNUSED4  )
    | (X << UNUSED3  ) | (1 << BODLEVEL2) | (0 << BODLEVEL1) | (1 << BODLEVEL0) );

  //
  //                   Fuse High Byte
  //
  #define BOOTRST     0  // ----------+
  #define BOOTSZ0     1  // ---------+|
  #define BOOTSZ1     2  // --------+||
  #define EESAVE      3  // -------+|||
  #define WDTON       4  // -----+ ||||
  #define SPIEN       5  // ----+| ||||
  #define DWEN        6  // ---+|| ||||
  #define RSTDISBL    7  // --+||| ||||
  //                          |||| ||||
  //      hfuse:              1101 0001 = 0xD1

  unsigned char hfuse HFUSE =
    ( (1 << RSTDISBL ) | (1 << DWEN     ) | (0 << SPIEN    ) | (1 << WDTON    )
    | (0 << EESAVE   ) | (0 << BOOTSZ1  ) | (0 << BOOTSZ0  ) | (1 << BOOTRST  ) );

  //
  //                  Fuse Low Byte
  //
  // CKDIV8    = 1   (Do not divide clock by 8)
  // CKOUT     = 1   (No clock output to PORTB0)
  // Low Power Crystal Oscillator
  // CKSEL3..1 = 110 (3.0 - 8 MHz)  (see datasheet 8271D-AVR-05/11 Table 9-3)
  // CKSEL0    = 1   (Crystal Osc., BOD enabled / Start-up Time: 16K CK /
  // SUT1..0   = 01   Add. Delay from Reset: 14CK )
  //                                (see datasheet 8271D-AVR-05/11 Table 9-4)
  //
  #define CKSEL0      0  // ----------+
  #define CKSEL1      1  // ---------+|
  #define CKSEL2      2  // --------+||
  #define CKSEL3      3  // -------+|||
  #define SUT0        4  // -----+ ||||
  #define SUT1        5  // ----+| ||||
  #define CKOUT       6  // ---+|| ||||
  #define CKDIV8      7  // --+||| ||||
  //                          |||| ||||
  //      lfuse:              1101 1101 = 0xDD

  unsigned char lfuse LFUSE =
    ( (1 << CKDIV8   ) | (1 << CKOUT    ) | (0 << SUT1     ) | (1 << SUT0     )
    | (1 << CKSEL3   ) | (1 << CKSEL2   ) | (0 << CKSEL1   ) | (1 << CKSEL0   ) );

//////////////////////////////////////////////////////////////////////////////////
#else

    #error unsupported MCU value (for now)

#endif
