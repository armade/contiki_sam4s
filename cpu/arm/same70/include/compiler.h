/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011, Atmel Corporation
 *
 * All rights reserved.
 *

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#ifndef _COMPILER_H_
#define _COMPILER_H_

/*
 * Peripherals registers definitions
 */
#include "same70.h"

#include "interrupt_sam_nvic.h"
#include "preprocessor.h"

//_____ D E C L A R A T I O N S ____________________________________________

#ifndef __ASSEMBLY__

#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

/* Define WEAK attribute */
#if defined   ( __CC_ARM   )
    #define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ )
    #define WEAK __weak
#elif defined (  __GNUC__  )
    #define WEAK __attribute__ ((weak))
#endif

/* Define Compiler name of tool chains */
#if defined   ( __CC_ARM   )
    #define COMPILER_NAME               "KEIL"
#elif defined ( __ICCARM__ )
    #define COMPILER_NAME               "IAR"
#elif defined (  __GNUC__  )
    #define COMPILER_NAME               "GCC"
#endif

/* Define NO_INIT attribute */
#if defined   ( __CC_ARM   )
    #define NO_INIT
#elif defined ( __ICCARM__ )
    #define NO_INIT __no_init
#elif defined (  __GNUC__  )
    #define NO_INIT
#endif
   
/* Define RAMFUNC attribute */
#if defined   ( __CC_ARM   ) /* Keil µVision 4 */
#   define RAMFUNC __attribute__ ((section(".ramfunc")))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
#   define RAMFUNC __ramfunc
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
#   define RAMFUNC __attribute__ ((section(".ramfunc")))
#endif

/* Define memory sync for tool chains */
#if defined   ( __CC_ARM   )
    #define memory_sync()        __dsb(15);__isb(15);
#elif defined ( __ICCARM__ )
    #define memory_sync()        __DSB();__ISB();
#elif defined (  __GNUC__  )
    #define memory_sync()        __DSB();__ISB();
#endif

/* Define memory barrier for tool chains */
#if defined   ( __CC_ARM   )
    #define memory_barrier()        __dmb(15);
#elif defined ( __ICCARM__ )
    #define memory_barrier()        __DMB();
#elif defined (  __GNUC__  )
    #define memory_barrier()        __DMB();
#endif

/*! \name Token Paste
 *
 * Paste N preprocessing tokens together, these tokens being allowed to be \#defined.
 *
 * May be used only within macros with the tokens passed as arguments if the tokens are \#defined.
 *
 * For example, writing TPASTE2(U, WIDTH) within a macro \#defined by
 * UTYPE(WIDTH) and invoked as UTYPE(UL_WIDTH) with UL_WIDTH \#defined as 32 is
 * equivalent to writing U32.
 */
//! @{
#define TPASTE2( a, b)                            a##b
#define TPASTE3( a, b, c)                         a##b##c
//! @}

/*! \name Absolute Token Paste
 *
 * Paste N preprocessing tokens together, these tokens being allowed to be \#defined.
 *
 * No restriction of use if the tokens are \#defined.
 *
 * For example, writing ATPASTE2(U, UL_WIDTH) anywhere with UL_WIDTH \#defined
 * as 32 is equivalent to writing U32.
 */
//! @{
#define ATPASTE2( a, b)                           TPASTE2( a, b)
#define ATPASTE3( a, b, c)                        TPASTE3( a, b, c)
//! @}


/**
 * \brief Emit the compiler pragma \a arg.
 *
 * \param arg The pragma directive as it would appear after \e \#pragma
 * (i.e. not stringified).
 */
#define COMPILER_PRAGMA(arg)            _Pragma(#arg)

/**
 * \def COMPILER_PACK_SET(alignment)
 * \brief Set maximum alignment for subsequent structure and union
 * definitions to \a alignment.
 */
#define COMPILER_PACK_SET(alignment)   COMPILER_PRAGMA(pack(alignment))

/**
 * \def COMPILER_PACK_RESET()
 * \brief Set default alignment for subsequent structure and union
 * definitions.
 */
#define COMPILER_PACK_RESET()          COMPILER_PRAGMA(pack())

/**
 * \brief Set user-defined section.
 * Place a data object or a function in a user-defined section.
 */
#if defined   ( __CC_ARM   )
    #define COMPILER_SECTION(a)    __attribute__((__section__(a)))
#elif defined ( __ICCARM__ )
    #define COMPILER_SECTION(a)    COMPILER_PRAGMA(location = a)
#elif defined (  __GNUC__  )
    #define COMPILER_SECTION(a)    __attribute__((__section__(a)))
#endif

/**
 * \brief Set aligned boundary.
 */
#if defined   ( __CC_ARM   )
    #define COMPILER_ALIGNED(a)    __attribute__((__aligned__(a)))
#elif defined ( __ICCARM__ )
    #define COMPILER_ALIGNED(a)    COMPILER_PRAGMA(data_alignment = a)
#elif defined (  __GNUC__  )
    #define COMPILER_ALIGNED(a)    __attribute__((__aligned__(a)))
#endif

/**
 * \brief Set word-aligned boundary.
 */

#if defined   ( __CC_ARM   )
    #define COMPILER_WORD_ALIGNED    __attribute__((__aligned__(4)))
#elif defined ( __ICCARM__ )
    #define COMPILER_WORD_ALIGNED    COMPILER_PRAGMA(data_alignment = 4)
#elif defined (  __GNUC__  )
    #define COMPILER_WORD_ALIGNED    __attribute__((__aligned__(4)))
#endif

/*! \name Usual Types
 */
//! @{
typedef unsigned char           Bool; //!< Boolean.
#ifndef __cplusplus
#if !defined(__bool_true_false_are_defined)
typedef unsigned char           bool; //!< Boolean.
#endif
#endif
typedef int8_t                  S8 ;  //!< 8-bit signed integer.
typedef uint8_t                 U8 ;  //!< 8-bit unsigned integer.
typedef int16_t                 S16;  //!< 16-bit signed integer.
typedef uint16_t                U16;  //!< 16-bit unsigned integer.
typedef uint16_t                le16_t;
typedef uint16_t                be16_t;
typedef int32_t                 S32;  //!< 32-bit signed integer.
typedef uint32_t                U32;  //!< 32-bit unsigned integer.
typedef uint32_t                le32_t;
typedef uint32_t                be32_t;
typedef int64_t                 S64;  //!< 64-bit signed integer.
typedef uint64_t                U64;  //!< 64-bit unsigned integer.
typedef float                   F32;  //!< 32-bit floating-point number.
typedef double                  F64;  //!< 64-bit floating-point number.
typedef uint32_t                iram_size_t;

/*! \name Mathematics
 *
 * The same considerations as for clz and ctz apply here but GCC does not
 * provide built-in functions to access the assembly instructions abs, min and
 * max and it does not produce them by itself in most cases, so two sets of
 * macros are defined here:
 *   - Abs, Min and Max to apply to constant expressions (values known at
 *     compile time);
 *   - abs, min and max to apply to non-constant expressions (values unknown at
 *     compile time), abs is found in stdlib.h.
 */
//! @{

/*! \brief Takes the absolute value of \a a.
 *
 * \param a Input value.
 *
 * \return Absolute value of \a a.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Abs(a)              (((a) <  0 ) ? -(a) : (a))

/*! \brief Takes the minimal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Minimal value of \a a and \a b.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Min(a, b)           (((a) < (b)) ?  (a) : (b))

/*! \brief Takes the maximal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Maximal value of \a a and \a b.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Max(a, b)           (((a) > (b)) ?  (a) : (b))

// abs() is already defined by stdlib.h

/*! \brief Takes the minimal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Minimal value of \a a and \a b.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
//#define min(a, b)   Min(a, b)

/*! \brief Takes the maximal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Maximal value of \a a and \a b.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#define max(a, b)   Max(a, b)

//! @}

#define  be32_to_cpu(x) __REV(x)
#define  cpu_to_be32(x) __REV(x)
#define  BE32_TO_CPU(x) __REV(x)
#define  CPU_TO_BE32(x) __REV(x)

/**
 * \def UNUSED
 * \brief Marking \a v as a unused parameter or value.
 */
#define UNUSED(v)          (void)(v)

/**
 * \weakgroup interrupt_group
 *
 * @{
 */

/**
 * \name Interrupt Service Routine definition
 *
 * @{
 */

/**
 * \brief Initialize interrupt vectors
 *
 * For NVIC the interrupt vectors are put in vector table. So nothing
 * to do to initialize them, except defined the vector function with
 * right name.
 *
 * This must be called prior to \ref irq_register_handler.
 */
#  define irq_initialize_vectors()   \
	do {                             \
	} while(0)

/**
 * \brief Register handler for interrupt
 *
 * For NVIC the interrupt vectors are put in vector table. So nothing
 * to do to register them, except defined the vector function with
 * right name.
 *
 * Usage:
 * \code
	irq_initialize_vectors();
	irq_register_handler(foo_irq_handler);
\endcode
 *
 * \note The function \a func must be defined with the \ref ISR macro.
 * \note The functions prototypes can be found in the device exception header
 *       files (exceptions.h).
 */
#  define irq_register_handler(int_num, int_prio)                      \
	NVIC_ClearPendingIRQ(    (IRQn_Type)int_num);                      \
	NVIC_SetPriority(    (IRQn_Type)int_num, int_prio);                \
	NVIC_EnableIRQ(      (IRQn_Type)int_num);                          \

//@}
      

#  define cpu_irq_enable()                     \
	do {                                       \
		/*g_interrupt_enabled = true; */           \
		__DMB();                               \
		__enable_irq();                        \
	} while (0)
#  define cpu_irq_disable()                    \
	do {                                       \
		__disable_irq();                       \
		__DMB();                               \
		/*g_interrupt_enabled = false; */          \
	} while (0)

typedef uint32_t irqflags_t;

#if !defined(__DOXYGEN__)
extern volatile char g_interrupt_enabled;
#endif

//#define cpu_irq_is_enabled()    (__get_PRIMASK() == 0)

static volatile uint32_t cpu_irq_critical_section_counter;
static volatile char     cpu_irq_prev_interrupt_state;

static inline irqflags_t cpu_irq_save(void)
{
	irqflags_t flags = cpu_irq_is_enabled();
	cpu_irq_disable();
	return flags;
}

static inline char cpu_irq_is_enabled_flags(irqflags_t flags)
{
	return (flags);
}

static inline void cpu_irq_restore(irqflags_t flags)
{
	if (cpu_irq_is_enabled_flags(flags))
		cpu_irq_enable();
}

void cpu_irq_enter_critical(void);
void cpu_irq_leave_critical(void);

/**
 * \weakgroup interrupt_deprecated_group
 * @{
 */

#define Enable_global_interrupt()            cpu_irq_enable()
#define Disable_global_interrupt()           cpu_irq_disable()
#define Is_global_interrupt_enabled()        cpu_irq_is_enabled()


//_____ M A C R O S ________________________________________________________

/*! \name Usual Constants
 */
//! @{
#define DISABLE   0
#define ENABLE    1
#define DISABLED  0
#define ENABLED   1
#define OFF       0
#define ON        1
#define FALSE     0
#define TRUE      1
#ifndef __cplusplus
#if !defined(__bool_true_false_are_defined)
#define false     FALSE
#define true      TRUE
#endif
#endif
#define KO        0
#define OK        1
#define PASS      0
#define FAIL      1
#define LOW       0
#define HIGH      1
#define CLR       0
#define SET       1
//! @}

/*! \brief Counts the trailing zero bits of the given value considered as a 32-bit integer.
 *
 * \param u Value of which to count the trailing zero bits.
 *
 * \return The count of trailing zero bits in \a u.
 */
#define ctz(u)              ((u) & (1ul <<  0) ?  0 : \
                                (u) & (1ul <<  1) ?  1 : \
                                (u) & (1ul <<  2) ?  2 : \
                                (u) & (1ul <<  3) ?  3 : \
                                (u) & (1ul <<  4) ?  4 : \
                                (u) & (1ul <<  5) ?  5 : \
                                (u) & (1ul <<  6) ?  6 : \
                                (u) & (1ul <<  7) ?  7 : \
                                (u) & (1ul <<  8) ?  8 : \
                                (u) & (1ul <<  9) ?  9 : \
                                (u) & (1ul << 10) ? 10 : \
                                (u) & (1ul << 11) ? 11 : \
                                (u) & (1ul << 12) ? 12 : \
                                (u) & (1ul << 13) ? 13 : \
                                (u) & (1ul << 14) ? 14 : \
                                (u) & (1ul << 15) ? 15 : \
                                (u) & (1ul << 16) ? 16 : \
                                (u) & (1ul << 17) ? 17 : \
                                (u) & (1ul << 18) ? 18 : \
                                (u) & (1ul << 19) ? 19 : \
                                (u) & (1ul << 20) ? 20 : \
                                (u) & (1ul << 21) ? 21 : \
                                (u) & (1ul << 22) ? 22 : \
                                (u) & (1ul << 23) ? 23 : \
                                (u) & (1ul << 24) ? 24 : \
                                (u) & (1ul << 25) ? 25 : \
                                (u) & (1ul << 26) ? 26 : \
                                (u) & (1ul << 27) ? 27 : \
                                (u) & (1ul << 28) ? 28 : \
                                (u) & (1ul << 29) ? 29 : \
                                (u) & (1ul << 30) ? 30 : \
                                (u) & (1ul << 31) ? 31 : \
                                32)

#endif // __ASSEMBLY__

/*! \name MCU Endianism Handling
 * ARM is MCU little endianism.
 */
//! @{
#define  MSB(u16)       (((U8  *)&(u16))[1]) //!< Most significant byte of \a u16.
#define  LSB(u16)       (((U8  *)&(u16))[0]) //!< Least significant byte of \a u16.

#define  MSH(u32)       (((U16 *)&(u32))[1]) //!< Most significant half-word of \a u32.
#define  LSH(u32)       (((U16 *)&(u32))[0]) //!< Least significant half-word of \a u32.
#define  MSB0W(u32)     (((U8  *)&(u32))[3]) //!< Most significant byte of 1st rank of \a u32.
#define  MSB1W(u32)     (((U8  *)&(u32))[2]) //!< Most significant byte of 2nd rank of \a u32.
#define  MSB2W(u32)     (((U8  *)&(u32))[1]) //!< Most significant byte of 3rd rank of \a u32.
#define  MSB3W(u32)     (((U8  *)&(u32))[0]) //!< Most significant byte of 4th rank of \a u32.
#define  LSB3W(u32)     MSB0W(u32)           //!< Least significant byte of 4th rank of \a u32.
#define  LSB2W(u32)     MSB1W(u32)           //!< Least significant byte of 3rd rank of \a u32.
#define  LSB1W(u32)     MSB2W(u32)           //!< Least significant byte of 2nd rank of \a u32.
#define  LSB0W(u32)     MSB3W(u32)           //!< Least significant byte of 1st rank of \a u32.

#define  MSW(u64)       (((U32 *)&(u64))[1]) //!< Most significant word of \a u64.
#define  LSW(u64)       (((U32 *)&(u64))[0]) //!< Least significant word of \a u64.
#define  MSH0(u64)      (((U16 *)&(u64))[3]) //!< Most significant half-word of 1st rank of \a u64.
#define  MSH1(u64)      (((U16 *)&(u64))[2]) //!< Most significant half-word of 2nd rank of \a u64.
#define  MSH2(u64)      (((U16 *)&(u64))[1]) //!< Most significant half-word of 3rd rank of \a u64.
#define  MSH3(u64)      (((U16 *)&(u64))[0]) //!< Most significant half-word of 4th rank of \a u64.
#define  LSH3(u64)      MSH0(u64)            //!< Least significant half-word of 4th rank of \a u64.
#define  LSH2(u64)      MSH1(u64)            //!< Least significant half-word of 3rd rank of \a u64.
#define  LSH1(u64)      MSH2(u64)            //!< Least significant half-word of 2nd rank of \a u64.
#define  LSH0(u64)      MSH3(u64)            //!< Least significant half-word of 1st rank of \a u64.
#define  MSB0D(u64)     (((U8  *)&(u64))[7]) //!< Most significant byte of 1st rank of \a u64.
#define  MSB1D(u64)     (((U8  *)&(u64))[6]) //!< Most significant byte of 2nd rank of \a u64.
#define  MSB2D(u64)     (((U8  *)&(u64))[5]) //!< Most significant byte of 3rd rank of \a u64.
#define  MSB3D(u64)     (((U8  *)&(u64))[4]) //!< Most significant byte of 4th rank of \a u64.
#define  MSB4D(u64)     (((U8  *)&(u64))[3]) //!< Most significant byte of 5th rank of \a u64.
#define  MSB5D(u64)     (((U8  *)&(u64))[2]) //!< Most significant byte of 6th rank of \a u64.
#define  MSB6D(u64)     (((U8  *)&(u64))[1]) //!< Most significant byte of 7th rank of \a u64.
#define  MSB7D(u64)     (((U8  *)&(u64))[0]) //!< Most significant byte of 8th rank of \a u64.
#define  LSB7D(u64)     MSB0D(u64)           //!< Least significant byte of 8th rank of \a u64.
#define  LSB6D(u64)     MSB1D(u64)           //!< Least significant byte of 7th rank of \a u64.
#define  LSB5D(u64)     MSB2D(u64)           //!< Least significant byte of 6th rank of \a u64.
#define  LSB4D(u64)     MSB3D(u64)           //!< Least significant byte of 5th rank of \a u64.
#define  LSB3D(u64)     MSB4D(u64)           //!< Least significant byte of 4th rank of \a u64.
#define  LSB2D(u64)     MSB5D(u64)           //!< Least significant byte of 3rd rank of \a u64.
#define  LSB1D(u64)     MSB6D(u64)           //!< Least significant byte of 2nd rank of \a u64.
#define  LSB0D(u64)     MSB7D(u64)           //!< Least significant byte of 1st rank of \a u64.

#define  BE16(x)        swap16(x)
#define  LE16(x)        (x)

#define  le16_to_cpu(x) (x)
#define  cpu_to_le16(x) (x)
#define  LE16_TO_CPU(x) (x)
#define  CPU_TO_LE16(x) (x)

#define  be16_to_cpu(x) swap16(x)
#define  cpu_to_be16(x) swap16(x)
#define  BE16_TO_CPU(x) swap16(x)
#define  CPU_TO_BE16(x) swap16(x)

#define  le32_to_cpu(x) (x)
#define  cpu_to_le32(x) (x)
#define  LE32_TO_CPU(x) (x)
#define  CPU_TO_LE32(x) (x)

#define div_ceil(a, b)      (((a) + (b) - 1) / (b))

#if defined(_ASSERT_ENABLE_)
#  if defined(TEST_SUITE_DEFINE_ASSERT_MACRO)
     // Assert() is defined in unit_test/suite.h
#    include "unit_test/suite.h"
#  else
#undef TEST_SUITE_DEFINE_ASSERT_MACRO
#    define Assert(expr) \
	{\
		if (!(expr)) while (true);\
	}
#  endif
#else
#  define Assert(expr) ((void) 0)
#endif

#endif  // _COMPILER_H_
