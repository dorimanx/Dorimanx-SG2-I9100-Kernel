/*
 ---------------------------------------------------------------------------
 Copyright (c) 1998-2008, Brian Gladman, Worcester, UK. All rights reserved.

 LICENSE TERMS

 The redistribution and use of this software (with or without changes)
 is allowed without the payment of fees or royalties provided that:

  1. source code distributions include the above copyright notice, this
     list of conditions and the following disclaimer;

  2. binary distributions include the above copyright notice, this list
     of conditions and the following disclaimer in their documentation;

  3. the name of the copyright holder is not used to endorse products
     built using this software without specific written permission.

 DISCLAIMER

 This software is provided 'as is' with no explicit or implied warranties
 in respect of its properties, including, but not limited to, correctness
 and/or fitness for purpose.
 ---------------------------------------------------------------------------
 Issue Date: 20/12/2007

 The unsigned integer types defined here are of the form uint_<nn>t where
 <nn> is the length of the type; for example, the unsigned 32-bit type is
 'uint_32t'.  These are NOT the same as the 'C99 integer types' that are
 defined in the inttypes.h and stdint.h headers since attempts to use these
 types have shown that support for them is still highly variable.  However,
 since the latter are of the form uint<nn>_t, a regular expression search
 and replace (in VC++ search on 'uint_{:z}t' and replace with 'uint\1_t')
 can be used to convert the types used here to the C99 standard types.
*/

#ifndef _BRG_TYPES_H
#define _BRG_TYPES_H

#if defined(__cplusplus)
extern "C" {
#endif

#define ptrint_t unsigned int

#ifndef BRG_UI8
#define BRG_UI8
typedef unsigned char uint_8t;
#endif

#ifndef BRG_UI16
#define BRG_UI16
typedef unsigned short uint_16t;
#endif

#ifndef BRG_UI32
#define BRG_UI32
#define li_32(h) 0x##h##u
typedef unsigned int uint_32t;
#endif

#ifndef BRG_UI64
#define BRG_UI64
#define li_64(h) 0x##h##ull
typedef unsigned long long uint_64t;
#endif


#ifndef RETURN_VALUES
#define RETURN_VALUES
#define VOID_RETURN  void
#define INT_RETURN   int
#endif

/*	These defines are used to detect and set the memory alignment of pointers.
    Note that offsets are in bytes.

	ALIGN_OFFSET(x,n)			return the positive or zero offset of 
								the memory addressed by the pointer 'x' 
								from an address that is aligned on an 
								'n' byte boundary ('n' is a power of 2)

	ALIGN_FLOOR(x,n)			return a pointer that points to memory
								that is aligned on an 'n' byte boundary 
								and is not higher than the memory address
								pointed to by 'x' ('n' is a power of 2)

	ALIGN_CEIL(x,n)				return a pointer that points to memory
								that is aligned on an 'n' byte boundary 
								and is not lower than the memory address
								pointed to by 'x' ('n' is a power of 2)
*/

#define ALIGN_OFFSET(x,n)	(((ptrint_t)(x)) & ((n) - 1))
#define ALIGN_FLOOR(x,n)	((uint_8t*)(x) - ( ((ptrint_t)(x)) & ((n) - 1)))
#define ALIGN_CEIL(x,n)		((uint_8t*)(x) + (-((ptrint_t)(x)) & ((n) - 1)))

/*  These defines are used to declare buffers in a way that allows
    faster operations on longer variables to be used.  In all these
    defines 'size' must be a power of 2 and >= 8. NOTE that the 
    buffer size is in bytes but the type length is in bits

    UNIT_TYPEDEF(x,size)        declares a variable 'x' of length 
                                'size' bits

    BUFR_TYPEDEF(x,size,bsize)  declares a buffer 'x' of length 'bsize' 
                                bytes defined as an array of variables
                                each of 'size' bits (bsize must be a 
                                multiple of size / 8)

    UNIT_CAST(x,size)           casts a variable to a type of 
                                length 'size' bits

    UPTR_CAST(x,size)           casts a pointer to a pointer to a 
                                varaiable of length 'size' bits
*/

#define UI_TYPE(size)               uint_##size##t
#define UNIT_TYPEDEF(x,size)        typedef UI_TYPE(size) x
#define BUFR_TYPEDEF(x,size,bsize)  typedef UI_TYPE(size) x[bsize / (size >> 3)]
#define UNIT_CAST(x,size)           ((UI_TYPE(size) )(x))  
#define UPTR_CAST(x,size)           ((UI_TYPE(size)*)(x))

#if defined(__cplusplus)
}
#endif

#endif
