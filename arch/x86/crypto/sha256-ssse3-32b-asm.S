########################################################################
# Implement fast SHA-256 with SSSE3 instructions. (in 32b mode)
#
# Copyright (C) 2013 Intel Corporation.
#
# Authors:
#     James Guilford <james.guilford@intel.com>
#     Kirk Yap <kirk.s.yap@intel.com>
#     Tim Chen <tim.c.chen@linux.intel.com>
#
#     "Emulation" of original x86_64 code in 32b mode done by:
#     Vincent Stehlévincent.stehle@intel.com>
#
# This software is available to you under a choice of one of two
# licenses.  You may choose to be licensed under the terms of the GNU
# General Public License (GPL) Version 2, available from the file
# COPYING in the main directory of this source tree, or the
# OpenIB.org BSD license below:
#
#     Redistribution and use in source and binary forms, with or
#     without modification, are permitted provided that the following
#     conditions are met:
#
#      - Redistributions of source code must retain the above
#	copyright notice, this list of conditions and the following
#	disclaimer.
#
#      - Redistributions in binary form must reproduce the above
#	copyright notice, this list of conditions and the following
#	disclaimer in the documentation and/or other materials
#	provided with the distribution.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
########################################################################
#
# This code is described in an Intel White-Paper:
# "Fast SHA-256 Implementations on Intel Architecture Processors"
#
# To find it, surf to http://www.intel.com/p/en_US/embedded
# and search for that title.
#
########################################################################

#include <linux/linkage.h>

/* "Emulation" macros.
 * Notes:
 * We rely on eax / xmm0 for scratch.
 */

# This is a mem -> mem 32b mov
.macro	_mov	i, o
	mov	\i, %eax
	mov	%eax, \o
.endm

# Operands are aligned.
.macro	_movdqa	i, o
	movdqa	\i, %xmm0
	movdqa	%xmm0, \o
.endm

## assume buffers not aligned
#define MOVDQ movdqu

################################ Define Macros

# _addm [mem], [mem]
# Add mem to mem using reg-mem add and store
# Notes:
# - both operands end up with the same value
# - we use %eax as scratch
.macro _addm p1 p2
	mov	\p2, %eax
	add	\p1, %eax
	mov	%eax, \p2
	mov	%eax, \p1
.endm

################################

# _COPY_XMM_AND_BSWAP [mem], [mem], [mem byte_flip_mask]
# Load xmm with mem and byte swap each dword
# Notes:
# - p1 and p3 are aligned
# - p2 is not
# - We use %xmm0 as scratch
.macro _COPY_XMM_AND_BSWAP p1 p2 p3
	MOVDQ \p2, %xmm0
	pshufb \p3, %xmm0
	movdqa %xmm0, \p1
.endm

################################

X0 = _xmm4
X1 = _xmm5
X2 = _xmm6
X3 = _xmm7

# The XTMPx registers cannot be X0-3 (and scratch %xmm0)
XTMP0 = %xmm4
XTMP1 = %xmm1
XTMP2 = %xmm2
XTMP3 = %xmm3
XTMP4 = %xmm5
XFER = %xmm0		# cannot be X0

SHUF_00BA = _xmm10	# shuffle xBxA -> 00BA
SHUF_DC00 = _xmm11	# shuffle xDxC -> DC00
BYTE_FLIP_MASK = _xmm12

NUM_BLKS = %ecx		# 3rd arg
INP = %esi		# 2nd arg
CTX = %edi		# 1st arg

SRND = _esi
c = _ecx
d = _r8d
e = _edx
TBL = %ebx
a = _eax
b = _ebx

f = _r9d
g = _r10d
h = _r11d

# Note: the yX registers cannot be a-h, CTX, TBL
# (and %ebp, %esp and scratch %eax)
y0 = %ecx
y1 = %edx
y2 = %esi



_INP_END_SIZE = 8
_INP_SIZE = 8
_XFER_SIZE = 16
_XMM_SAVE_SIZE = 0
_XMM_SIZE = 16 * 16
_ERX_SIZE = 16 * 4
_SPILL_SIZE = 2 * 4

_INP_END 	= 0
_INP		= _INP_END	+ _INP_END_SIZE
_XFER		= _INP		+ _INP_SIZE
_XMM_SAVE	= _XFER		+ _XFER_SIZE
_XMM		= _XMM_SAVE	+ _XMM_SAVE_SIZE
_ERX		= _XMM		+ _XMM_SIZE
_SPILL		= _ERX		+ _ERX_SIZE
STACK_SIZE	= _SPILL	+ _SPILL_SIZE

.macro _declxmm off
_xmm\off = (_XMM + (16 * \off))
.endm

# _xmm0-3 unused
_declxmm 4
_declxmm 5
_declxmm 6
_declxmm 7
# _xmm8-9 unused
_declxmm 10
_declxmm 11
_declxmm 12
# _xmm13-15 unused

.macro _declerx name off
\name = (_ERX + (4 * \off))
.endm

_declerx _eax, 0
_declerx _ebx, 1
_declerx _ecx, 2
_declerx _edx, 3
# _ebp unused
_declerx _esi, 5
# _edi unused
# _esp unused
_declerx _r8d, 8
_declerx _r9d, 9
_declerx _r10d, 10
_declerx _r11d, 11
# _r12-15d unused

# rotate_Xs
# Rotate values of symbols X0...X3
.macro rotate_Xs
X_ = X0
X0 = X1
X1 = X2
X2 = X3
X3 = X_
.endm

# ROTATE_ARGS
# Rotate values of symbols a...h
.macro ROTATE_ARGS
TMP_ = h
h = g
g = f
f = e
e = d
d = c
c = b
b = a
a = TMP_
.endm

atmp = %ebp
etmp = TBL
htmp = %eax
x0tmp = %xmm7
x1tmp = %xmm6
spill0 = _SPILL
spill1 = _SPILL + 4

.macro FOUR_ROUNDS_AND_SCHED
	## compute s0 four at a time and s1 two at a time
	## compute W[-16] + W[-7] 4 at a time

	# Spill some registers.
	mov	atmp, spill0(%esp)
	mov	etmp, spill1(%esp)

	# Capture to tmp.
	mov	a(%esp), atmp
	mov	e(%esp), etmp
	mov	h(%esp), htmp

	movdqa	X3(%esp), XTMP0
	mov	etmp, y0		# y0 = e
	ror	$(25-11), y0		# y0 = e >> (25-11)
	mov	atmp, y1		# y1 = a
	palignr $4, X2(%esp), XTMP0	# XTMP0 = W[-7]
	ror	$(22-13), y1		# y1 = a >> (22-13)
	xor	etmp, y0		# y0 = e ^ (e >> (25-11))
	mov	f(%esp), y2		# y2 = f
	ror	$(11-6), y0		# y0 = (e >> (11-6)) ^ (e >> (25-6))
	movdqa	X1(%esp), XTMP1
	xor	atmp, y1		# y1 = a ^ (a >> (22-13)
	xor	g(%esp), y2		# y2 = f^g
	paddd	x0tmp, XTMP0		# XTMP0 = W[-7] + W[-16]
	xor	etmp, y0		# y0 = e ^ (e >> (11-6)) ^ (e >> (25-6))
	and	etmp, y2		# y2 = (f^g)&e
	ror	$(13-2), y1		# y1 = (a >> (13-2)) ^ (a >> (22-2))
	## compute s0
	palignr $4, x0tmp, XTMP1	# XTMP1 = W[-15]
	xor	atmp, y1		# y1 = a ^ (a >> (13-2)) ^ (a >> (22-2))
	ror	$6, y0			# y0 = S1 = (e>>6) & (e>>11) ^ (e>>25)
	xor	g(%esp), y2		# y2 = CH = ((f^g)&e)^g
	movdqa	XTMP1, XTMP2		# XTMP2 = W[-15]
	ror	$2, y1			# y1 = S0 = (a>>2) ^ (a>>13) ^ (a>>22)
	add	y0, y2			# y2 = S1 + CH
	add	_XFER(%esp) , y2	# y2 = k + w + S1 + CH
	movdqa	XTMP1, XTMP3		# XTMP3 = W[-15]
	mov	atmp, y0		# y0 = a
	add	y2, htmp		# h = h + S1 + CH + k + w
	mov	atmp, y2		# y2 = a
	pslld	$(32-7), XTMP1
	or	c(%esp), y0		# y0 = a|c
	add	htmp, d(%esp)		# d = d + h + S1 + CH + k + w
	and	c(%esp), y2		# y2 = a&c
	psrld	$7, XTMP2
	and	b(%esp), y0		# y0 = (a|c)&b
	add	y1, htmp		# h = h + S1 + CH + k + w + S0
	por	XTMP2, XTMP1		# XTMP1 = W[-15] ror 7
	or	y2, y0			# y0 = MAJ = (a|c)&b)|(a&c)
	add	y0, htmp		# h = h + S1 + CH + k + w + S0 + MAJ

	# Update tmp to stack
	# a and e are r/o; no need to update
	mov	htmp, h(%esp)

	ROTATE_ARGS

	# Capture to tmp.
	mov	a(%esp), atmp
	mov	e(%esp), etmp
	mov	h(%esp), htmp

	movdqa	XTMP3, XTMP2		# XTMP2 = W[-15]
	mov	etmp, y0		# y0 = e
	mov	atmp, y1		# y1 = a
	movdqa	XTMP3, XTMP4		# XTMP4 = W[-15]
	ror	$(25-11), y0		# y0 = e >> (25-11)
	xor	etmp, y0		# y0 = e ^ (e >> (25-11))
	mov	f(%esp), y2		# y2 = f
	ror	$(22-13), y1		# y1 = a >> (22-13)
	pslld	$(32-18), XTMP3
	xor	atmp, y1		# y1 = a ^ (a >> (22-13)
	ror	$(11-6), y0		# y0 = (e >> (11-6)) ^ (e >> (25-6))
	xor	g(%esp), y2		# y2 = f^g
	psrld	$18, XTMP2
	ror	$(13-2), y1		# y1 = (a >> (13-2)) ^ (a >> (22-2))
	xor	etmp, y0		# y0 = e ^ (e >> (11-6)) ^ (e >> (25-6))
	and	etmp, y2		# y2 = (f^g)&e
	ror	$6, y0			# y0 = S1 = (e>>6) & (e>>11) ^ (e>>25)
	pxor	XTMP3, XTMP1
	xor	atmp, y1		# y1 = a ^ (a >> (13-2)) ^ (a >> (22-2))
	xor	g(%esp), y2		# y2 = CH = ((f^g)&e)^g
	psrld	$3, XTMP4		# XTMP4 = W[-15] >> 3
	add	y0, y2			# y2 = S1 + CH
	add	(1*4 + _XFER)(%esp), y2 # y2 = k + w + S1 + CH
	ror	$2, y1			# y1 = S0 = (a>>2) ^ (a>>13) ^ (a>>22)
	pxor	XTMP2, XTMP1		# XTMP1 = W[-15] ror 7 ^ W[-15] ror 18
	mov	atmp, y0		# y0 = a
	add	y2, htmp		# h = h + S1 + CH + k + w
	mov	atmp, y2		# y2 = a
	pxor	XTMP4, XTMP1		# XTMP1 = s0
	or	c(%esp), y0		# y0 = a|c
	add	htmp, d(%esp)		# d = d + h + S1 + CH + k + w
	and	c(%esp), y2		# y2 = a&c
	## compute low s1
	pshufd	$0b11111010, X3(%esp), XTMP2	# XTMP2 = W[-2] {BBAA}
	and	b(%esp), y0		# y0 = (a|c)&b
	add	y1, htmp		# h = h + S1 + CH + k + w + S0
	paddd	XTMP1, XTMP0		# XTMP0 = W[-16] + W[-7] + s0
	or	y2, y0			# y0 = MAJ = (a|c)&b)|(a&c)
	add	y0, htmp		# h = h + S1 + CH + k + w + S0 + MAJ

	# Update tmp to stack
	# a and e are r/o; no need to update
	mov	htmp, h(%esp)

	ROTATE_ARGS

	# Capture to tmp.
	mov	a(%esp), atmp
	mov	e(%esp), etmp
	mov	h(%esp), htmp

	movdqa	XTMP2, XTMP3		# XTMP3 = W[-2] {BBAA}
	mov	etmp, y0		# y0 = e
	mov	atmp, y1		# y1 = a
	ror	$(25-11), y0		# y0 = e >> (25-11)
	movdqa	XTMP2, XTMP4		# XTMP4 = W[-2] {BBAA}
	xor	etmp, y0		# y0 = e ^ (e >> (25-11))
	ror	$(22-13), y1		# y1 = a >> (22-13)
	mov	f(%esp), y2		# y2 = f
	xor	atmp, y1		# y1 = a ^ (a >> (22-13)
	ror	$(11-6), y0		# y0 = (e >> (11-6)) ^ (e >> (25-6))
	psrlq	$17, XTMP2		# XTMP2 = W[-2] ror 17 {xBxA}
	xor	g(%esp), y2		# y2 = f^g
	psrlq	$19, XTMP3		# XTMP3 = W[-2] ror 19 {xBxA}
	xor	etmp, y0		# y0 = e ^ (e >> (11-6)) ^ (e >> (25-6))
	and	etmp, y2		# y2 = (f^g)&e
	psrld	$10, XTMP4		# XTMP4 = W[-2] >> 10 {BBAA}
	ror	$(13-2), y1		# y1 = (a >> (13-2)) ^ (a >> (22-2))
	xor	atmp, y1		# y1 = a ^ (a >> (13-2)) ^ (a >> (22-2))
	xor	g(%esp), y2		# y2 = CH = ((f^g)&e)^g
	ror	$6, y0			# y0 = S1 = (e>>6) & (e>>11) ^ (e>>25)
	pxor	XTMP3, XTMP2
	add	y0, y2			# y2 = S1 + CH
	ror	$2, y1			# y1 = S0 = (a>>2) ^ (a>>13) ^ (a>>22)
	add	(2*4 + _XFER)(%esp), y2 # y2 = k + w + S1 + CH
	pxor	XTMP2, XTMP4		# XTMP4 = s1 {xBxA}
	mov	atmp, y0		# y0 = a
	add	y2, htmp		# h = h + S1 + CH + k + w
	mov	atmp, y2		# y2 = a
	pshufb	SHUF_00BA(%esp), XTMP4	# XTMP4 = s1 {00BA}
	or	c(%esp), y0		# y0 = a|c
	add	htmp, d(%esp)		# d = d + h + S1 + CH + k + w
	and	c(%esp), y2		# y2 = a&c
	paddd	XTMP4, XTMP0		# XTMP0 = {..., ..., W[1], W[0]}
	and	b(%esp), y0		# y0 = (a|c)&b
	add	y1, htmp		# h = h + S1 + CH + k + w + S0
	## compute high s1
	pshufd	$0b01010000, XTMP0, XTMP2 # XTMP2 = W[-2] {BBAA}
	or	y2, y0			# y0 = MAJ = (a|c)&b)|(a&c)
	add	y0, htmp		# h = h + S1 + CH + k + w + S0 + MAJ

	# Update tmp to stack
	# a and e are r/o; no need to update
	mov	htmp, h(%esp)

	ROTATE_ARGS

	# Capture to tmp.
	mov	a(%esp), atmp
	mov	e(%esp), etmp
	mov	h(%esp), htmp

	movdqa	XTMP2, XTMP3		# XTMP3 = W[-2] {DDCC}
	mov	etmp, y0		# y0 = e
	ror	$(25-11), y0		# y0 = e >> (25-11)
	mov	atmp, y1		# y1 = a
	movdqa	XTMP2, x0tmp		# X0	= W[-2] {DDCC}
	ror	$(22-13), y1		# y1 = a >> (22-13)
	xor	etmp, y0		# y0 = e ^ (e >> (25-11))
	mov	f(%esp), y2		# y2 = f
	ror	$(11-6), y0		# y0 = (e >> (11-6)) ^ (e >> (25-6))
	psrlq	$17, XTMP2		# XTMP2 = W[-2] ror 17 {xDxC}
	xor	atmp, y1		# y1 = a ^ (a >> (22-13)
	xor	g(%esp), y2		# y2 = f^g
	psrlq	$19, XTMP3		# XTMP3 = W[-2] ror 19 {xDxC}
	xor	etmp, y0		# y0 = e ^ (e >> (11-6)) ^ (e >> (25
	and	etmp, y2		# y2 = (f^g)&e
	ror	$(13-2), y1		# y1 = (a >> (13-2)) ^ (a >> (22-2))
	psrld	$10, x0tmp		# X0 = W[-2] >> 10 {DDCC}
	xor	atmp, y1		# y1 = a ^ (a >> (13-2)) ^ (a >> (22
	ror	$6, y0			# y0 = S1 = (e>>6) & (e>>11) ^ (e>>2
	xor	g(%esp), y2		# y2 = CH = ((f^g)&e)^g
	pxor	XTMP3, XTMP2
	ror	$2, y1			# y1 = S0 = (a>>2) ^ (a>>13) ^ (a>>2
	add	y0, y2			# y2 = S1 + CH
	add	(3*4 + _XFER)(%esp), y2 # y2 = k + w + S1 + CH
	pxor	XTMP2, x0tmp		# X0 = s1 {xDxC}
	mov	atmp, y0		# y0 = a
	add	y2, htmp		# h = h + S1 + CH + k + w
	mov	atmp, y2		# y2 = a
	pshufb	SHUF_DC00(%esp), x0tmp	# X0 = s1 {DC00}
	or	c(%esp), y0		# y0 = a|c
	add	htmp, d(%esp)		# d = d + h + S1 + CH + k + w
	and	c(%esp), y2		# y2 = a&c
	paddd	XTMP0, x0tmp		# X0 = {W[3], W[2], W[1], W[0]}
	and	b(%esp), y0		# y0 = (a|c)&b
	add	y1, htmp		# h = h + S1 + CH + k + w + S0
	or	y2, y0			# y0 = MAJ = (a|c)&b)|(a&c)
	add	y0, htmp		# h = h + S1 + CH + k + w + S0 + MAJ

	# Update tmp to stack
	# a and e are r/o; no need to update
	mov	htmp, h(%esp)
	movdqa	x0tmp, X0(%esp)

	ROTATE_ARGS
	rotate_Xs

	# Unspill from stack.
	mov	spill0(%esp), atmp
	mov	spill1(%esp), etmp

.endm

## input is [esp + _XFER + %1 * 4]
.macro DO_ROUND round

	# Capture to tmp.
	mov	a(%esp), atmp
	mov	e(%esp), etmp
	mov	h(%esp), htmp

	mov	etmp, y0		# y0 = e
	ror	$(25-11), y0		# y0 = e >> (25-11)
	mov	atmp, y1		# y1 = a
	xor	etmp, y0		# y0 = e ^ (e >> (25-11))
	ror	$(22-13), y1		# y1 = a >> (22-13)
	mov	f(%esp), y2		# y2 = f
	xor	atmp, y1		# y1 = a ^ (a >> (22-13)
	ror	$(11-6), y0		# y0 = (e >> (11-6)) ^ (e >> (25-6))
	xor	g(%esp), y2		# y2 = f^g
	xor	etmp, y0		# y0 = e ^ (e >> (11-6)) ^ (e >> (25-6))
	ror	$(13-2), y1		# y1 = (a >> (13-2)) ^ (a >> (22-2))
	and	etmp, y2		# y2 = (f^g)&e
	xor	atmp, y1		# y1 = a ^ (a >> (13-2)) ^ (a >> (22-2))
	ror	$6, y0			# y0 = S1 = (e>>6) & (e>>11) ^ (e>>25)
	xor	g(%esp), y2		# y2 = CH = ((f^g)&e)^g
	add	y0, y2			# y2 = S1 + CH
	ror	$2, y1			# y1 = S0 = (a>>2) ^ (a>>13) ^ (a>>22)
	offset = \round * 4 + _XFER
	add	offset(%esp), y2	# y2 = k + w + S1 + CH
	mov	atmp, y0		# y0 = a
	add	y2, htmp		# h = h + S1 + CH + k + w
	mov	atmp, y2		# y2 = a
	or	c(%esp), y0		# y0 = a|c
	add	htmp, d(%esp)		# d = d + h + S1 + CH + k + w
	and	c(%esp), y2		# y2 = a&c
	and	b(%esp), y0		# y0 = (a|c)&b
	add	y1, htmp		# h = h + S1 + CH + k + w + S0
	or	y2, y0			# y0 = MAJ = (a|c)&b)|(a&c)
	add	y0, htmp		# h = h + S1 + CH + k + w + S0 + MAJ

	# Update tmp to stack
	# a and e are r/o; no need to update
	mov	htmp, h(%esp)

	ROTATE_ARGS
.endm

########################################################################
## void sha256_transform_ssse3(void *input_data, UINT32 digest[8],
##				UINT64 num_blks)
## arg 1 : pointer to input data
## arg 2 : pointer to digest
## arg 3 : Num blocks
## Note: num_blks is not really 64b as this is an unsigned, cast.
## TODO! really make PIC one day (see PSHUFFLE_BYTE_FLIP_MASK,
## _SHUF_00BA, _SHUF_DC00 and K256)
########################################################################
.text
ENTRY(sha256_transform_ssse3)
.align 32

	push	%ebx
	push    %ebp
	push	%esi
	push	%edi
	mov	%esp, %ebp

	sub     $STACK_SIZE, %esp
	and	$~15, %esp

	mov	(20 + 0)(%ebp), INP
	mov	(20 + 4)(%ebp), CTX
	mov	(20 + 8)(%ebp), NUM_BLKS

	shl     $6, NUM_BLKS			# convert to bytes
	jz      done_hash
	add     INP, NUM_BLKS
	mov     NUM_BLKS, _INP_END(%esp)	# pointer to end of data

	## load initial digest
	_mov     4*0(CTX), a(%esp)
	_mov     4*1(CTX), b(%esp)
	_mov     4*2(CTX), c(%esp)
	_mov     4*3(CTX), d(%esp)
	_mov     4*4(CTX), e(%esp)
	_mov     4*5(CTX), f(%esp)
	_mov     4*6(CTX), g(%esp)
	_mov     4*7(CTX), h(%esp)

	_movdqa  PSHUFFLE_BYTE_FLIP_MASK, BYTE_FLIP_MASK(%esp)
	_movdqa  _SHUF_00BA, SHUF_00BA(%esp)
	_movdqa  _SHUF_DC00, SHUF_DC00(%esp)

loop0:
	lea     K256, TBL

	## byte swap first 16 dwords
	_COPY_XMM_AND_BSWAP      X0(%esp), 0*16(INP), BYTE_FLIP_MASK(%esp)
	_COPY_XMM_AND_BSWAP      X1(%esp), 1*16(INP), BYTE_FLIP_MASK(%esp)
	_COPY_XMM_AND_BSWAP      X2(%esp), 2*16(INP), BYTE_FLIP_MASK(%esp)
	_COPY_XMM_AND_BSWAP      X3(%esp), 3*16(INP), BYTE_FLIP_MASK(%esp)

	mov     INP, _INP(%esp)

	## schedule 48 input dwords, by doing 3 rounds of 16 each
	_mov     $3, SRND(%esp)

	# Registers CTX, TBL are live here.
	# SRND, INP and NUM_BLKS are not.

.align 16
loop1:
	# Capture to tmp.
	movdqa  X0(%esp), x0tmp

	movdqa  (TBL), XFER
	paddd   x0tmp, XFER
	movdqa  XFER, _XFER(%esp)
	FOUR_ROUNDS_AND_SCHED

	# Capture to tmp.
	movdqa  X0(%esp), x0tmp

	movdqa  1*16(TBL), XFER
	paddd   x0tmp, XFER
	movdqa  XFER, _XFER(%esp)
	FOUR_ROUNDS_AND_SCHED

	# Capture to tmp.
	movdqa  X0(%esp), x0tmp

	movdqa  2*16(TBL), XFER
	paddd   x0tmp, XFER
	movdqa  XFER, _XFER(%esp)
	FOUR_ROUNDS_AND_SCHED

	# Capture to tmp.
	movdqa  X0(%esp), x0tmp

	movdqa  3*16(TBL), XFER
	paddd   x0tmp, XFER
	movdqa  XFER, _XFER(%esp)
	add     $4*16, TBL
	FOUR_ROUNDS_AND_SCHED

	sub     $1, SRND(%esp)
	jne     loop1

	_mov     $2, SRND(%esp)

	# Spill some registers.
	mov	atmp, spill0(%esp)
	mov	etmp, spill1(%esp)

	# Capture to tmp
	movdqa  X0(%esp), x0tmp
	movdqa  X1(%esp), x1tmp

loop2:
	paddd   (TBL), x0tmp
	movdqa  x0tmp, _XFER(%esp)

	DO_ROUND	0
	DO_ROUND	1
	DO_ROUND	2
	DO_ROUND	3

	# Unspill from stack.
	mov	spill0(%esp), atmp
	mov	spill1(%esp), etmp

	paddd   1*16(TBL), x1tmp
	movdqa  x1tmp, _XFER(%esp)
	add     $2*16, TBL

	# Spill some registers.
	mov	atmp, spill0(%esp)
	mov	etmp, spill1(%esp)

	DO_ROUND	0
	DO_ROUND	1
	DO_ROUND	2
	DO_ROUND	3

	# Unspill from stack.
	mov	spill0(%esp), atmp
	mov	spill1(%esp), etmp

	movdqa  X2(%esp), x0tmp
	movdqa  X3(%esp), x1tmp

	sub     $1, SRND(%esp)
	jne     loop2

	# Update tmp to stack
	movdqa	x0tmp, X0(%esp)
	movdqa	x1tmp, X1(%esp)

	_addm    (4*0)(CTX),a(%esp)
	_addm    (4*1)(CTX),b(%esp)
	_addm    (4*2)(CTX),c(%esp)
	_addm    (4*3)(CTX),d(%esp)
	_addm    (4*4)(CTX),e(%esp)
	_addm    (4*5)(CTX),f(%esp)
	_addm    (4*6)(CTX),g(%esp)
	_addm    (4*7)(CTX),h(%esp)

	mov     _INP(%esp), INP
	add     $64, INP
	cmp     _INP_END(%esp), INP
	jne     loop0

done_hash:

	mov	%ebp, %esp
	pop	%edi
	pop	%esi
	pop	%ebp
	pop	%ebx

	ret
ENDPROC(sha256_transform_ssse3)

.data
.align 64
K256:
	.long 0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5
	.long 0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5
	.long 0xd807aa98,0x12835b01,0x243185be,0x550c7dc3
	.long 0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174
	.long 0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc
	.long 0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da
	.long 0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7
	.long 0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967
	.long 0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13
	.long 0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85
	.long 0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3
	.long 0xd192e819,0xd6990624,0xf40e3585,0x106aa070
	.long 0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5
	.long 0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3
	.long 0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208
	.long 0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2

PSHUFFLE_BYTE_FLIP_MASK:
	.octa 0x0c0d0e0f08090a0b0405060700010203

# shuffle xBxA -> 00BA
_SHUF_00BA:
	.octa 0xFFFFFFFFFFFFFFFF0b0a090803020100

# shuffle xDxC -> DC00
_SHUF_DC00:
	.octa 0x0b0a090803020100FFFFFFFFFFFFFFFF

