# AP/STA 双链路看门狗
# address range: 0x42003d60..0x42003e28
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42003d60 <.data+0x3d40>:
42003d60:	364100               	entry	a1, 32
42003d63:	654608               	call8	0x4200c1c8
42003d66:	9114f1               	l32r	a9, 0x420001b8 (0x3fc9ebc4)
42003d69:	a07a20               	or	a7, a10, a10
42003d6c:	8809                	l32i.n	a8, a9, 0
42003d6e:	808ac0               	sub	a8, a10, a8
42003d71:	a2a1f3               	movi	a10, 0x1f3
42003d74:	873a02               	bltu	a10, a8, 0x42003d7a
42003d77:	462800               	j	0x42003e1c
42003d7a:	8110f1               	l32r	a8, 0x420001bc (0x3fc9ebcc)
42003d7d:	726900               	s32i	a7, a9, 0
42003d80:	822800               	l32i	a8, a8, 0
42003d83:	92a097               	movi	a9, 151
42003d86:	8087c0               	sub	a8, a7, a8
42003d89:	92d93a               	addmi	a9, a9, 0x3a00
42003d8c:	873902               	bltu	a9, a8, 0x42003d92
42003d8f:	462200               	j	0x42003e1c
42003d92:	a10bf1               	l32r	a10, 0x420001c0 (0x3fca0144)
42003d95:	a55004               	call8	0x420082a0
42003d98:	6d0a                	mov.n	a6, a10
42003d9a:	a10af1               	l32r	a10, 0x420001c4 (0x3fca0110)
42003d9d:	65b204               	call8	0x420088c4
42003da0:	0c1b                	movi.n	a11, 1
42003da2:	8109f1               	l32r	a8, 0x420001c8 (0x3fc9ebc8)
42003da5:	60b683               	moveqz	a11, a6, a6
42003da8:	cc96                	bnez.n	a6, 0x42003db5
42003daa:	a2cafd               	addi	a10, a10, -3
42003dad:	0c19                	movi.n	a9, 1
42003daf:	a09b93               	movnez	a9, a11, a10
42003db2:	168906               	beqz	a9, 0x42003e1e
42003db5:	0c09                	movi.n	a9, 0
42003db7:	9908                	s32i.n	a9, a8, 0
42003db9:	61c4f0               	l32r	a6, 0x420000cc (0x3fc9ebc0)
42003dbc:	c02000               	memw
42003dbf:	920600               	l8ui	a9, a6, 0
42003dc2:	909074               	extui	a9, a9, 0, 8
42003dc5:	565901               	bnez	a9, 0x42003dde
42003dc8:	8808                	l32i.n	a8, a8, 0
42003dca:	9c08                	beqz.n	a8, 0x42003dde
42003dcc:	92a0b8               	movi	a9, 184
42003dcf:	8087c0               	sub	a8, a7, a8
42003dd2:	92d90b               	addmi	a9, a9, 0xb00
42003dd5:	87b905               	bgeu	a9, a8, 0x42003dde
42003dd8:	a1fdf0               	l32r	a10, 0x420001cc (0x3c0c051b)
42003ddb:	a5f3ff               	call8	0x42003d14
42003dde:	c02000               	memw
42003de1:	820600               	l8ui	a8, a6, 0
42003de4:	808074               	extui	a8, a8, 0, 8
42003de7:	bc18                	beqz.n	a8, 0x42003e1c
42003de9:	81f9f0               	l32r	a8, 0x420001d0 (0x1b4e81b5)
42003dec:	b1faf0               	l32r	a11, 0x420001d4 (0xaaaaaaab)
42003def:	8077a2               	muluh	a7, a7, a8
42003df2:	5c0a                	movi.n	a10, 80
42003df4:	82a0ff               	movi	a8, 255
42003df7:	707504               	extui	a7, a7, 5, 1
42003dfa:	70a893               	movnez	a10, a8, a7
42003dfd:	b0baa2               	muluh	a11, a10, a11
42003e00:	0c0c                	movi.n	a12, 0
42003e02:	b0b141               	srli	a11, a11, 1
42003e05:	2585ff               	call8	0x42003658
42003e08:	71cdf0               	l32r	a7, 0x4200013c (0x3fc97104)
42003e0b:	2cd6                	movi.n	a6, 45
42003e0d:	8807                	l32i.n	a8, a7, 0
42003e0f:	671809               	beq	a8, a6, 0x42003e1c
42003e12:	a1cbf0               	l32r	a10, 0x42000140 (0x3fc9ec70)
42003e15:	2cdb                	movi.n	a11, 45
42003e17:	a55c06               	call8	0x4200a3e0
42003e1a:	6907                	s32i.n	a6, a7, 0
42003e1c:	1df0                	retw.n
42003e1e:	9808                	l32i.n	a9, a8, 0
42003e20:	5659f9               	bnez	a9, 0x42003db9
42003e23:	7908                	s32i.n	a7, a8, 0
42003e25:	06e4ff               	j	0x42003db9
