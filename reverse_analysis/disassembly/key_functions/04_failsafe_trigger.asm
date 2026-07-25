# 失联保护触发
# address range: 0x42003d14..0x42003d60
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42003d14 <.data+0x3cf4>:
42003d14:	364100               	entry	a1, 32
42003d17:	91edf0               	l32r	a9, 0x420000cc (0x3fc9ebc0)
42003d1a:	20c220               	or	a12, a2, a2
42003d1d:	c02000               	memw
42003d20:	820900               	l8ui	a8, a9, 0
42003d23:	808074               	extui	a8, a8, 0, 8
42003d26:	564803               	bnez	a8, 0x42003d5e
42003d29:	82a001               	movi	a8, 1
42003d2c:	b122f1               	l32r	a11, 0x420001b4 (0x3c0c04d7)
42003d2f:	a1c4f0               	l32r	a10, 0x42000040 (0x3fca0230)
42003d32:	c02000               	memw
42003d35:	824900               	s8i	a8, a9, 0
42003d38:	659707               	call8	0x4200b6b0
42003d3b:	e5f9ff               	call8	0x42003cd8
42003d3e:	a100f1               	l32r	a10, 0x42000140 (0x3fc9ec70)
42003d41:	b2a02d               	movi	a11, 45
42003d44:	a56906               	call8	0x4200a3e0
42003d47:	81fdf0               	l32r	a8, 0x4200013c (0x3fc97104)
42003d4a:	2cd9                	movi.n	a9, 45
42003d4c:	0c0a                	movi.n	a10, 0
42003d4e:	9908                	s32i.n	a9, a8, 0
42003d50:	a5d7ff               	call8	0x42003acc
42003d53:	0c0c                	movi.n	a12, 0
42003d55:	b2a078               	movi	a11, 120
42003d58:	a2a0ff               	movi	a10, 255
42003d5b:	e58fff               	call8	0x42003658
42003d5e:	1df0                	retw.n
