# Partner 脚本安全停止
# address range: 0x42007288..0x420072bc
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42007288 <.data+0x7268>:
42007288:	364100               	entry	a1, 32
4200728b:	8160e5               	l32r	a8, 0x4200080c (0x3fc9f19a)
4200728e:	92a000               	movi	a9, 0
42007291:	c02000               	memw
42007294:	720800               	l8ui	a7, a8, 0
42007297:	202074               	extui	a2, a2, 0, 8
4200729a:	c02000               	memw
4200729d:	924800               	s8i	a9, a8, 0
420072a0:	707074               	extui	a7, a7, 0, 8
420072a3:	65f1fb               	call8	0x420031b8
420072a6:	162200               	beqz	a2, 0x420072ac
420072a9:	a582ff               	call8	0x42006ad4
420072ac:	8c77                	beqz.n	a7, 0x420072b7
420072ae:	b158e5               	l32r	a11, 0x42000810 (0x3c0cb657)
420072b1:	a163e3               	l32r	a10, 0x42000040 (0x3fca0230)
420072b4:	254604               	call8	0x4200b718
420072b7:	1df0                	retw.n
420072b9:	000000               	ill
