# Partner 步骤及循环推进
# address range: 0x4200745c..0x420074d4
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

4200745c <.data+0x743c>:
4200745c:	a5d604               	call8	0x4200c1c8
4200745f:	21f2e4               	l32r	a2, 0x42000828 (0x3fc9f199)
42007462:	31f3e4               	l32r	a3, 0x42000830 (0x3fc9f194)
42007465:	820200               	l8ui	a8, a2, 0
42007468:	71e6e4               	l32r	a7, 0x42000800 (0x3fc9ff48)
4200746b:	808890               	addx2	a8, a8, a8
4200746e:	9803                	l32i.n	a9, a3, 0
42007470:	7088a0               	addx4	a8, a8, a7
42007473:	88f8                	l32i.n	a8, a8, 60
42007475:	909ac0               	sub	a9, a10, a9
42007478:	4d0a                	mov.n	a4, a10
4200747a:	8739da               	bltu	a9, a8, 0x42007458
4200747d:	6565ff               	call8	0x42006ad4
42007480:	820200               	l8ui	a8, a2, 0
42007483:	920731               	l8ui	a9, a7, 49
42007486:	1b88                	addi.n	a8, a8, 1
42007488:	808074               	extui	a8, a8, 0, 8
4200748b:	824200               	s8i	a8, a2, 0
4200748e:	97382f               	bltu	a8, a9, 0x420074c1
42007491:	81e6e4               	l32r	a8, 0x4200082c (0x3fc9f198)
42007494:	a20730               	l8ui	a10, a7, 48
42007497:	920800               	l8ui	a9, a8, 0
4200749a:	524200               	s8i	a5, a2, 0
4200749d:	1b99                	addi.n	a9, a9, 1
4200749f:	909074               	extui	a9, a9, 0, 8
420074a2:	924800               	s8i	a9, a8, 0
420074a5:	0c08                	movi.n	a8, 0
420074a7:	a73916               	bltu	a9, a10, 0x420074c1
420074aa:	c02000               	memw
420074ad:	524600               	s8i	a5, a6, 0
420074b0:	65d0fb               	call8	0x420031b8
420074b3:	b1e3e4               	l32r	a11, 0x42000840 (0x3c0cb728)
420074b6:	a1e2e2               	l32r	a10, 0x42000040 (0x3fca0230)
420074b9:	cd07                	mov.n	a12, a7
420074bb:	651f04               	call8	0x4200b6b0
420074be:	86e5ff               	j	0x42007458
420074c1:	808890               	addx2	a8, a8, a8
420074c4:	7088a0               	addx4	a8, a8, a7
420074c7:	4903                	s32i.n	a4, a3, 0
420074c9:	a2c838               	addi	a10, a8, 56
420074cc:	6561ff               	call8	0x42006ae4
420074cf:	46e1ff               	j	0x42007458
	...
