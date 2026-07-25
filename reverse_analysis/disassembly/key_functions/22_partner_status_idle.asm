# 格式化空闲状态 JSON
# address range: 0x420074dc..0x4200755e
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

420074dc <.data+0x74bc>:
420074dc:	366100               	entry	a1, 48
420074df:	90f240               	nsau	a9, a2
420074e2:	909541               	srli	a9, a9, 5
420074e5:	163207               	beqz	a2, 0x4200755c
420074e8:	0c18                	movi.n	a8, 1
420074ea:	308993               	movnez	a8, a9, a3
420074ed:	56b806               	bnez	a8, 0x4200755c
420074f0:	61c7e4               	l32r	a6, 0x4200080c (0x3fc9f19a)
420074f3:	71c3e4               	l32r	a7, 0x42000800 (0x3fc9ff48)
420074f6:	c02000               	memw
420074f9:	820600               	l8ui	a8, a6, 0
420074fc:	808074               	extui	a8, a8, 0, 8
420074ff:	56b805               	bnez	a8, 0x4200755e
42007502:	0c0a                	movi.n	a10, 0
42007504:	c02000               	memw
42007507:	820600               	l8ui	a8, a6, 0
4200750a:	d1cfe4               	l32r	a13, 0x42000848 (0x3c0cb747)
4200750d:	808074               	extui	a8, a8, 0, 8
42007510:	8c18                	beqz.n	a8, 0x42007515
42007512:	d1cce4               	l32r	a13, 0x42000844 (0x3c0cb742)
42007515:	c02000               	memw
42007518:	820600               	l8ui	a8, a6, 0
4200751b:	0c0f                	movi.n	a15, 0
4200751d:	808074               	extui	a8, a8, 0, 8
42007520:	168800               	beqz	a8, 0x4200752c
42007523:	81c1e4               	l32r	a8, 0x42000828 (0x3fc9f199)
42007526:	f20800               	l8ui	a15, a8, 0
42007529:	f2cf01               	addi	a15, a15, 1
4200752c:	c02000               	memw
4200752f:	920600               	l8ui	a9, a6, 0
42007532:	81bee4               	l32r	a8, 0x4200082c (0x3fc9f198)
42007535:	909074               	extui	a9, a9, 0, 8
42007538:	b20731               	l8ui	a11, a7, 49
4200753b:	820800               	l8ui	a8, a8, 0
4200753e:	8c09                	beqz.n	a9, 0x42007542
42007540:	1b88                	addi.n	a8, a8, 1
42007542:	a931                	s32i.n	a10, a1, 12
42007544:	920730               	l8ui	a9, a7, 48
42007547:	c1c1e4               	l32r	a12, 0x4200084c (0x3c0cb74d)
4200754a:	b901                	s32i.n	a11, a1, 0
4200754c:	9921                	s32i.n	a9, a1, 8
4200754e:	8911                	s32i.n	a8, a1, 4
42007550:	ed07                	mov.n	a14, a7
42007552:	bd03                	mov.n	a11, a3
42007554:	ad02                	mov.n	a10, a2
42007556:	817fe3               	l32r	a8, 0x42000354 (0x4209d890)
42007559:	e00800               	callx8	a8
4200755c:	1df0                	retw.n
