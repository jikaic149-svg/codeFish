# Partner 步骤推进入口
# address range: 0x42007438..0x4200745c
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42007438 <.data+0x7418>:
42007438:	364100               	entry	a1, 32
4200743b:	61f4e4               	l32r	a6, 0x4200080c (0x3fc9f19a)
4200743e:	c02000               	memw
42007441:	820600               	l8ui	a8, a6, 0
42007444:	808074               	extui	a8, a8, 0, 8
42007447:	16d800               	beqz	a8, 0x42007458
4200744a:	e527fc               	call8	0x420036c8
4200744d:	a05a20               	or	a5, a10, a10
42007450:	168a00               	beqz	a10, 0x4200745c
42007453:	0c0a                	movi.n	a10, 0
42007455:	25e3ff               	call8	0x42007288
42007458:	1df0                	retw.n
	...
