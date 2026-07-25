# Partner 脚本启动
# address range: 0x420073bc..0x42007438
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

420073bc <.data+0x739c>:
420073bc:	364100               	entry	a1, 32
420073bf:	6110e5               	l32r	a6, 0x42000800 (0x3fc9ff48)
420073c2:	c117e5               	l32r	a12, 0x42000820 (0x3c0cb6d9)
420073c5:	820631               	l8ui	a8, a6, 49
420073c8:	169800               	beqz	a8, 0x420073d5
420073cb:	e52ffc               	call8	0x420036c8
420073ce:	7d0a                	mov.n	a7, a10
420073d0:	9c2a                	beqz.n	a10, 0x420073e6
420073d2:	c114e5               	l32r	a12, 0x42000824 (0x3c0cb6f0)
420073d5:	ad02                	mov.n	a10, a2
420073d7:	bd03                	mov.n	a11, a3
420073d9:	81dee3               	l32r	a8, 0x42000354 (0x4209d890)
420073dc:	e00800               	callx8	a8
420073df:	0c02                	movi.n	a2, 0
420073e1:	1df0                	retw.n
420073e3:	000000               	ill
420073e6:	0c1a                	movi.n	a10, 1
420073e8:	e5e9ff               	call8	0x42007288
420073eb:	0c1a                	movi.n	a10, 1
420073ed:	a5a3fc               	call8	0x42003e28
420073f0:	a52efc               	call8	0x420036dc
420073f3:	810de5               	l32r	a8, 0x42000828 (0x3fc9f199)
420073f6:	724800               	s8i	a7, a8, 0
420073f9:	810ce5               	l32r	a8, 0x4200082c (0x3fc9f198)
420073fc:	724800               	s8i	a7, a8, 0
420073ff:	a5dc04               	call8	0x4200c1c8
42007402:	810be5               	l32r	a8, 0x42000830 (0x3fc9f194)
42007405:	0c19                	movi.n	a9, 1
42007407:	a908                	s32i.n	a10, a8, 0
42007409:	8100e5               	l32r	a8, 0x4200080c (0x3fc9f19a)
4200740c:	a10ae5               	l32r	a10, 0x42000834 (0x3fc9ff80)
4200740f:	c02000               	memw
42007412:	924800               	s8i	a9, a8, 0
42007415:	e56cff               	call8	0x42006ae4
42007418:	c108e5               	l32r	a12, 0x42000838 (0x3c0cb704)
4200741b:	dd06                	mov.n	a13, a6
4200741d:	bd03                	mov.n	a11, a3
4200741f:	ad02                	mov.n	a10, a2
42007421:	81cce3               	l32r	a8, 0x42000354 (0x4209d890)
42007424:	e00800               	callx8	a8
42007427:	b105e5               	l32r	a11, 0x4200083c (0x3c0cb710)
4200742a:	a105e3               	l32r	a10, 0x42000040 (0x3fca0230)
4200742d:	cd06                	mov.n	a12, a6
4200742f:	252804               	call8	0x4200b6b0
42007432:	0c12                	movi.n	a2, 1
42007434:	46eaff               	j	0x420073e1
	...
