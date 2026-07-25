# Partner 脚本解析与持久化
# address range: 0x420072bc..0x420073bc
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

420072bc <.data+0x729c>:
420072bc:	36e103               	entry	a1, 0x1f0
420072bf:	82a1cc               	movi	a8, 0x1cc
420072c2:	8a61                	add.n	a6, a1, a8
420072c4:	81d4e3               	l32r	a8, 0x42000214 (0x3fca08c0)
420072c7:	ad02                	mov.n	a10, a2
420072c9:	c02000               	memw
420072cc:	8808                	l32i.n	a8, a8, 0
420072ce:	ed05                	mov.n	a14, a5
420072d0:	c02000               	memw
420072d3:	8906                	s32i.n	a8, a6, 0
420072d5:	dd04                	mov.n	a13, a4
420072d7:	c2c114               	addi	a12, a1, 20
420072da:	bd03                	mov.n	a11, a3
420072dc:	101120               	or	a1, a1, a1
420072df:	6590ff               	call8	0x42006be4
420072e2:	7d02                	mov.n	a7, a2
420072e4:	2d0a                	mov.n	a2, a10
420072e6:	162a08               	beqz	a10, 0x4200736c
420072e9:	0c1a                	movi.n	a10, 1
420072eb:	101120               	or	a1, a1, a1
420072ee:	a5f9ff               	call8	0x42007288
420072f1:	a143e5               	l32r	a10, 0x42000800 (0x3fc9ff48)
420072f4:	c2a1b8               	movi	a12, 0x1b8
420072f7:	b2c114               	addi	a11, a1, 20
420072fa:	812ce4               	l32r	a8, 0x420003ac (0x400011f4)
420072fd:	e00800               	callx8	a8
42007300:	a141e5               	l32r	a10, 0x42000804 (0x3fc9f19b)
42007303:	cd03                	mov.n	a12, a3
42007305:	bd07                	mov.n	a11, a7
42007307:	8129e4               	l32r	a8, 0x420003ac (0x400011f4)
4200730a:	e00800               	callx8	a8
4200730d:	813de5               	l32r	a8, 0x42000804 (0x3fc9f19b)
42007310:	cb71                	addi.n	a7, a1, 12
42007312:	3a38                	add.n	a3, a8, a3
42007314:	0c08                	movi.n	a8, 0
42007316:	ad07                	mov.n	a10, a7
42007318:	824300               	s8i	a8, a3, 0
4200731b:	814ae3               	l32r	a8, 0x42000044 (0x420a5bc0)
4200731e:	e00800               	callx8	a8
42007321:	b131e5               	l32r	a11, 0x420007e8 (0x3c0cb59d)
42007324:	0c0d                	movi.n	a13, 0
42007326:	0c0c                	movi.n	a12, 0
42007328:	70a720               	or	a10, a7, a7
4200732b:	101120               	or	a1, a1, a1
4200732e:	250e03               	call8	0x4200a410
42007331:	c138e5               	l32r	a12, 0x42000814 (0x3c0cb670)
42007334:	2d0a                	mov.n	a2, a10
42007336:	9cca                	beqz.n	a10, 0x42007356
42007338:	c133e5               	l32r	a12, 0x42000804 (0x3fc9f19b)
4200733b:	b12de5               	l32r	a11, 0x420007f0 (0x3c0cb231)
4200733e:	cba1                	addi.n	a10, a1, 12
42007340:	101120               	or	a1, a1, a1
42007343:	251903               	call8	0x4200a4d4
42007346:	7d0a                	mov.n	a7, a10
42007348:	a2c10c               	addi	a10, a1, 12
4200734b:	101120               	or	a1, a1, a1
4200734e:	e51003               	call8	0x4200a45c
42007351:	ecd7                	bnez.n	a7, 0x42007382
42007353:	c131e5               	l32r	a12, 0x42000818 (0x3c0cb68f)
42007356:	bd05                	mov.n	a11, a5
42007358:	ad04                	mov.n	a10, a4
4200735a:	101120               	or	a1, a1, a1
4200735d:	6582ff               	call8	0x42006b84
42007360:	22a000               	movi	a2, 0
42007363:	a2c10c               	addi	a10, a1, 12
42007366:	101120               	or	a1, a1, a1
42007369:	a51003               	call8	0x4200a474
4200736c:	81aae3               	l32r	a8, 0x42000214 (0x3fca08c0)
4200736f:	c02000               	memw
42007372:	9806                	l32i.n	a9, a6, 0
42007374:	c02000               	memw
42007377:	8808                	l32i.n	a8, a8, 0
42007379:	87193c               	beq	a9, a8, 0x420073b9
4200737c:	81b5e3               	l32r	a8, 0x42000250 (0x4037d2e8)
4200737f:	e00800               	callx8	a8
42007382:	811fe5               	l32r	a8, 0x42000800 (0x3fc9ff48)
42007385:	b125e5               	l32r	a11, 0x4200081c (0x3c0cb6b8)
42007388:	d20831               	l8ui	a13, a8, 49
4200738b:	a12de3               	l32r	a10, 0x42000040 (0x3fca0230)
4200738e:	cd08                	mov.n	a12, a8
42007390:	101120               	or	a1, a1, a1
42007393:	e53104               	call8	0x4200b6b0
42007396:	46f2ff               	j	0x42007363
42007399:	cba1                	addi.n	a10, a1, 12
4200739b:	101120               	or	a1, a1, a1
4200739e:	650d03               	call8	0x4200a474
420073a1:	819ce3               	l32r	a8, 0x42000214 (0x3fca08c0)
420073a4:	c02000               	memw
420073a7:	9806                	l32i.n	a9, a6, 0
420073a9:	c02000               	memw
420073ac:	8808                	l32i.n	a8, a8, 0
420073ae:	8799ca               	bne	a9, a8, 0x4200737c
420073b1:	ad02                	mov.n	a10, a2
420073b3:	8125e3               	l32r	a8, 0x42000048 (0x420959b0)
420073b6:	e00800               	callx8	a8
420073b9:	1df0                	retw.n
	...
