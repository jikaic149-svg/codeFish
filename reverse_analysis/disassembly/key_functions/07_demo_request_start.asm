# Demo 启动路径
# address range: 0x42003f34..0x42003f9c
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42003f34 <.data+0x3f14>:
42003f34:	8166f0               	l32r	a8, 0x420000cc (0x3fc9ebc0)
42003f37:	2142f0               	l32r	a2, 0x42000040 (0x3fca0230)
42003f3a:	c02000               	memw
42003f3d:	820800               	l8ui	a8, a8, 0
42003f40:	808074               	extui	a8, a8, 0, 8
42003f43:	8ce8                	beqz.n	a8, 0x42003f55
42003f45:	b1aaf0               	l32r	a11, 0x420001f0 (0x3c0c0598)
42003f48:	ad02                	mov.n	a10, a2
42003f4a:	e57c07               	call8	0x4200b718
42003f4d:	21a6f0               	l32r	a2, 0x420001e8 (0x3c0c0575)
42003f50:	06f7ff               	j	0x42003f30
42003f53:	000081               	src	a0, a0, a0
42003f56:	46f0c0               	j	0x41ff431b
42003f59:	200092               	excw
42003f5c:	2800                	l32i.n	a2, a0, 0
42003f5e:	562901               	bnez	a9, 0x42003f74
42003f61:	0c19                	movi.n	a9, 1
42003f63:	c02000               	memw
42003f66:	926800               	s32i	a9, a8, 0
42003f69:	6572ff               	call8	0x42003690
42003f6c:	b1a2f0               	l32r	a11, 0x420001f4 (0x3c0c05d3)
42003f6f:	ad02                	mov.n	a10, a2
42003f71:	657a07               	call8	0x4200b718
42003f74:	0c18                	movi.n	a8, 1
42003f76:	c02000               	memw
42003f79:	8907                	s32i.n	a8, a7, 0
42003f7b:	e52408               	call8	0x4200c1c8
42003f7e:	819ef0               	l32r	a8, 0x420001f8 (0x3fc9ebb8)
42003f81:	a908                	s32i.n	a10, a8, 0
42003f83:	5caa                	movi.n	a10, 90
42003f85:	25b2ff               	call8	0x42003aa8
42003f88:	0c1a                	movi.n	a10, 1
42003f8a:	e5efff               	call8	0x42003e88
42003f8d:	b19bf0               	l32r	a11, 0x420001fc (0x3c0c0604)
42003f90:	ad02                	mov.n	a10, a2
42003f92:	657807               	call8	0x4200b718
42003f95:	2195f0               	l32r	a2, 0x420001ec (0x3c0c058b)
42003f98:	06e5ff               	j	0x42003f30
	...
