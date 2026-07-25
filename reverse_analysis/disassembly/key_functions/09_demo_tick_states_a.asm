# Demo 状态分派前半
# address range: 0x42003fc1..0x42003fec
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42003fc1 <.data+0x3fa1>:
42003fc1:	652008               	call8	0x4200c1c8
42003fc4:	b18df0               	l32r	a11, 0x420001f8 (0x3fc9ebb8)
42003fc7:	c02000               	memw
42003fca:	9807                	l32i.n	a9, a7, 0
42003fcc:	880b                	l32i.n	a8, a11, 0
42003fce:	808ac0               	sub	a8, a10, a8
42003fd1:	663902               	bnei	a9, 3, 0x42003fd7
42003fd4:	061d00               	j	0x4200404c
42003fd7:	f64911               	bgeui	a9, 4, 0x42003fec
42003fda:	261922               	beqi	a9, 1, 0x42004000
42003fdd:	262946               	beqi	a9, 2, 0x42004027
42003fe0:	0c08                	movi.n	a8, 0
42003fe2:	c02000               	memw
42003fe5:	8907                	s32i.n	a8, a7, 0
42003fe7:	46f4ff               	j	0x42003fbc
	...
