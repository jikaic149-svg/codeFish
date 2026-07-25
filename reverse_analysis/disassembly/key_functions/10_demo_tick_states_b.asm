# Demo 状态分派后半
# address range: 0x42003fec..0x42004070
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42003fec <.data+0x3fcc>:
42003fec:	6649f0               	bnei	a9, 4, 0x42003fe0
42003fef:	92a4af               	movi	a9, 0x4af
42003ff2:	87b9c6               	bgeu	a9, a8, 0x42003fbc
42003ff5:	b186f0               	l32r	a11, 0x42000210 (0x3c0c066f)
42003ff8:	c02000               	memw
42003ffb:	6907                	s32i.n	a6, a7, 0
42003ffd:	460700               	j	0x4200401e
42004000:	9180f0               	l32r	a9, 0x42000200 (0xf9f)
42004003:	87b9b5               	bgeu	a9, a8, 0x42003fbc
42004006:	82a002               	movi	a8, 2
42004009:	a90b                	s32i.n	a10, a11, 0
4200400b:	c02000               	memw
4200400e:	8907                	s32i.n	a8, a7, 0
42004010:	a2a087               	movi	a10, 135
42004013:	65a9ff               	call8	0x42003aa8
42004016:	0c1a                	movi.n	a10, 1
42004018:	e5e6ff               	call8	0x42003e88
4200401b:	b17af0               	l32r	a11, 0x42000204 (0x3c0c063f)
4200401e:	a108f0               	l32r	a10, 0x42000040 (0x3fca0230)
42004021:	656f07               	call8	0x4200b718
42004024:	06e5ff               	j	0x42003fbc
42004027:	92a0ab               	movi	a9, 171
4200402a:	92d90d               	addmi	a9, a9, 0xd00
4200402d:	873902               	bltu	a9, a8, 0x42004033
42004030:	06e2ff               	j	0x42003fbc
42004033:	82a003               	movi	a8, 3
42004036:	a26b00               	s32i	a10, a11, 0
42004039:	c02000               	memw
4200403c:	8907                	s32i.n	a8, a7, 0
4200403e:	a5c9ff               	call8	0x42003cd8
42004041:	2cda                	movi.n	a10, 45
42004043:	65a6ff               	call8	0x42003aa8
42004046:	b170f0               	l32r	a11, 0x42000208 (0x3c0c064e)
42004049:	46f4ff               	j	0x4200401e
4200404c:	916df0               	l32r	a9, 0x42000200 (0xf9f)
4200404f:	873902               	bltu	a9, a8, 0x42004055
42004052:	86d9ff               	j	0x42003fbc
42004055:	82a004               	movi	a8, 4
42004058:	a26b00               	s32i	a10, a11, 0
4200405b:	c02000               	memw
4200405e:	8907                	s32i.n	a8, a7, 0
42004060:	65c7ff               	call8	0x42003cd8
42004063:	5caa                	movi.n	a10, 90
42004065:	25a4ff               	call8	0x42003aa8
42004068:	b169f0               	l32r	a11, 0x4200020c (0x3c0c0660)
4200406b:	c6ebff               	j	0x4200401e
	...
