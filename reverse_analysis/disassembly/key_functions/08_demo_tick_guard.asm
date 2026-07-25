# Demo 状态机入口
# address range: 0x42003f9c..0x42003fc1
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42003f9c <.data+0x3f7c>:
42003f9c:	364100               	entry	a1, 32
42003f9f:	718ef0               	l32r	a7, 0x420001d8 (0x3fc9ebbc)
42003fa2:	c02000               	memw
42003fa5:	8807                	l32i.n	a8, a7, 0
42003fa7:	9c18                	beqz.n	a8, 0x42003fbc
42003fa9:	8148f0               	l32r	a8, 0x420000cc (0x3fc9ebc0)
42003fac:	c02000               	memw
42003faf:	620800               	l8ui	a6, a8, 0
42003fb2:	606074               	extui	a6, a6, 0, 8
42003fb5:	8c86                	beqz.n	a6, 0x42003fc1
42003fb7:	0c0a                	movi.n	a10, 0
42003fb9:	e5e6ff               	call8	0x42003e28
42003fbc:	1df0                	retw.n
42003fbe:	000000               	ill
