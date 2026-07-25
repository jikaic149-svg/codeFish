# Demo 启动/取消入口
# address range: 0x42003f18..0x42003f34
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42003f18 <.data+0x3ef8>:
42003f18:	364100               	entry	a1, 32
42003f1b:	71aff0               	l32r	a7, 0x420001d8 (0x3fc9ebbc)
42003f1e:	c02000               	memw
42003f21:	822700               	l32i	a8, a7, 0
42003f24:	16c800               	beqz	a8, 0x42003f34
42003f27:	a2a001               	movi	a10, 1
42003f2a:	e5efff               	call8	0x42003e28
42003f2d:	21adf0               	l32r	a2, 0x420001e4 (0x3c0c0566)
42003f30:	1df0                	retw.n
	...
