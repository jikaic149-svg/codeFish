# CPGTask 入口与 20 ms 周期
# address range: 0x42003a8c..0x42003aa8
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42003a8c <.data+0x3a6c>:
42003a8c:	364100               	entry	a1, 32
42003a8f:	b1a9f1               	l32r	a11, 0x42000134 (0x3c0c0269)
42003a92:	a16bf1               	l32r	a10, 0x42000040 (0x3fca0230)
42003a95:	25c807               	call8	0x4200b718
42003a98:	e5c6ff               	call8	0x42003708
42003a9b:	1c4a                	movi.n	a10, 20
42003a9d:	81a6f1               	l32r	a8, 0x42000138 (0x403803fc)
42003aa0:	e00800               	callx8	a8
42003aa3:	46fcff               	j	0x42003a98
	...
