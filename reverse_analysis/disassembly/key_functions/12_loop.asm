# Arduino loop：安全与状态机调度
# address range: 0x42004864..0x4200487c
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42004864 <.data+0x4844>:
42004864:	364100               	entry	a1, 32
42004867:	a54fff               	call8	0x42003d60
4200486a:	2573ff               	call8	0x42003f9c
4200486d:	a5bc02               	call8	0x42007438
42004870:	a2a00a               	movi	a10, 10
42004873:	8131ee               	l32r	a8, 0x42000138 (0x403803fc)
42004876:	e00800               	callx8	a8
42004879:	1df0                	retw.n
	...
