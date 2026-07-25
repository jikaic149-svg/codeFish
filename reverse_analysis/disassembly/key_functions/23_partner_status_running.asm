# 计算运行中步骤剩余时间
# address range: 0x4200755e..0x42007594
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

4200755e <.data+0x753e>:
4200755e:	51b2e4               	l32r	a5, 0x42000828 (0x3fc9f199)
42007561:	820731               	l8ui	a8, a7, 49
42007564:	920500               	l8ui	a9, a5, 0
42007567:	87b997               	bgeu	a9, a8, 0x42007502
4200756a:	e5c504               	call8	0x4200c1c8
4200756d:	81b0e4               	l32r	a8, 0x42000830 (0x3fc9f194)
42007570:	922800               	l32i	a9, a8, 0
42007573:	820500               	l8ui	a8, a5, 0
42007576:	90bac0               	sub	a11, a10, a9
42007579:	808890               	addx2	a8, a8, a8
4200757c:	7088a0               	addx4	a8, a8, a7
4200757f:	82280f               	l32i	a8, a8, 60
42007582:	873b02               	bltu	a11, a8, 0x42007588
42007585:	46deff               	j	0x42007502
42007588:	809980               	add	a9, a9, a8
4200758b:	a0a9c0               	sub	a10, a9, a10
4200758e:	86dcff               	j	0x42007504
42007591:	000000               	ill
