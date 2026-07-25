# 控制到达/退出失联保护
# address range: 0x420036dc..0x42003708
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

420036dc <.data+0x36bc>:
420036dc:	364100               	entry	a1, 32
420036df:	917bf2               	l32r	a9, 0x420000cc (0x3fc9ebc0)
420036e2:	c02000               	memw
420036e5:	820900               	l8ui	a8, a9, 0
420036e8:	808074               	extui	a8, a8, 0, 8
420036eb:	164801               	beqz	a8, 0x42003703
420036ee:	82a000               	movi	a8, 0
420036f1:	b177f2               	l32r	a11, 0x420000d0 (0x3c0c0219)
420036f4:	a153f2               	l32r	a10, 0x42000040 (0x3fca0230)
420036f7:	c02000               	memw
420036fa:	824900               	s8i	a8, a9, 0
420036fd:	a50108               	call8	0x4200b718
42003700:	e5f8ff               	call8	0x42003690
42003703:	1df0                	retw.n
42003705:	000000               	ill
