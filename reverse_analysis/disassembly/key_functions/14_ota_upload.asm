# Web OTA 上传处理函数
# address range: 0x420054b0..0x420058ec
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

420054b0 <.data+0x5490>:
420054b0:	362109               	entry	a1, 0x490
420054b3:	8158eb               	l32r	a8, 0x42000214 (0x3fca08c0)
420054b6:	a2c110               	addi	a10, a1, 16
420054b9:	c02000               	memw
420054bc:	8808                	l32i.n	a8, a8, 0
420054be:	92a45c               	movi	a9, 0x45c
420054c1:	aa99                	add.n	a9, a9, a10
420054c3:	c02000               	memw
420054c6:	8909                	s32i.n	a8, a9, 0
420054c8:	82af60               	movi	a8, -160
420054cb:	aa88                	add.n	a8, a8, a10
420054cd:	62a0ac               	movi	a6, 172
420054d0:	6a68                	add.n	a6, a8, a6
420054d2:	5c0c                	movi.n	a12, 80
420054d4:	0c0b                	movi.n	a11, 0
420054d6:	ad06                	mov.n	a10, a6
420054d8:	81a3eb               	l32r	a8, 0x42000364 (0x400011e8)
420054db:	e00800               	callx8	a8
420054de:	41eaeb               	l32r	a4, 0x42000488 (0x3c0caa9f)
420054e1:	0c08                	movi.n	a8, 0
420054e3:	bd04                	mov.n	a11, a4
420054e5:	ad02                	mov.n	a10, a2
420054e7:	8961                	s32i.n	a8, a1, 24
420054e9:	522283               	l32i	a5, a2, 0x20c
420054ec:	a5d542               	call8	0x42048248
420054ef:	0baa                	addi.n	a10, a10, -1
420054f1:	4ce8                	movi.n	a8, 78
420054f3:	7d02                	mov.n	a7, a2
420054f5:	a7b859               	bgeu	a8, a10, 0x42005552
420054f8:	b1e5eb               	l32r	a11, 0x4200048c (0x3c0caaae)
420054fb:	a1d1ea               	l32r	a10, 0x42000040 (0x3fca0230)
420054fe:	a52106               	call8	0x4200b718
42005501:	b1e3eb               	l32r	a11, 0x42000490 (0x3c0caad5)
42005504:	ad02                	mov.n	a10, a2
42005506:	8194eb               	l32r	a8, 0x42000358 (0x420a7474)
42005509:	e00800               	callx8	a8
4200550c:	b17ceb               	l32r	a11, 0x420002fc (0x3c0c0b4c)
4200550f:	ad02                	mov.n	a10, a2
42005511:	817deb               	l32r	a8, 0x42000308 (0x420a7498)
42005514:	e00800               	callx8	a8
42005517:	c180eb               	l32r	a12, 0x42000318 (0x3c0dd73d)
4200551a:	b180eb               	l32r	a11, 0x4200031c (0x3c0c0ba4)
4200551d:	20a220               	or	a10, a2, a2
42005520:	654e43               	call8	0x42048a08
42005523:	b1dceb               	l32r	a11, 0x42000494 (0x3c0caae6)
42005526:	7cfc                	movi.n	a12, -1
42005528:	ad07                	mov.n	a10, a7
4200552a:	255243               	call8	0x42048a4c
4200552d:	7cf2                	movi.n	a2, -1
4200552f:	92c110               	addi	a9, a1, 16
42005532:	82a45c               	movi	a8, 0x45c
42005535:	9a88                	add.n	a8, a8, a9
42005537:	c02000               	memw
4200553a:	922800               	l32i	a9, a8, 0
4200553d:	8135eb               	l32r	a8, 0x42000214 (0x3fca08c0)
42005540:	c02000               	memw
42005543:	822800               	l32i	a8, a8, 0
42005546:	879902               	bne	a9, a8, 0x4200554c
42005549:	46e700               	j	0x420058ea
4200554c:	8141eb               	l32r	a8, 0x42000250 (0x4037d2e8)
4200554f:	e00800               	callx8	a8
42005552:	5c0d                	movi.n	a13, 80
42005554:	cd06                	mov.n	a12, a6
42005556:	bd04                	mov.n	a11, a4
42005558:	ad02                	mov.n	a10, a2
4200555a:	25d742               	call8	0x420482cc
4200555d:	56ca06               	bnez	a10, 0x420055cd
42005560:	ad06                	mov.n	a10, a6
42005562:	8194eb               	l32r	a8, 0x420003b4 (0x40001248)
42005565:	e00800               	callx8	a8
42005568:	42c16c               	addi	a4, a1, 108
4200556b:	bd0a                	mov.n	a11, a10
4200556d:	0c0d                	movi.n	a13, 0
4200556f:	cd04                	mov.n	a12, a4
42005571:	ad06                	mov.n	a10, a6
42005573:	25925e               	call8	0x42063e94
42005576:	563a05               	bnez	a10, 0x420055cd
42005579:	21c7eb               	l32r	a2, 0x42000498 (0x3c0cab08)
4200557c:	0c09                	movi.n	a9, 0
4200557e:	32a000               	movi	a3, 0
42005581:	a20200               	l8ui	a10, a2, 0
42005584:	926100               	s32i	a9, a1, 0
42005587:	81e3eb               	l32r	a8, 0x42000514 (0x420a5ae0)
4200558a:	e00800               	callx8	a8
4200558d:	6d0a                	mov.n	a6, a10
4200558f:	a20201               	l8ui	a10, a2, 1
42005592:	81e0eb               	l32r	a8, 0x42000514 (0x420a5ae0)
42005595:	e00800               	callx8	a8
42005598:	1bc6                	addi.n	a12, a6, 1
4200559a:	b0fc40               	nsau	a11, a12
4200559d:	b0b541               	srli	a11, a11, 5
420055a0:	ac9c                	beqz.n	a12, 0x420055cd
420055a2:	1bca                	addi.n	a12, a10, 1
420055a4:	0c18                	movi.n	a8, 1
420055a6:	c0b883               	moveqz	a11, a8, a12
420055a9:	ec0b                	bnez.n	a11, 0x420055cd
420055ab:	c06611               	slli	a6, a6, 4
420055ae:	a06620               	or	a6, a6, a10
420055b1:	3aa4                	add.n	a10, a4, a3
420055b3:	a20a00               	l8ui	a10, a10, 0
420055b6:	922100               	l32i	a9, a1, 0
420055b9:	a06630               	xor	a6, a6, a10
420055bc:	606920               	or	a6, a9, a6
420055bf:	32c301               	addi	a3, a3, 1
420055c2:	609074               	extui	a9, a6, 0, 8
420055c5:	22c202               	addi	a2, a2, 2
420055c8:	66c3b5               	bnei	a3, 32, 0x42005581
420055cb:	bc69                	beqz.n	a9, 0x42005605
420055cd:	b1b3eb               	l32r	a11, 0x4200049c (0x3c0cab49)
420055d0:	a19cea               	l32r	a10, 0x42000040 (0x3fca0230)
420055d3:	651406               	call8	0x4200b718
420055d6:	b1aeeb               	l32r	a11, 0x42000490 (0x3c0caad5)
420055d9:	70a720               	or	a10, a7, a7
420055dc:	815feb               	l32r	a8, 0x42000358 (0x420a7474)
420055df:	e00800               	callx8	a8
420055e2:	b146eb               	l32r	a11, 0x420002fc (0x3c0c0b4c)
420055e5:	70a720               	or	a10, a7, a7
420055e8:	8148eb               	l32r	a8, 0x42000308 (0x420a7498)
420055eb:	e00800               	callx8	a8
420055ee:	c14aeb               	l32r	a12, 0x42000318 (0x3c0dd73d)
420055f1:	b14aeb               	l32r	a11, 0x4200031c (0x3c0c0ba4)
420055f4:	70a720               	or	a10, a7, a7
420055f7:	254143               	call8	0x42048a08
420055fa:	b1a9eb               	l32r	a11, 0x420004a0 (0x3c0cab6d)
420055fd:	7cfc                	movi.n	a12, -1
420055ff:	46c9ff               	j	0x42005528
42005602:	000000               	ill
42005605:	256dfe               	call8	0x42003cd8
42005608:	318eea               	l32r	a3, 0x42000040 (0x3fca0230)
4200560b:	b1a6eb               	l32r	a11, 0x420004a4 (0x3c0cab8e)
4200560e:	30a320               	or	a10, a3, a3
42005611:	651006               	call8	0x4200b718
42005614:	e6150c               	bgei	a5, 1, 0x42005624
42005617:	b1a4eb               	l32r	a11, 0x420004a8 (0x3c0cabbb)
4200561a:	ad03                	mov.n	a10, a3
4200561c:	0c15                	movi.n	a5, 1
4200561e:	a50f06               	call8	0x4200b718
42005621:	b05501               	slli	a5, a5, 21
42005624:	b1a2eb               	l32r	a11, 0x420004ac (0x3c0cabf9)
42005627:	cd05                	mov.n	a12, a5
42005629:	30a320               	or	a10, a3, a3
4200562c:	250806               	call8	0x4200b6b0
4200562f:	0c0a                	movi.n	a10, 0
42005631:	a5c509               	call8	0x4200f28c
42005634:	4d0a                	mov.n	a4, a10
42005636:	ec4a                	bnez.n	a10, 0x4200565e
42005638:	b19eeb               	l32r	a11, 0x420004b0 (0x3c0cac18)
4200563b:	ad03                	mov.n	a10, a3
4200563d:	a50d06               	call8	0x4200b718
42005640:	b150eb               	l32r	a11, 0x42000380 (0x3c0ca760)
42005643:	ad07                	mov.n	a10, a7
42005645:	8144eb               	l32r	a8, 0x42000358 (0x420a7474)
42005648:	e00800               	callx8	a8
4200564b:	c133eb               	l32r	a12, 0x42000318 (0x3c0dd73d)
4200564e:	b133eb               	l32r	a11, 0x4200031c (0x3c0c0ba4)
42005651:	ad07                	mov.n	a10, a7
42005653:	653b43               	call8	0x42048a08
42005656:	b197eb               	l32r	a11, 0x420004b4 (0x3c0cac33)
42005659:	7cfc                	movi.n	a12, -1
4200565b:	46b2ff               	j	0x42005528
4200565e:	d22a04               	l32i	a13, a10, 16
42005661:	b195eb               	l32r	a11, 0x420004b8 (0x3c0cac57)
42005664:	c2ca18               	addi	a12, a10, 24
42005667:	30a320               	or	a10, a3, a3
4200566a:	650406               	call8	0x4200b6b0
4200566d:	822783               	l32i	a8, a7, 0x20c
42005670:	561800               	bnez	a8, 0x42005675
42005673:	5844                	l32i.n	a5, a4, 16
42005675:	c2c118               	addi	a12, a1, 24
42005678:	50b520               	or	a11, a5, a5
4200567b:	ad04                	mov.n	a10, a4
4200567d:	25e809               	call8	0x4200f500
42005680:	6d0a                	mov.n	a6, a10
42005682:	acba                	beqz.n	a10, 0x420056b1
42005684:	b18eeb               	l32r	a11, 0x420004bc (0x3c0cac7a)
42005687:	a0ca20               	or	a12, a10, a10
4200568a:	30a320               	or	a10, a3, a3
4200568d:	250206               	call8	0x4200b6b0
42005690:	b13ceb               	l32r	a11, 0x42000380 (0x3c0ca760)
42005693:	ad07                	mov.n	a10, a7
42005695:	8130eb               	l32r	a8, 0x42000358 (0x420a7474)
42005698:	e00800               	callx8	a8
4200569b:	c11feb               	l32r	a12, 0x42000318 (0x3c0dd73d)
4200569e:	b11feb               	l32r	a11, 0x4200031c (0x3c0c0ba4)
420056a1:	ad07                	mov.n	a10, a7
420056a3:	653643               	call8	0x42048a08
420056a6:	b186eb               	l32r	a11, 0x420004c0 (0x3c0cac91)
420056a9:	7cfc                	movi.n	a12, -1
420056ab:	469eff               	j	0x42005528
420056ae:	000000               	ill
420056b1:	b184eb               	l32r	a11, 0x420004c4 (0x3c0cacae)
420056b4:	ad03                	mov.n	a10, a3
420056b6:	250606               	call8	0x4200b718
420056b9:	222783               	l32i	a2, a7, 0x20c
420056bc:	0c18                	movi.n	a8, 1
420056be:	208283               	moveqz	a8, a2, a2
420056c1:	30f240               	nsau	a3, a2
420056c4:	8931                	s32i.n	a8, a1, 12
420056c6:	303541               	srli	a3, a3, 5
420056c9:	c2a400               	movi	a12, 0x400
420056cc:	b2c16c               	addi	a11, a1, 108
420056cf:	ad07                	mov.n	a10, a7
420056d1:	a58143               	call8	0x42048eec
420056d4:	8d0a                	mov.n	a8, a10
420056d6:	d66a08               	bgez	a10, 0x42005760
420056d9:	3b9a                	addi.n	a9, a10, 3
420056db:	562900               	bnez	a9, 0x420056e1
420056de:	5673fe               	bnez	a3, 0x420056c9
420056e1:	66084c               	bnei	a8, -1, 0x42005731
420056e4:	169304               	beqz	a3, 0x42005731
420056e7:	6156ea               	l32r	a6, 0x42000040 (0x3fca0230)
420056ea:	b177eb               	l32r	a11, 0x420004c8 (0x3c0cacbe)
420056ed:	ad06                	mov.n	a10, a6
420056ef:	a50206               	call8	0x4200b718
420056f2:	82af60               	movi	a8, -160
420056f5:	82c810               	addi	a8, a8, 16
420056f8:	1a88                	add.n	a8, a8, a1
420056fa:	a2282a               	l32i	a10, a8, 168
420056fd:	659709               	call8	0x4200f074
42005700:	cd0a                	mov.n	a12, a10
42005702:	3105eb               	l32r	a3, 0x42000318 (0x3c0dd73d)
42005705:	5105eb               	l32r	a5, 0x4200031c (0x3c0c0ba4)
42005708:	16ca16               	beqz	a10, 0x42005878
4200570b:	b17aeb               	l32r	a11, 0x420004f4 (0x3c0cad6c)
4200570e:	60a620               	or	a10, a6, a6
42005711:	e5f905               	call8	0x4200b6b0
42005714:	b11beb               	l32r	a11, 0x42000380 (0x3c0ca760)
42005717:	ad07                	mov.n	a10, a7
42005719:	810feb               	l32r	a8, 0x42000358 (0x420a7474)
4200571c:	e00800               	callx8	a8
4200571f:	cd03                	mov.n	a12, a3
42005721:	bd05                	mov.n	a11, a5
42005723:	ad07                	mov.n	a10, a7
42005725:	252e43               	call8	0x42048a08
42005728:	b174eb               	l32r	a11, 0x420004f8 (0x3c0cad83)
4200572b:	7cfc                	movi.n	a12, -1
4200572d:	c67dff               	j	0x42005528
42005730:	00b166               	excw
42005733:	eba1                	addi.n	a10, a1, 14
42005735:	43ea25               	excw
42005738:	fe05a22106259109 	{ l32r	a15, 0x41fedf4c; excw; excw }
42005740:	b110eb               	l32r	a11, 0x42000380 (0x3c0ca760)
42005743:	ad07                	mov.n	a10, a7
42005745:	8104eb               	l32r	a8, 0x42000358 (0x420a7474)
42005748:	e00800               	callx8	a8
4200574b:	c1f3ea               	l32r	a12, 0x42000318 (0x3c0dd73d)
4200574e:	b1f3ea               	l32r	a11, 0x4200031c (0x3c0c0ba4)
42005751:	ad07                	mov.n	a10, a7
42005753:	652b43               	call8	0x42048a08
42005756:	b15eeb               	l32r	a11, 0x420004d0 (0x3c0cacf0)
42005759:	7cfc                	movi.n	a12, -1
4200575b:	4672ff               	j	0x42005528
4200575e:	000016               	excw
42005761:	3af8                	add.n	a15, a8, a3
42005763:	cd0a                	mov.n	a12, a10
42005765:	a901                	s32i.n	a10, a1, 0
42005767:	a861                	l32i.n	a10, a1, 24
42005769:	b2c16c               	addi	a11, a1, 108
4200576c:	257409               	call8	0x4200eeb0
4200576f:	8801                	l32i.n	a8, a1, 0
42005771:	a0ca20               	or	a12, a10, a10
42005774:	acda                	beqz.n	a10, 0x420057a5
42005776:	b157eb               	l32r	a11, 0x420004d4 (0x3c0cad0b)
42005779:	a131ea               	l32r	a10, 0x42000040 (0x3fca0230)
4200577c:	25f305               	call8	0x4200b6b0
4200577f:	a861                	l32i.n	a10, a1, 24
42005781:	e58c09               	call8	0x4200f050
42005784:	b1ffea               	l32r	a11, 0x42000380 (0x3c0ca760)
42005787:	ad07                	mov.n	a10, a7
42005789:	81f3ea               	l32r	a8, 0x42000358 (0x420a7474)
4200578c:	e00800               	callx8	a8
4200578f:	c1e2ea               	l32r	a12, 0x42000318 (0x3c0dd73d)
42005792:	b1e2ea               	l32r	a11, 0x4200031c (0x3c0c0ba4)
42005795:	ad07                	mov.n	a10, a7
42005797:	252743               	call8	0x42048a08
4200579a:	b14feb               	l32r	a11, 0x420004d8 (0x3c0cad22)
4200579d:	7cfc                	movi.n	a12, -1
4200579f:	4661ff               	j	0x42005528
420057a2:	000000               	ill
420057a5:	914deb               	l32r	a9, 0x420004dc (0xcccccccd)
420057a8:	8a66                	add.n	a6, a6, a8
420057aa:	909682               	mull	a9, a6, a9
420057ad:	814ceb               	l32r	a8, 0x420004e0 (0x19999800)
420057b0:	8a99                	add.n	a9, a9, a8
420057b2:	814ceb               	l32r	a8, 0x420004e4 (0x66666)
420057b5:	004b40               	ssai	11
420057b8:	909981               	src	a9, a9, a9
420057bb:	908862               	excw
420057be:	0b88                	addi.n	a8, a8, -1
420057c0:	808060               	neg	a8, a8
420057c3:	808074               	extui	a8, a8, 0, 8
420057c6:	56a800               	bnez	a8, 0x420057d4
420057c9:	60a5c0               	sub	a10, a5, a6
420057cc:	0c19                	movi.n	a9, 1
420057ce:	a08983               	moveqz	a8, a9, a10
420057d1:	16c806               	beqz	a8, 0x42005841
420057d4:	0006ca               	float.s	f0, a6, 0
420057d7:	16d207               	beqz	a2, 0x42005858
420057da:	40a0fa               	rfr	a10, f0
420057dd:	814dea               	l32r	a8, 0x42000114 (0x400022a4)
420057e0:	e00800               	callx8	a8
420057e3:	d14beb               	l32r	a13, 0x42000510 (0x40590000)
420057e6:	0c0c                	movi.n	a12, 0
420057e8:	814cea               	l32r	a8, 0x42000118 (0x40002418)
420057eb:	e00800               	callx8	a8
420057ee:	a901                	s32i.n	a10, a1, 0
420057f0:	b911                	s32i.n	a11, a1, 4
420057f2:	50a520               	or	a10, a5, a5
420057f5:	8148eb               	l32r	a8, 0x42000518 (0x40002334)
420057f8:	e00800               	callx8	a8
420057fb:	cd0a                	mov.n	a12, a10
420057fd:	dd0b                	mov.n	a13, a11
420057ff:	a801                	l32i.n	a10, a1, 0
42005801:	b811                	l32i.n	a11, a1, 4
42005803:	8146ea               	l32r	a8, 0x4200011c (0x40002250)
42005806:	e00800               	callx8	a8
42005809:	8146ea               	l32r	a8, 0x42000124 (0x4000252c)
4200580c:	e00800               	callx8	a8
4200580f:	a901                	s32i.n	a10, a1, 0
42005811:	b135eb               	l32r	a11, 0x420004e8 (0x3c0cad3b)
42005814:	a10bea               	l32r	a10, 0x42000040 (0x3fca0230)
42005817:	cd06                	mov.n	a12, a6
42005819:	65e905               	call8	0x4200b6b0
4200581c:	a801                	l32i.n	a10, a1, 0
4200581e:	813dea               	l32r	a8, 0x42000114 (0x400022a4)
42005821:	e00800               	callx8	a8
42005824:	cd0a                	mov.n	a12, a10
42005826:	dd0b                	mov.n	a13, a11
42005828:	160204               	beqz	a2, 0x4200586c
4200582b:	ed0a                	mov.n	a14, a10
4200582d:	fd0b                	mov.n	a15, a11
4200582f:	a104ea               	l32r	a10, 0x42000040 (0x3fca0230)
42005832:	b12eeb               	l32r	a11, 0x420004ec (0x3c0cad4d)
42005835:	50c520               	or	a12, a5, a5
42005838:	65e705               	call8	0x4200b6b0
4200583b:	a101ea               	l32r	a10, 0x42000040 (0x3fca0230)
4200583e:	e5ea05               	call8	0x4200b6ec
42005841:	508672               	excw
42005844:	0b88                	addi.n	a8, a8, -1
42005846:	808060               	neg	a8, a8
42005849:	808074               	extui	a8, a8, 0, 8
4200584c:	1698e7               	beqz	a8, 0x420056c9
4200584f:	8831                	l32i.n	a8, a1, 12
42005851:	1648e7               	beqz	a8, 0x420056c9
42005854:	c6a3ff               	j	0x420056e7
42005857:	0082a0               	addx4	a8, a2, a0
4200585a:	759088               	call12	0x41f8e160
4200585d:	015018               	l32r	a0, 0x41fcb9a0
42005860:	fa10                	add.n	a1, a0, a15
42005862:	002a03               	excw
42005865:	410086               	l32r	a4, 0x41fe7068
42005868:	e9ff                	s32i.n	a14, a15, 60
4200586a:	0000b1               	sra	a0, a0
4200586d:	21eba1               	l32r	a2, 0x41fee01c
42005870:	f4e9e5               	excw
42005873:	e30586               	lsi	f14, a5, 0x218
42005876:	f0ff40               	nsau	a15, a15
42005879:	a42025               	excw
4200587c:	97092d               	bnone	a9, a9, 0x420058ad
4200587f:	0aac                	add.n	a10, a12, a0
42005881:	4ab1                	add.n	a11, a1, a4
42005883:	1eebcd0aad0665e2 	{ excw; excw; srai	a6, a5, 2 }
4200588b:	05b1bd               	call0	0x41fc339c
4200588e:	eaad                	add.n	a10, a13, a14
42005890:	0781b1               	bany	a1, a0, 0x42005845
42005893:	eae0                	add.n	a14, a0, a14
42005895:	0800                	l32i.n	a0, a0, 0
42005897:	cd03                	mov.n	a12, a3
42005899:	bd05                	mov.n	a11, a5
4200589b:	ad07                	mov.n	a10, a7
4200589d:	a51643               	call8	0x42048a08
420058a0:	b118eb               	l32r	a11, 0x42000500 (0x3c0cadc5)
420058a3:	7cfc                	movi.n	a12, -1
420058a5:	c61fff               	j	0x42005528
420058a8:	b117eb               	l32r	a11, 0x42000504 (0x3c0cadeb)
420058ab:	ad06                	mov.n	a10, a6
420058ad:	a5e605               	call8	0x4200b718
420058b0:	b116eb               	l32r	a11, 0x42000508 (0x3c0cae20)
420058b3:	70a720               	or	a10, a7, a7
420058b6:	81a8ea               	l32r	a8, 0x42000358 (0x420a7474)
420058b9:	e00800               	callx8	a8
420058bc:	b190ea               	l32r	a11, 0x420002fc (0x3c0c0b4c)
420058bf:	70a720               	or	a10, a7, a7
420058c2:	8191ea               	l32r	a8, 0x42000308 (0x420a7498)
420058c5:	e00800               	callx8	a8
420058c8:	cd03                	mov.n	a12, a3
420058ca:	bd05                	mov.n	a11, a5
420058cc:	ad07                	mov.n	a10, a7
420058ce:	a51343               	call8	0x42048a08
420058d1:	b10eeb               	l32r	a11, 0x4200050c (0x3c0cae27)
420058d4:	7cfc                	movi.n	a12, -1
420058d6:	ad07                	mov.n	a10, a7
420058d8:	251743               	call8	0x42048a4c
420058db:	a2a3e8               	movi	a10, 0x3e8
420058de:	259006               	call8	0x4200c1e0
420058e1:	a1bbea               	l32r	a10, 0x420003d0 (0x3fca022c)
420058e4:	a50005               	call8	0x4200a8f0
420058e7:	0611ff               	j	0x4200552f
420058ea:	1df0                	retw.n
