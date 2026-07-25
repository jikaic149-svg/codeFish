# 相机参数及机器鱼控制分派
# address range: 0x42005fac..0x4200657c
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42005fac <.data+0x5f8c>:
42005fac:	36e100               	entry	a1, 112
42005faf:	8199e8               	l32r	a8, 0x42000214 (0x3fca08c0)
42005fb2:	52c14c               	addi	a5, a1, 76
42005fb5:	c02000               	memw
42005fb8:	822800               	l32i	a8, a8, 0
42005fbb:	42c108               	addi	a4, a1, 8
42005fbe:	c02000               	memw
42005fc1:	8905                	s32i.n	a8, a5, 0
42005fc3:	bd04                	mov.n	a11, a4
42005fc5:	0c08                	movi.n	a8, 0
42005fc7:	ad02                	mov.n	a10, a2
42005fc9:	7d02                	mov.n	a7, a2
42005fcb:	8921                	s32i.n	a8, a1, 8
42005fcd:	2597ff               	call8	0x42005940
42005fd0:	dcea                	bnez.n	a10, 0x42005ff2
42005fd2:	4821                	l32i.n	a4, a1, 8
42005fd4:	b179e9               	l32r	a11, 0x420005b8 (0x3c0cafb1)
42005fd7:	2c0d                	movi.n	a13, 32
42005fd9:	cbc1                	addi.n	a12, a1, 12
42005fdb:	ad04                	mov.n	a10, a4
42005fdd:	e51642               	call8	0x4204814c
42005fe0:	ac9a                	beqz.n	a10, 0x4200600d
42005fe2:	ad04                	mov.n	a10, a4
42005fe4:	81ece8               	l32r	a8, 0x42000394 (0x403844c0)
42005fe7:	e00800               	callx8	a8
42005fea:	ad07                	mov.n	a10, a7
42005fec:	101120               	or	a1, a1, a1
42005fef:	2594ff               	call8	0x42005930
42005ff2:	7cf2                	movi.n	a2, -1
42005ff4:	8188e8               	l32r	a8, 0x42000214 (0x3fca08c0)
42005ff7:	c02000               	memw
42005ffa:	9805                	l32i.n	a9, a5, 0
42005ffc:	c02000               	memw
42005fff:	8808                	l32i.n	a8, a8, 0
42006001:	879902               	bne	a9, a8, 0x42006007
42006004:	065c01               	j	0x42006578
42006007:	8192e8               	l32r	a8, 0x42000250 (0x4037d2e8)
4200600a:	e00800               	callx8	a8
4200600d:	b15ce9               	l32r	a11, 0x42000580 (0x3c0debff)
42006010:	2c0d                	movi.n	a13, 32
42006012:	c2c12c               	addi	a12, a1, 44
42006015:	ad04                	mov.n	a10, a4
42006017:	651342               	call8	0x4204814c
4200601a:	2d0a                	mov.n	a2, a10
4200601c:	562afc               	bnez	a10, 0x42005fe2
4200601f:	ad04                	mov.n	a10, a4
42006021:	81dce8               	l32r	a8, 0x42000394 (0x403844c0)
42006024:	e00800               	callx8	a8
42006027:	656bfd               	call8	0x420036dc
4200602a:	a2c12c               	addi	a10, a1, 44
4200602d:	813be9               	l32r	a8, 0x4200051c (0x400014d0)
42006030:	e00800               	callx8	a8
42006033:	3d0a                	mov.n	a3, a10
42006035:	a5bf46               	call8	0x4204cc30
42006038:	b161e9               	l32r	a11, 0x420005bc (0x3c0cafb5)
4200603b:	4d0a                	mov.n	a4, a10
4200603d:	cba1                	addi.n	a10, a1, 12
4200603f:	8184e9               	l32r	a8, 0x42000650 (0x40001230)
42006042:	e00800               	callx8	a8
42006045:	ec8a                	bnez.n	a10, 0x42006071
42006047:	822402               	l32i	a8, a4, 8
4200604a:	66480e               	bnei	a8, 4, 0x4200605c
4200604d:	822410               	l32i	a8, a4, 64
42006050:	30b320               	or	a11, a3, a3
42006053:	40a420               	or	a10, a4, a4
42006056:	e00800               	callx8	a8
42006059:	967a4c               	bltz	a10, 0x42006524
4200605c:	c1afe8               	l32r	a12, 0x42000318 (0x3c0dd73d)
4200605f:	b1afe8               	l32r	a11, 0x4200031c (0x3c0c0ba4)
42006062:	70a720               	or	a10, a7, a7
42006065:	259a42               	call8	0x42048a08
42006068:	c2a000               	movi	a12, 0
4200606b:	b2a000               	movi	a11, 0
4200606e:	461901               	j	0x420064d7
42006071:	b153e9               	l32r	a11, 0x420005c0 (0x3c0cafbf)
42006074:	cba1                	addi.n	a10, a1, 12
42006076:	8176e9               	l32r	a8, 0x42000650 (0x40001230)
42006079:	e00800               	callx8	a8
4200607c:	cc5a                	bnez.n	a10, 0x42006085
4200607e:	822417               	l32i	a8, a4, 92
42006081:	c6f2ff               	j	0x42006050
42006084:	00b14f               	excw
42006087:	e9cb                	s32i.n	a14, a11, 48
42006089:	a18171               	l32r	a10, 0x41fe2690
4200608c:	e9e0                	s32i.n	a14, a0, 56
4200608e:	0800                	l32i.n	a0, a0, 0
42006090:	cc5a                	bnez.n	a10, 0x42006099
42006092:	822411               	l32i	a8, a4, 68
42006095:	c6edff               	j	0x42006050
42006098:	00b14b               	olt.s	b11, f1, f0
4200609b:	e9cb                	s32i.n	a14, a11, 48
4200609d:	a1816c               	l32r	a10, 0x41fe12a4
420060a0:	e9e0                	s32i.n	a14, a0, 56
420060a2:	0800                	l32i.n	a0, a0, 0
420060a4:	cc5a                	bnez.n	a10, 0x420060ad
420060a6:	822412               	l32i	a8, a4, 72
420060a9:	c6e8ff               	j	0x42006050
420060ac:	00b147               	excw
420060af:	e9cb                	s32i.n	a14, a11, 48
420060b1:	a18167               	l32r	a10, 0x41fdfeb8
420060b4:	e9e0                	s32i.n	a14, a0, 56
420060b6:	0800                	l32i.n	a0, a0, 0
420060b8:	cc4a                	bnez.n	a10, 0x420060c0
420060ba:	822413               	l32i	a8, a4, 76
420060bd:	c6e3ff               	j	0x42006050
420060c0:	b144e9               	l32r	a11, 0x420005d0 (0x3c0cafe6)
420060c3:	a2c10c               	addi	a10, a1, 12
420060c6:	8162e9               	l32r	a8, 0x42000650 (0x40001230)
420060c9:	e00800               	callx8	a8
420060cc:	cc5a                	bnez.n	a10, 0x420060d5
420060ce:	822416               	l32i	a8, a4, 88
420060d1:	c6deff               	j	0x42006050
420060d4:	00b13f               	excw
420060d7:	e9cb                	s32i.n	a14, a11, 48
420060d9:	a1815d               	l32r	a10, 0x41fdd6e0
420060dc:	e9e0                	s32i.n	a14, a0, 56
420060de:	0800                	l32i.n	a0, a0, 0
420060e0:	cc5a                	bnez.n	a10, 0x420060e9
420060e2:	822418               	l32i	a8, a4, 96
420060e5:	c6d9ff               	j	0x42006050
420060e8:	00b13b               	ueq.s	b11, f1, f0
420060eb:	e9cb                	s32i.n	a14, a11, 48
420060ed:	a18158               	l32r	a10, 0x41fdc2f4
420060f0:	e9e0                	s32i.n	a14, a0, 56
420060f2:	0800                	l32i.n	a0, a0, 0
420060f4:	cc5a                	bnez.n	a10, 0x420060fd
420060f6:	822419               	l32i	a8, a4, 100
420060f9:	c6d4ff               	j	0x42006050
420060fc:	00b137               	excw
420060ff:	e9cb                	s32i.n	a14, a11, 48
42006101:	a18153               	l32r	a10, 0x41fdaf08
42006104:	e9e0                	s32i.n	a14, a0, 56
42006106:	0800                	l32i.n	a0, a0, 0
42006108:	cc5a                	bnez.n	a10, 0x42006111
4200610a:	82241a               	l32i	a8, a4, 104
4200610d:	c6cfff               	j	0x42006050
42006110:	00b133               	clamps	a11, a1, 7
42006113:	e9cb                	s32i.n	a14, a11, 48
42006115:	a1814e               	l32r	a10, 0x41fd9b1c
42006118:	e9e0                	s32i.n	a14, a0, 56
4200611a:	0800                	l32i.n	a0, a0, 0
4200611c:	cc5a                	bnez.n	a10, 0x42006125
4200611e:	82241b               	l32i	a8, a4, 108
42006121:	c6caff               	j	0x42006050
42006124:	00b12f               	excw
42006127:	e9cb                	s32i.n	a14, a11, 48
42006129:	a18149               	l32r	a10, 0x41fd8730
4200612c:	e9e0                	s32i.n	a14, a0, 56
4200612e:	0800                	l32i.n	a0, a0, 0
42006130:	cc5a                	bnez.n	a10, 0x42006139
42006132:	82241c               	l32i	a8, a4, 112
42006135:	c6c5ff               	j	0x42006050
42006138:	00b12b               	oeq.s	b11, f1, f0
4200613b:	e9cb                	s32i.n	a14, a11, 48
4200613d:	a18144               	l32r	a10, 0x41fd7344
42006140:	e9e0                	s32i.n	a14, a0, 56
42006142:	0800                	l32i.n	a0, a0, 0
42006144:	cc5a                	bnez.n	a10, 0x4200614d
42006146:	82241d               	l32i	a8, a4, 116
42006149:	c6c0ff               	j	0x42006050
4200614c:	00b127               	excw
4200614f:	e9cb                	s32i.n	a14, a11, 48
42006151:	a1813f               	l32r	a10, 0x41fd5f58
42006154:	e9e0                	s32i.n	a14, a0, 56
42006156:	0800                	l32i.n	a0, a0, 0
42006158:	cc5a                	bnez.n	a10, 0x42006161
4200615a:	82241f               	l32i	a8, a4, 124
4200615d:	c6bbff               	j	0x42006050
42006160:	00b123               	sext	a11, a1, 7
42006163:	e9cb                	s32i.n	a14, a11, 48
42006165:	a1813a               	l32r	a10, 0x41fd4b6c
42006168:	e9e0                	s32i.n	a14, a0, 56
4200616a:	0800                	l32i.n	a0, a0, 0
4200616c:	cc5a                	bnez.n	a10, 0x42006175
4200616e:	822420               	l32i	a8, a4, 128
42006171:	c6b6ff               	j	0x42006050
42006174:	00b11f               	excw
42006177:	e9cb                	s32i.n	a14, a11, 48
42006179:	a18135               	l32r	a10, 0x41fd3780
4200617c:	e9e0                	s32i.n	a14, a0, 56
4200617e:	0800                	l32i.n	a0, a0, 0
42006180:	cc5a                	bnez.n	a10, 0x42006189
42006182:	822421               	l32i	a8, a4, 132
42006185:	c6b1ff               	j	0x42006050
42006188:	00b11b               	un.s	b11, f1, f0
4200618b:	e9cb                	s32i.n	a14, a11, 48
4200618d:	a18130               	l32r	a10, 0x41fd2394
42006190:	e9e0                	s32i.n	a14, a0, 56
42006192:	0800                	l32i.n	a0, a0, 0
42006194:	cc4a                	bnez.n	a10, 0x4200619c
42006196:	82241e               	l32i	a8, a4, 120
42006199:	c6acff               	j	0x42006050
4200619c:	b118e9               	l32r	a11, 0x420005fc (0x3c0cb036)
4200619f:	a2c10c               	addi	a10, a1, 12
420061a2:	812be9               	l32r	a8, 0x42000650 (0x40001230)
420061a5:	e00800               	callx8	a8
420061a8:	cc5a                	bnez.n	a10, 0x420061b1
420061aa:	822425               	l32i	a8, a4, 148
420061ad:	c6a7ff               	j	0x42006050
420061b0:	00b113               	wsr.epc1	a0
420061b3:	e9cb                	s32i.n	a14, a11, 48
420061b5:	a18126               	l32r	a10, 0x41fcfbbc
420061b8:	e9e0                	s32i.n	a14, a0, 56
420061ba:	0800                	l32i.n	a0, a0, 0
420061bc:	cc5a                	bnez.n	a10, 0x420061c5
420061be:	822426               	l32i	a8, a4, 152
420061c1:	c6a2ff               	j	0x42006050
420061c4:	00b10f               	excw
420061c7:	e9cb                	s32i.n	a14, a11, 48
420061c9:	a18121               	l32r	a10, 0x41fce7d0
420061cc:	e9e0                	s32i.n	a14, a0, 56
420061ce:	0800                	l32i.n	a0, a0, 0
420061d0:	cc5a                	bnez.n	a10, 0x420061d9
420061d2:	822427               	l32i	a8, a4, 156
420061d5:	c69dff               	j	0x42006050
420061d8:	00b10b               	excw
420061db:	e9cb                	s32i.n	a14, a11, 48
420061dd:	a1811c               	l32r	a10, 0x41fcd3e4
420061e0:	e9e0                	s32i.n	a14, a0, 56
420061e2:	0800                	l32i.n	a0, a0, 0
420061e4:	cc5a                	bnez.n	a10, 0x420061ed
420061e6:	822428               	l32i	a8, a4, 160
420061e9:	c698ff               	j	0x42006050
420061ec:	00b107               	excw
420061ef:	e9cb                	s32i.n	a14, a11, 48
420061f1:	a18117               	l32r	a10, 0x41fcbff8
420061f4:	e9e0                	s32i.n	a14, a0, 56
420061f6:	0800                	l32i.n	a0, a0, 0
420061f8:	cc5a                	bnez.n	a10, 0x42006201
420061fa:	822429               	l32i	a8, a4, 164
420061fd:	c693ff               	j	0x42006050
42006200:	00b103               	rsr.epc1	a0
42006203:	e9cb                	s32i.n	a14, a11, 48
42006205:	a18112               	l32r	a10, 0x41fcac0c
42006208:	e9e0                	s32i.n	a14, a0, 56
4200620a:	0800                	l32i.n	a0, a0, 0
4200620c:	cc5a                	bnez.n	a10, 0x42006215
4200620e:	822422               	l32i	a8, a4, 136
42006211:	c68eff               	j	0x42006050
42006214:	00b1ff               	excw
42006217:	e8cb                	l32i.n	a14, a11, 48
42006219:	a1810d               	l32r	a10, 0x41fc9820
4200621c:	e9e0                	s32i.n	a14, a0, 56
4200621e:	0800                	l32i.n	a0, a0, 0
42006220:	cc5a                	bnez.n	a10, 0x42006229
42006222:	822423               	l32i	a8, a4, 140
42006225:	c689ff               	j	0x42006050
42006228:	00b1fb               	excw
4200622b:	e8cb                	l32i.n	a14, a11, 48
4200622d:	a18108               	l32r	a10, 0x41fc8434
42006230:	e9e0                	s32i.n	a14, a0, 56
42006232:	0800                	l32i.n	a0, a0, 0
42006234:	cc5a                	bnez.n	a10, 0x4200623d
42006236:	822424               	l32i	a8, a4, 144
42006239:	c684ff               	j	0x42006050
4200623c:	00b1f7               	excw
4200623f:	e8a2                	l32i.n	a14, a2, 40
42006241:	c10c81               	l32r	a12, 0x41fe6674
42006244:	03e9e0               	excw
42006247:	0800                	l32i.n	a0, a0, 0
42006249:	568a01               	bnez	a10, 0x42006265
4200624c:	818de8               	l32r	a8, 0x42000480 (0x3fc9f190)
4200624f:	326800               	s32i	a3, a8, 0
42006252:	81cce8               	l32r	a8, 0x42000584 (0x3fc9f18c)
42006255:	820800               	l8ui	a8, a8, 0
42006258:	1608e0               	beqz	a8, 0x4200605c
4200625b:	0c1a                	movi.n	a10, 1
4200625d:	e5b3ff               	call8	0x42005d9c
42006260:	067eff               	j	0x4200605c
42006263:	0000b1               	sra	a0, a0
42006266:	eee8cba181f9e8e0 	{ l32r	a14, 0x41ff9208; j	0x41fffa84; srai	a14, a8, 0 }
4200626e:	0800                	l32i.n	a0, a0, 0
42006270:	6d0a                	mov.n	a6, a10
42006272:	563a04               	bnez	a10, 0x420062b9
42006275:	a2c12c               	addi	a10, a1, 44
42006278:	81a9e8               	l32r	a8, 0x4200051c (0x400014d0)
4200627b:	e00800               	callx8	a8
4200627e:	c126e8               	l32r	a12, 0x42000318 (0x3c0dd73d)
42006281:	b126e8               	l32r	a11, 0x4200031c (0x3c0c0ba4)
42006284:	4d0a                	mov.n	a4, a10
42006286:	ad07                	mov.n	a10, a7
42006288:	e57742               	call8	0x42048a08
4200628b:	c1e6e8               	l32r	a12, 0x42000624 (0x3c0cb08b)
4200628e:	b1e6e8               	l32r	a11, 0x42000628 (0x3c0cb096)
42006291:	ad07                	mov.n	a10, a7
42006293:	657742               	call8	0x42048a08
42006296:	0c0c                	movi.n	a12, 0
42006298:	0c0b                	movi.n	a11, 0
4200629a:	ad07                	mov.n	a10, a7
4200629c:	e57a42               	call8	0x42048a4c
4200629f:	0c1a                	movi.n	a10, 1
420062a1:	65fe00               	call8	0x42007288
420062a4:	0c0a                	movi.n	a10, 0
420062a6:	25b8fd               	call8	0x42003e28
420062a9:	0b44                	addi.n	a4, a4, -1
420062ab:	a2a001               	movi	a10, 1
420062ae:	40a693               	movnez	a10, a6, a4
420062b1:	65bdfd               	call8	0x42003e88
420062b4:	064fff               	j	0x42005ff4
420062b7:	0000b1               	sra	a0, a0
420062ba:	dce8                	bnez.n	a8, 0x420062dc
420062bc:	cba1                	addi.n	a10, a1, 12
420062be:	81e4e8               	l32r	a8, 0x42000650 (0x40001230)
420062c1:	e00800               	callx8	a8
420062c4:	563a04               	bnez	a10, 0x4200630b
420062c7:	a2c12c               	addi	a10, a1, 44
420062ca:	8194e8               	l32r	a8, 0x4200051c (0x400014d0)
420062cd:	e00800               	callx8	a8
420062d0:	c112e8               	l32r	a12, 0x42000318 (0x3c0dd73d)
420062d3:	b112e8               	l32r	a11, 0x4200031c (0x3c0c0ba4)
420062d6:	4d0a                	mov.n	a4, a10
420062d8:	ad07                	mov.n	a10, a7
420062da:	e57242               	call8	0x42048a08
420062dd:	c1d1e8               	l32r	a12, 0x42000624 (0x3c0cb08b)
420062e0:	b1d2e8               	l32r	a11, 0x42000628 (0x3c0cb096)
420062e3:	ad07                	mov.n	a10, a7
420062e5:	257242               	call8	0x42048a08
420062e8:	0c0c                	movi.n	a12, 0
420062ea:	b2a000               	movi	a11, 0
420062ed:	70a720               	or	a10, a7, a7
420062f0:	a57542               	call8	0x42048a4c
420062f3:	0c1a                	movi.n	a10, 1
420062f5:	25f900               	call8	0x42007288
420062f8:	0c0a                	movi.n	a10, 0
420062fa:	e5b2fd               	call8	0x42003e28
420062fd:	0ba4                	addi.n	a10, a4, -1
420062ff:	a0fa40               	nsau	a10, a10
42006302:	a0a541               	srli	a10, a10, 5
42006305:	a5d6fd               	call8	0x42004070
42006308:	063aff               	j	0x42005ff4
4200630b:	b1c9e8               	l32r	a11, 0x42000630 (0x3c0cb0b0)
4200630e:	cba1                	addi.n	a10, a1, 12
42006310:	81d0e8               	l32r	a8, 0x42000650 (0x40001230)
42006313:	e00800               	callx8	a8
42006316:	6d0a                	mov.n	a6, a10
42006318:	561a04               	bnez	a10, 0x4200635d
4200631b:	a2c12c               	addi	a10, a1, 44
4200631e:	817fe8               	l32r	a8, 0x4200051c (0x400014d0)
42006321:	e00800               	callx8	a8
42006324:	c1fde7               	l32r	a12, 0x42000318 (0x3c0dd73d)
42006327:	b1fde7               	l32r	a11, 0x4200031c (0x3c0c0ba4)
4200632a:	4d0a                	mov.n	a4, a10
4200632c:	ad07                	mov.n	a10, a7
4200632e:	a56d42               	call8	0x42048a08
42006331:	c1bce8               	l32r	a12, 0x42000624 (0x3c0cb08b)
42006334:	b1bde8               	l32r	a11, 0x42000628 (0x3c0cb096)
42006337:	ad07                	mov.n	a10, a7
42006339:	e56c42               	call8	0x42048a08
4200633c:	0c0c                	movi.n	a12, 0
4200633e:	0c0b                	movi.n	a11, 0
42006340:	ad07                	mov.n	a10, a7
42006342:	a57042               	call8	0x42048a4c
42006345:	0c1a                	movi.n	a10, 1
42006347:	25f400               	call8	0x42007288
4200634a:	0c0a                	movi.n	a10, 0
4200634c:	a5adfd               	call8	0x42003e28
4200634f:	0b44                	addi.n	a4, a4, -1
42006351:	0c1a                	movi.n	a10, 1
42006353:	40a693               	movnez	a10, a6, a4
42006356:	e5d8fd               	call8	0x420040e4
42006359:	c625ff               	j	0x42005ff4
4200635c:	00b1b5               	extui	a11, a0, 17, 12
4200635f:	e8cb                	l32i.n	a14, a11, 48
42006361:	a181bb               	l32r	a10, 0x41ff5168
42006364:	e8e0                	l32i.n	a14, a0, 56
42006366:	0800                	l32i.n	a0, a0, 0
42006368:	6d0a                	mov.n	a6, a10
4200636a:	563a04               	bnez	a10, 0x420063b1
4200636d:	a2c12c               	addi	a10, a1, 44
42006370:	816be8               	l32r	a8, 0x4200051c (0x400014d0)
42006373:	e00800               	callx8	a8
42006376:	c1e8e7               	l32r	a12, 0x42000318 (0x3c0dd73d)
42006379:	b1e8e7               	l32r	a11, 0x4200031c (0x3c0c0ba4)
4200637c:	4d0a                	mov.n	a4, a10
4200637e:	ad07                	mov.n	a10, a7
42006380:	656842               	call8	0x42048a08
42006383:	c1a8e8               	l32r	a12, 0x42000624 (0x3c0cb08b)
42006386:	b1a8e8               	l32r	a11, 0x42000628 (0x3c0cb096)
42006389:	ad07                	mov.n	a10, a7
4200638b:	e56742               	call8	0x42048a08
4200638e:	0c0c                	movi.n	a12, 0
42006390:	0c0b                	movi.n	a11, 0
42006392:	ad07                	mov.n	a10, a7
42006394:	656b42               	call8	0x42048a4c
42006397:	0c1a                	movi.n	a10, 1
42006399:	e5ee00               	call8	0x42007288
4200639c:	0c0a                	movi.n	a10, 0
4200639e:	a5a8fd               	call8	0x42003e28
420063a1:	0b44                	addi.n	a4, a4, -1
420063a3:	a2a001               	movi	a10, 1
420063a6:	40a693               	movnez	a10, a6, a4
420063a9:	25dffd               	call8	0x4200419c
420063ac:	0611ff               	j	0x42005ff4
420063af:	0000b1               	sra	a0, a0
420063b2:	a1e8a2               	l32r	a10, 0x41feef54
420063b5:	c10c81               	l32r	a12, 0x41fe67e8
420063b8:	a6e8e0               	blti	a8, 128, 0x4200639c
420063bb:	0800                	l32i.n	a0, a0, 0
420063bd:	560a03               	bnez	a10, 0x420063f1
420063c0:	c1d6e7               	l32r	a12, 0x42000318 (0x3c0dd73d)
420063c3:	b1d6e7               	l32r	a11, 0x4200031c (0x3c0c0ba4)
420063c6:	70a720               	or	a10, a7, a7
420063c9:	e56342               	call8	0x42048a08
420063cc:	c196e8               	l32r	a12, 0x42000624 (0x3c0cb08b)
420063cf:	b196e8               	l32r	a11, 0x42000628 (0x3c0cb096)
420063d2:	70a720               	or	a10, a7, a7
420063d5:	256342               	call8	0x42048a08
420063d8:	0c0c                	movi.n	a12, 0
420063da:	0c0b                	movi.n	a11, 0
420063dc:	ad07                	mov.n	a10, a7
420063de:	e56642               	call8	0x42048a4c
420063e1:	0c1a                	movi.n	a10, 1
420063e3:	65ea00               	call8	0x42007288
420063e6:	0c0a                	movi.n	a10, 0
420063e8:	e5a3fd               	call8	0x42003e28
420063eb:	e58efd               	call8	0x42003cd8
420063ee:	8600ff               	j	0x42005ff4
420063f1:	b192e8               	l32r	a11, 0x4200063c (0x3c0cb0d2)
420063f4:	cba1                	addi.n	a10, a1, 12
420063f6:	8196e8               	l32r	a8, 0x42000650 (0x40001230)
420063f9:	e00800               	callx8	a8
420063fc:	56da03               	bnez	a10, 0x4200643d
420063ff:	a2c12c               	addi	a10, a1, 44
42006402:	8146e8               	l32r	a8, 0x4200051c (0x400014d0)
42006405:	e00800               	callx8	a8
42006408:	c1c4e7               	l32r	a12, 0x42000318 (0x3c0dd73d)
4200640b:	b1c4e7               	l32r	a11, 0x4200031c (0x3c0c0ba4)
4200640e:	4d0a                	mov.n	a4, a10
42006410:	ad07                	mov.n	a10, a7
42006412:	655f42               	call8	0x42048a08
42006415:	c183e8               	l32r	a12, 0x42000624 (0x3c0cb08b)
42006418:	b184e8               	l32r	a11, 0x42000628 (0x3c0cb096)
4200641b:	ad07                	mov.n	a10, a7
4200641d:	a55e42               	call8	0x42048a08
42006420:	0c0c                	movi.n	a12, 0
42006422:	0c0b                	movi.n	a11, 0
42006424:	ad07                	mov.n	a10, a7
42006426:	656242               	call8	0x42048a4c
42006429:	0c1a                	movi.n	a10, 1
4200642b:	e5e500               	call8	0x42007288
4200642e:	0c0a                	movi.n	a10, 0
42006430:	659ffd               	call8	0x42003e28
42006433:	ad04                	mov.n	a10, a4
42006435:	2567fd               	call8	0x42003aa8
42006438:	06eefe               	j	0x42005ff4
4200643b:	0000b1               	sra	a0, a0
4200643e:	80e8cb               	movf.s	f14, f8, b8
42006441:	a18183               	l32r	a10, 0x41fe7248
42006444:	e8e0                	l32i.n	a14, a0, 56
42006446:	0800                	l32i.n	a0, a0, 0
42006448:	6d0a                	mov.n	a6, a10
4200644a:	563a04               	bnez	a10, 0x42006491
4200644d:	a2c12c               	addi	a10, a1, 44
42006450:	8133e8               	l32r	a8, 0x4200051c (0x400014d0)
42006453:	e00800               	callx8	a8
42006456:	c1b0e7               	l32r	a12, 0x42000318 (0x3c0dd73d)
42006459:	b1b0e7               	l32r	a11, 0x4200031c (0x3c0c0ba4)
4200645c:	4d0a                	mov.n	a4, a10
4200645e:	ad07                	mov.n	a10, a7
42006460:	655a42               	call8	0x42048a08
42006463:	c170e8               	l32r	a12, 0x42000624 (0x3c0cb08b)
42006466:	b170e8               	l32r	a11, 0x42000628 (0x3c0cb096)
42006469:	ad07                	mov.n	a10, a7
4200646b:	e55942               	call8	0x42048a08
4200646e:	0c0c                	movi.n	a12, 0
42006470:	0c0b                	movi.n	a11, 0
42006472:	ad07                	mov.n	a10, a7
42006474:	655d42               	call8	0x42048a4c
42006477:	0c1a                	movi.n	a10, 1
42006479:	e5e000               	call8	0x42007288
4200647c:	0c0a                	movi.n	a10, 0
4200647e:	a59afd               	call8	0x42003e28
42006481:	0b44                	addi.n	a4, a4, -1
42006483:	a2a001               	movi	a10, 1
42006486:	40a693               	movnez	a10, a6, a4
42006489:	2564fd               	call8	0x42003acc
4200648c:	06d9fe               	j	0x42005ff4
4200648f:	0000b1               	sra	a0, a0
42006492:	6ce8                	movi.n	a8, -18
42006494:	cba1                	addi.n	a10, a1, 12
42006496:	816ee8               	l32r	a8, 0x42000650 (0x40001230)
42006499:	e00800               	callx8	a8
4200649c:	561a04               	bnez	a10, 0x420064e1
4200649f:	a2a001               	movi	a10, 1
420064a2:	65de00               	call8	0x42007288
420064a5:	25a7fd               	call8	0x42003f18
420064a8:	b1c8e7               	l32r	a11, 0x420003c8 (0x3c0ca882)
420064ab:	4d0a                	mov.n	a4, a10
420064ad:	ad07                	mov.n	a10, a7
420064af:	8196e7               	l32r	a8, 0x42000308 (0x420a7498)
420064b2:	e00800               	callx8	a8
420064b5:	c198e7               	l32r	a12, 0x42000318 (0x3c0dd73d)
420064b8:	b199e7               	l32r	a11, 0x4200031c (0x3c0c0ba4)
420064bb:	ad07                	mov.n	a10, a7
420064bd:	a55442               	call8	0x42048a08
420064c0:	c159e8               	l32r	a12, 0x42000624 (0x3c0cb08b)
420064c3:	b159e8               	l32r	a11, 0x42000628 (0x3c0cb096)
420064c6:	ad07                	mov.n	a10, a7
420064c8:	e55342               	call8	0x42048a08
420064cb:	ad04                	mov.n	a10, a4
420064cd:	81b9e7               	l32r	a8, 0x420003b4 (0x40001248)
420064d0:	e00800               	callx8	a8
420064d3:	cd0a                	mov.n	a12, a10
420064d5:	bd04                	mov.n	a11, a4
420064d7:	ad07                	mov.n	a10, a7
420064d9:	255742               	call8	0x42048a4c
420064dc:	861200               	j	0x4200652a
420064df:	0000b1               	sra	a0, a0
420064e2:	59e8                	s32i.n	a5, a8, 56
420064e4:	a2c10c               	addi	a10, a1, 12
420064e7:	815ae8               	l32r	a8, 0x42000650 (0x40001230)
420064ea:	e00800               	callx8	a8
420064ed:	ec4a                	bnez.n	a10, 0x42006515
420064ef:	0c1a                	movi.n	a10, 1
420064f1:	65d900               	call8	0x42007288
420064f4:	0c0a                	movi.n	a10, 0
420064f6:	2593fd               	call8	0x42003e28
420064f9:	6573fd               	call8	0x42003c30
420064fc:	b1b3e7               	l32r	a11, 0x420003c8 (0x3c0ca882)
420064ff:	4d0a                	mov.n	a4, a10
42006501:	ad07                	mov.n	a10, a7
42006503:	8181e7               	l32r	a8, 0x42000308 (0x420a7498)
42006506:	e00800               	callx8	a8
42006509:	c183e7               	l32r	a12, 0x42000318 (0x3c0dd73d)
4200650c:	b184e7               	l32r	a11, 0x4200031c (0x3c0c0ba4)
4200650f:	c6ecff               	j	0x420064c6
42006512:	000000               	ill
42006515:	b14de8               	l32r	a11, 0x4200064c (0x3c0cb0e2)
42006518:	a2c10c               	addi	a10, a1, 12
4200651b:	814de8               	l32r	a8, 0x42000650 (0x40001230)
4200651e:	e00800               	callx8	a8
42006521:	16ba00               	beqz	a10, 0x42006530
42006524:	70a720               	or	a10, a7, a7
42006527:	a595fe               	call8	0x42004e80
4200652a:	2d0a                	mov.n	a2, a10
4200652c:	06b1fe               	j	0x42005ff4
4200652f:	0042a0               	addx4	a4, a2, a0
42006532:	013043               	l32r	a0, 0x41fd71f4
42006535:	83d6a3               	excw
42006538:	0065e4               	extui	a6, a0, 5, 15
4200653b:	fc0c                	bnez.n	a12, 0x4200656f
4200653d:	1880                	l32i.n	a1, a0, 32
4200653f:	4a30                	add.n	a3, a0, a4
42006541:	404074               	extui	a4, a4, 0, 8
42006544:	ad04                	mov.n	a10, a4
42006546:	25ddfc               	call8	0x42003318
42006549:	311ae8               	l32r	a3, 0x420005b4 (0x3c0cafa5)
4200654c:	8c14                	beqz.n	a4, 0x42006551
4200654e:	3118e8               	l32r	a3, 0x420005b0 (0x3c0caf97)
42006551:	b19de7               	l32r	a11, 0x420003c8 (0x3c0ca882)
42006554:	ad07                	mov.n	a10, a7
42006556:	816ce7               	l32r	a8, 0x42000308 (0x420a7498)
42006559:	e00800               	callx8	a8
4200655c:	c16fe7               	l32r	a12, 0x42000318 (0x3c0dd73d)
4200655f:	b16fe7               	l32r	a11, 0x4200031c (0x3c0c0ba4)
42006562:	70a720               	or	a10, a7, a7
42006565:	254a42               	call8	0x42048a08
42006568:	ad03                	mov.n	a10, a3
4200656a:	8192e7               	l32r	a8, 0x420003b4 (0x40001248)
4200656d:	e00800               	callx8	a8
42006570:	cd0a                	mov.n	a12, a10
42006572:	bd03                	mov.n	a11, a3
42006574:	c6d7ff               	j	0x420064d7
42006577:	001df0               	subx8	a1, a13, a0
	...
