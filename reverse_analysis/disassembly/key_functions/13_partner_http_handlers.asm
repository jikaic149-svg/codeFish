# Partner HTTP 处理函数
# address range: 0x42004910..0x42004e80
# generator: GNU objdump (crosstool-NG esp-14.2.0_20251107) 2.43.1
# Note: inline Xtensa jump tables can appear as instructions in linear output.


reverse_analysis\segments\segment_02_IROM_0x42000020_0xb701c.bin:     file format binary


Disassembly of section .data:

42004910 <.data+0x48f0>:
42004910:	366102               	entry	a1, 0x130
42004913:	6140ee               	l32r	a6, 0x42000214 (0x3fca08c0)
42004916:	82a10c               	movi	a8, 0x10c
42004919:	8a51                	add.n	a5, a1, a8
4200491b:	c02000               	memw
4200491e:	8806                	l32i.n	a8, a6, 0
42004920:	cb71                	addi.n	a7, a1, 12
42004922:	c02000               	memw
42004925:	8905                	s32i.n	a8, a5, 0
42004927:	b2a100               	movi	a11, 0x100
4200492a:	ad07                	mov.n	a10, a7
4200492c:	e5ba02               	call8	0x420074dc
4200492f:	b173ee               	l32r	a11, 0x420002fc (0x3c0c0b4c)
42004932:	ad02                	mov.n	a10, a2
42004934:	8175ee               	l32r	a8, 0x42000308 (0x420a7498)
42004937:	e00800               	callx8	a8
4200493a:	c171ee               	l32r	a12, 0x42000300 (0x3c0c0b5d)
4200493d:	b171ee               	l32r	a11, 0x42000304 (0x3c0c0b66)
42004940:	ad02                	mov.n	a10, a2
42004942:	650c44               	call8	0x42048a08
42004945:	20a220               	or	a10, a2, a2
42004948:	7cfc                	movi.n	a12, -1
4200494a:	70b720               	or	a11, a7, a7
4200494d:	e50f44               	call8	0x42048a4c
42004950:	c02000               	memw
42004953:	9805                	l32i.n	a9, a5, 0
42004955:	c02000               	memw
42004958:	8806                	l32i.n	a8, a6, 0
4200495a:	2d0a                	mov.n	a2, a10
4200495c:	871905               	beq	a9, a8, 0x42004965
4200495f:	813cee               	l32r	a8, 0x42000250 (0x4037d2e8)
42004962:	e00800               	callx8	a8
42004965:	1df0                	retw.n
42004967:	003641               	srli	a3, a0, 6
4200496a:	00b168               	excw
4200496d:	ee20a2208165eee0 	{ excw; addi	a2, a8, 81; srai	a14, a14, 0 }
42004975:	0800                	l32i.n	a0, a0, 0
42004977:	c162ee               	l32r	a12, 0x42000300 (0x3c0c0b5d)
4200497a:	b162ee               	l32r	a11, 0x42000304 (0x3c0c0b66)
4200497d:	20a220               	or	a10, a2, a2
42004980:	650844               	call8	0x42048a08
42004983:	b163ee               	l32r	a11, 0x42000310 (0x3c0df6ac)
42004986:	7cfc                	movi.n	a12, -1
42004988:	ad02                	mov.n	a10, a2
4200498a:	250c44               	call8	0x42048a4c
4200498d:	2d0a                	mov.n	a2, a10
4200498f:	1df0                	retw.n
42004991:	000000               	ill
42004994:	364100               	entry	a1, 32
42004997:	b15fee               	l32r	a11, 0x42000314 (0x3c0c0b8d)
4200499a:	20a220               	or	a10, a2, a2
4200499d:	815aee               	l32r	a8, 0x42000308 (0x420a7498)
420049a0:	e00800               	callx8	a8
420049a3:	c15dee               	l32r	a12, 0x42000318 (0x3c0dd73d)
420049a6:	b15dee               	l32r	a11, 0x4200031c (0x3c0c0ba4)
420049a9:	20a220               	or	a10, a2, a2
420049ac:	a50544               	call8	0x42048a08
420049af:	b15cee               	l32r	a11, 0x42000320 (0x3c0c0bc0)
420049b2:	c2a17c               	movi	a12, 0x17c
420049b5:	ad02                	mov.n	a10, a2
420049b7:	650944               	call8	0x42048a4c
420049ba:	2d0a                	mov.n	a2, a10
420049bc:	1df0                	retw.n
420049be:	000036               	excw
420049c1:	4100b1               	l32r	a4, 0x41ff0dc4
420049c4:	58ee                	l32i.n	a5, a14, 56
420049c6:	20a220               	or	a10, a2, a2
420049c9:	814fee               	l32r	a8, 0x42000308 (0x420a7498)
420049cc:	e00800               	callx8	a8
420049cf:	c152ee               	l32r	a12, 0x42000318 (0x3c0dd73d)
420049d2:	b152ee               	l32r	a11, 0x4200031c (0x3c0c0ba4)
420049d5:	20a220               	or	a10, a2, a2
420049d8:	e50244               	call8	0x42048a08
420049db:	b153ee               	l32r	a11, 0x42000328 (0x3c0c0d57)
420049de:	c2a262               	movi	a12, 0x262
420049e1:	ad02                	mov.n	a10, a2
420049e3:	a50644               	call8	0x42048a4c
420049e6:	2d0a                	mov.n	a2, a10
420049e8:	1df0                	retw.n
420049ea:	000036               	excw
420049ed:	4100c1               	l32r	a4, 0x41ff4df0
420049f0:	4feeb144eead0225 	{ excw; excw }
420049f8:	0144c1               	l32r	a0, 0x41ff4f08
420049fb:	4dee                	excw
420049fd:	b14dee               	l32r	a11, 0x42000334 (0x3c0c0ff2)
42004a00:	ad02                	mov.n	a10, a2
42004a02:	650044               	call8	0x42048a08
42004a05:	c14cee               	l32r	a12, 0x42000338 (0x3c0d128f)
42004a08:	b14dee               	l32r	a11, 0x4200033c (0x3c0c0ff9)
42004a0b:	ad02                	mov.n	a10, a2
42004a0d:	a5ff43               	call8	0x42048a08
42004a10:	b14cee               	l32r	a11, 0x42000340 (0x3c0c1001)
42004a13:	ad02                	mov.n	a10, a2
42004a15:	813cee               	l32r	a8, 0x42000308 (0x420a7498)
42004a18:	e00800               	callx8	a8
42004a1b:	c13fee               	l32r	a12, 0x42000318 (0x3c0dd73d)
42004a1e:	b13fee               	l32r	a11, 0x4200031c (0x3c0c0ba4)
42004a21:	20a220               	or	a10, a2, a2
42004a24:	25fe43               	call8	0x42048a08
42004a27:	c147ee               	l32r	a12, 0x42000344 (0x967b)
42004a2a:	b147ee               	l32r	a11, 0x42000348 (0x3c0c100b)
42004a2d:	ad02                	mov.n	a10, a2
42004a2f:	e50144               	call8	0x42048a4c
42004a32:	2d0a                	mov.n	a2, a10
42004a34:	1df0                	retw.n
42004a36:	000036               	excw
42004a39:	41000c               	l32r	a4, 0x41fc7a3c
42004a3c:	1aa5                	add.n	a10, a5, a1
42004a3e:	8402b1               	excw
42004a41:	2f                      	.byte	0x2f
42004a42:	eead028130eee008 	{ l32r	a14, 0x41fc54f8; j	0x41ff2d4e; excw }
42004a4a:	00b140               	excw
42004a4d:	ee7cfcad02a5ff43 	{ excw; neg	a2, a10; addx2	a15, a3, a15 }
42004a55:	2d0a                	mov.n	a2, a10
42004a57:	1df0                	retw.n
42004a59:	000000               	ill
42004a5c:	36e101               	entry	a1, 240
42004a5f:	61eded               	l32r	a6, 0x42000214 (0x3fca08c0)
42004a62:	82a0cc               	movi	a8, 204
42004a65:	8a51                	add.n	a5, a1, a8
42004a67:	c02000               	memw
42004a6a:	8806                	l32i.n	a8, a6, 0
42004a6c:	c139ee               	l32r	a12, 0x42000350 (0x3c0ca6ba)
42004a6f:	cb71                	addi.n	a7, a1, 12
42004a71:	c02000               	memw
42004a74:	8905                	s32i.n	a8, a5, 0
42004a76:	40d420               	or	a13, a4, a4
42004a79:	b2a0c0               	movi	a11, 192
42004a7c:	70a720               	or	a10, a7, a7
42004a7f:	8135ee               	l32r	a8, 0x42000354 (0x4209d890)
42004a82:	e00800               	callx8	a8
42004a85:	bd03                	mov.n	a11, a3
42004a87:	20a220               	or	a10, a2, a2
42004a8a:	8133ee               	l32r	a8, 0x42000358 (0x420a7474)
42004a8d:	e00800               	callx8	a8
42004a90:	b11bee               	l32r	a11, 0x420002fc (0x3c0c0b4c)
42004a93:	ad02                	mov.n	a10, a2
42004a95:	811cee               	l32r	a8, 0x42000308 (0x420a7498)
42004a98:	e00800               	callx8	a8
42004a9b:	ad02                	mov.n	a10, a2
42004a9d:	7cfc                	movi.n	a12, -1
42004a9f:	bd07                	mov.n	a11, a7
42004aa1:	a5fa43               	call8	0x42048a4c
42004aa4:	c02000               	memw
42004aa7:	9805                	l32i.n	a9, a5, 0
42004aa9:	c02000               	memw
42004aac:	8806                	l32i.n	a8, a6, 0
42004aae:	2d0a                	mov.n	a2, a10
42004ab0:	871905               	beq	a9, a8, 0x42004ab9
42004ab3:	81e7ed               	l32r	a8, 0x42000250 (0x4037d2e8)
42004ab6:	e00800               	callx8	a8
42004ab9:	1df0                	retw.n
42004abb:	003661               	excw
42004abe:	0261d5               	s32i	a0, a1, 0x354
42004ac1:	ed82                	excw
42004ac3:	a10c8a               	l32r	a10, 0x41fe72f4
42004ac6:	51c020               	l32r	a5, 0x41fccdc8
42004ac9:	008806               	excw
42004acc:	cb71                	addi.n	a7, a1, 12
42004ace:	0c0b                	movi.n	a11, 0
42004ad0:	c02000               	memw
42004ad3:	8905                	s32i.n	a8, a5, 0
42004ad5:	c2a060               	movi	a12, 96
42004ad8:	ad07                	mov.n	a10, a7
42004ada:	8122ee               	l32r	a8, 0x42000364 (0x400011e8)
42004add:	e00800               	callx8	a8
42004ae0:	b2a060               	movi	a11, 96
42004ae3:	ad07                	mov.n	a10, a7
42004ae5:	658d02               	call8	0x420073bc
42004ae8:	dcda                	bnez.n	a10, 0x42004b09
42004aea:	b11cee               	l32r	a11, 0x4200035c (0x3c0ca6d9)
42004aed:	cd07                	mov.n	a12, a7
42004aef:	ad02                	mov.n	a10, a2
42004af1:	a5f6ff               	call8	0x42004a5c
42004af4:	c02000               	memw
42004af7:	9805                	l32i.n	a9, a5, 0
42004af9:	c02000               	memw
42004afc:	8806                	l32i.n	a8, a6, 0
42004afe:	2d0a                	mov.n	a2, a10
42004b00:	871930               	beq	a9, a8, 0x42004b34
42004b03:	81d3ed               	l32r	a8, 0x42000250 (0x4037d2e8)
42004b06:	e00800               	callx8	a8
42004b09:	c115ee               	l32r	a12, 0x42000360 (0x3c0ca6e6)
42004b0c:	42c16c               	addi	a4, a1, 108
42004b0f:	dd07                	mov.n	a13, a7
42004b11:	b2a0a0               	movi	a11, 160
42004b14:	ad04                	mov.n	a10, a4
42004b16:	810fee               	l32r	a8, 0x42000354 (0x4209d890)
42004b19:	e00800               	callx8	a8
42004b1c:	b1f8ed               	l32r	a11, 0x420002fc (0x3c0c0b4c)
42004b1f:	ad02                	mov.n	a10, a2
42004b21:	81f9ed               	l32r	a8, 0x42000308 (0x420a7498)
42004b24:	e00800               	callx8	a8
42004b27:	7cfc                	movi.n	a12, -1
42004b29:	bd04                	mov.n	a11, a4
42004b2b:	ad02                	mov.n	a10, a2
42004b2d:	e5f143               	call8	0x42048a4c
42004b30:	06f0ff               	j	0x42004af4
42004b33:	001df0               	subx8	a1, a13, a0
42004b36:	000036               	excw
42004b39:	4100b1               	l32r	a4, 0x41ff0f3c
42004b3c:	0bee                	addi.n	a14, a14, -1
42004b3e:	20a220               	or	a10, a2, a2
42004b41:	8105ee               	l32r	a8, 0x42000358 (0x420a7474)
42004b44:	e00800               	callx8	a8
42004b47:	c109ee               	l32r	a12, 0x4200036c (0x3c0ca710)
42004b4a:	b109ee               	l32r	a11, 0x42000370 (0x3c0ca716)
42004b4d:	ad02                	mov.n	a10, a2
42004b4f:	a5eb43               	call8	0x42048a08
42004b52:	c1f6ed               	l32r	a12, 0x4200032c (0x3c0c0fba)
42004b55:	b1ebed               	l32r	a11, 0x42000304 (0x3c0c0b66)
42004b58:	ad02                	mov.n	a10, a2
42004b5a:	e5ea43               	call8	0x42048a08
42004b5d:	0c0c                	movi.n	a12, 0
42004b5f:	0c0b                	movi.n	a11, 0
42004b61:	ad02                	mov.n	a10, a2
42004b63:	a5ee43               	call8	0x42048a4c
42004b66:	2d0a                	mov.n	a2, a10
42004b68:	1df0                	retw.n
42004b6a:	000036               	excw
42004b6d:	610182               	l32r	a6, 0x41fe5374
42004b70:	a08c8a               	round.s	a8, f12, 10
42004b73:	6181a8               	l32r	a6, 0x41feed78
42004b76:	ed42                	excw
42004b78:	2283c0               	excw
42004b7b:	200088               	excw
42004b7e:	0892                	l32i.n	a0, a2, 36
42004b80:	a0abc0               	sub	a10, a11, a10
42004b83:	200089               	excw
42004b86:	0692d9               	j	0x41ffb1d2
42004b89:	0d0b                	mov.n	a0, a11
42004b8b:	8487b9               	excw
42004b8e:	23c1f9               	ssiu	f2, a1, 0x3e4
42004b91:	edb1                	excw
42004b93:	f9ed                	s32i.n	a15, a13, 56
42004b95:	ad02                	mov.n	a10, a2
42004b97:	65ecff               	call8	0x42004a5c
42004b9a:	819eed               	l32r	a8, 0x42000214 (0x3fca08c0)
42004b9d:	c02000               	memw
42004ba0:	9806                	l32i.n	a9, a6, 0
42004ba2:	c02000               	memw
42004ba5:	8808                	l32i.n	a8, a8, 0
42004ba7:	879902               	bne	a9, a8, 0x42004bad
42004baa:	862500               	j	0x42004c44
42004bad:	81a8ed               	l32r	a8, 0x42000250 (0x4037d2e8)
42004bb0:	e00800               	callx8	a8
42004bb3:	1ba4                	addi.n	a10, a4, 1
42004bb5:	81f6ed               	l32r	a8, 0x42000390 (0x403844a0)
42004bb8:	e00800               	callx8	a8
42004bbb:	5d0a                	mov.n	a5, a10
42004bbd:	0c03                	movi.n	a3, 0
42004bbf:	7cd7                	movi.n	a7, -3
42004bc1:	cc7a                	bnez.n	a10, 0x42004bcc
42004bc3:	c1eeed               	l32r	a12, 0x4200037c (0x3c0ca752)
42004bc6:	b1eeed               	l32r	a11, 0x42000380 (0x3c0ca760)
42004bc9:	06f2ff               	j	0x42004b95
42004bcc:	30c4c0               	sub	a12, a4, a3
42004bcf:	3ab5                	add.n	a11, a5, a3
42004bd1:	ad02                	mov.n	a10, a2
42004bd3:	a53144               	call8	0x42048eec
42004bd6:	771a16               	beq	a10, a7, 0x42004bf0
42004bd9:	e61a11               	bgei	a10, 1, 0x42004bee
42004bdc:	ad05                	mov.n	a10, a5
42004bde:	81eded               	l32r	a8, 0x42000394 (0x403844c0)
42004be1:	e00800               	callx8	a8
42004be4:	c1e8ed               	l32r	a12, 0x42000384 (0x3c0ca77a)
42004be7:	b1e8ed               	l32r	a11, 0x42000388 (0x3c0ca792)
42004bea:	c6e9ff               	j	0x42004b95
42004bed:	00aa33               	clamps	a10, a10, 7
42004bf0:	4733d8               	bltu	a3, a4, 0x42004bcc
42004bf3:	4a85                	add.n	a8, a5, a4
42004bf5:	0c09                	movi.n	a9, 0
42004bf7:	924800               	s8i	a9, a8, 0
42004bfa:	c2a080               	movi	a12, 128
42004bfd:	0c0b                	movi.n	a11, 0
42004bff:	cba1                	addi.n	a10, a1, 12
42004c01:	81d8ed               	l32r	a8, 0x42000364 (0x400011e8)
42004c04:	e00800               	callx8	a8
42004c07:	cb31                	addi.n	a3, a1, 12
42004c09:	bd04                	mov.n	a11, a4
42004c0b:	d2a080               	movi	a13, 128
42004c0e:	cd03                	mov.n	a12, a3
42004c10:	50a520               	or	a10, a5, a5
42004c13:	a56a02               	call8	0x420072bc
42004c16:	4d0a                	mov.n	a4, a10
42004c18:	ad05                	mov.n	a10, a5
42004c1a:	81deed               	l32r	a8, 0x42000394 (0x403844c0)
42004c1d:	e00800               	callx8	a8
42004c20:	cc54                	bnez.n	a4, 0x42004c29
42004c22:	cd03                	mov.n	a12, a3
42004c24:	c6efff               	j	0x42004be7
42004c27:	0000b1               	sra	a0, a0
42004c2a:	b4ed20               	excw
42004c2d:	a22081               	l32i	a10, a0, 0x204
42004c30:	b6ede0               	bltui	a13, 128, 0x42004c14
42004c33:	0800                	l32i.n	a0, a0, 0
42004c35:	b1d5ed               	l32r	a11, 0x4200038c (0x3c0ca7a2)
42004c38:	c2afff               	movi	a12, -1
42004c3b:	20a220               	or	a10, a2, a2
42004c3e:	e5e043               	call8	0x42048a4c
42004c41:	46d5ff               	j	0x42004b9a
42004c44:	2d0a                	mov.n	a2, a10
42004c46:	1df0                	retw.n
42004c48:	364100               	entry	a1, 32
42004c4b:	b1d3ed               	l32r	a11, 0x42000398 (0x3c0ca7da)
42004c4e:	20a220               	or	a10, a2, a2
42004c51:	81aded               	l32r	a8, 0x42000308 (0x420a7498)
42004c54:	e00800               	callx8	a8
42004c57:	c1aaed               	l32r	a12, 0x42000300 (0x3c0c0b5d)
42004c5a:	b1aaed               	l32r	a11, 0x42000304 (0x3c0c0b66)
42004c5d:	ad02                	mov.n	a10, a2
42004c5f:	a5da43               	call8	0x42048a08
42004c62:	258702               	call8	0x420074d4
42004c65:	bd0a                	mov.n	a11, a10
42004c67:	7cfc                	movi.n	a12, -1
42004c69:	ad02                	mov.n	a10, a2
42004c6b:	25de43               	call8	0x42048a4c
42004c6e:	2d0a                	mov.n	a2, a10
42004c70:	1df0                	retw.n
42004c72:	000036               	excw
42004c75:	610361               	l32r	a6, 0x41fdd084
42004c78:	67ed82               	bbsi	a13, 6, 0x42004bfe
42004c7b:	a17c82               	l32r	a10, 0x41fe566c
42004c7e:	c810                	l32i.n	a12, a0, 4
42004c80:	1a58                	add.n	a5, a8, a1
42004c82:	c02000               	memw
42004c85:	8806                	l32i.n	a8, a6, 0
42004c87:	b14fed               	l32r	a11, 0x420001c4 (0x3fca0110)
42004c8a:	c02000               	memw
42004c8d:	8905                	s32i.n	a8, a5, 0
42004c8f:	a2c154               	addi	a10, a1, 84
42004c92:	101120               	or	a1, a1, a1
42004c95:	65c703               	call8	0x4200890c
42004c98:	b14aed               	l32r	a11, 0x420001c0 (0x3fca0144)
42004c9b:	a2c170               	addi	a10, a1, 112
42004c9e:	101120               	or	a1, a1, a1
42004ca1:	a56003               	call8	0x420082ac
42004ca4:	a148ed               	l32r	a10, 0x420001c4 (0x3fca0110)
42004ca7:	72c190               	addi	a7, a1, -112
42004caa:	101120               	or	a1, a1, a1
42004cad:	65c103               	call8	0x420088c4
42004cb0:	663a54               	bnei	a10, 3, 0x42004d08
42004cb3:	a2a094               	movi	a10, 148
42004cb6:	b143ed               	l32r	a11, 0x420001c4 (0x3fca0110)
42004cb9:	aaa7                	add.n	a10, a7, a10
42004cbb:	101120               	or	a1, a1, a1
42004cbe:	e5c503               	call8	0x4200891c
42004cc1:	31b7ed               	l32r	a3, 0x420003a0 (0x3c0df51c)
42004cc4:	a2a0a4               	movi	a10, 164
42004cc7:	0c8c                	movi.n	a12, 8
42004cc9:	30b320               	or	a11, a3, a3
42004ccc:	aaa7                	add.n	a10, a7, a10
42004cce:	81b7ed               	l32r	a8, 0x420003ac (0x400011f4)
42004cd1:	e00800               	callx8	a8
42004cd4:	82a0ac               	movi	a8, 172
42004cd7:	8a87                	add.n	a8, a7, a8
42004cd9:	0c04                	movi.n	a4, 0
42004cdb:	a2a0b4               	movi	a10, 180
42004cde:	4908                	s32i.n	a4, a8, 0
42004ce0:	4918                	s32i.n	a4, a8, 4
42004ce2:	bd03                	mov.n	a11, a3
42004ce4:	c2a008               	movi	a12, 8
42004ce7:	a0a780               	add	a10, a7, a10
42004cea:	81b0ed               	l32r	a8, 0x420003ac (0x400011f4)
42004ced:	e00800               	callx8	a8
42004cf0:	82a0bc               	movi	a8, 188
42004cf3:	8a87                	add.n	a8, a7, a8
42004cf5:	4908                	s32i.n	a4, a8, 0
42004cf7:	4918                	s32i.n	a4, a8, 4
42004cf9:	0c0b                	movi.n	a11, 0
42004cfb:	a2c154               	addi	a10, a1, 84
42004cfe:	81aced               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004d01:	e00800               	callx8	a8
42004d04:	060400               	j	0x42004d18
42004d07:	00b1a5               	extui	a11, a0, 17, 11
42004d0a:	eda2                	excw
42004d0c:	c12410               	l32r	a12, 0x41fc8d9c
42004d0f:	112025               	l32r	a1, 0x41fce190
42004d12:	d806                	l32i.n	a13, a6, 0
42004d14:	46eaff               	j	0x42004cc1
42004d17:	00820a               	add.s	f8, f2, f0
42004d1a:	001628               	excw
42004d1d:	050c0b               	call0	0x4200fde0
42004d20:	a2c154               	addi	a10, a1, 84
42004d23:	81a3ed               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004d26:	e00800               	callx8	a8
42004d29:	420a00               	l8ui	a4, a10, 0
42004d2c:	b2a001               	movi	a11, 1
42004d2f:	a2c154               	addi	a10, a1, 84
42004d32:	819fed               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004d35:	e00800               	callx8	a8
42004d38:	320a00               	l8ui	a3, a10, 0
42004d3b:	0c2b                	movi.n	a11, 2
42004d3d:	a2c154               	addi	a10, a1, 84
42004d40:	819ced               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004d43:	e00800               	callx8	a8
42004d46:	f20a00               	l8ui	a15, a10, 0
42004d49:	b2a003               	movi	a11, 3
42004d4c:	a2c154               	addi	a10, a1, 84
42004d4f:	f26107               	s32i	a15, a1, 28
42004d52:	8197ed               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004d55:	e00800               	callx8	a8
42004d58:	820a00               	l8ui	a8, a10, 0
42004d5b:	f871                	l32i.n	a15, a1, 28
42004d5d:	c191ed               	l32r	a12, 0x420003a4 (0x3c0ca7fa)
42004d60:	8901                	s32i.n	a8, a1, 0
42004d62:	ed03                	mov.n	a14, a3
42004d64:	dd04                	mov.n	a13, a4
42004d66:	1c0b                	movi.n	a11, 16
42004d68:	a2c134               	addi	a10, a1, 52
42004d6b:	817aed               	l32r	a8, 0x42000354 (0x4209d890)
42004d6e:	e00800               	callx8	a8
42004d71:	0c0b                	movi.n	a11, 0
42004d73:	a2c170               	addi	a10, a1, 112
42004d76:	818eed               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004d79:	e00800               	callx8	a8
42004d7c:	820a00               	l8ui	a8, a10, 0
42004d7f:	162805               	beqz	a8, 0x42004dd5
42004d82:	0c0b                	movi.n	a11, 0
42004d84:	a2c170               	addi	a10, a1, 112
42004d87:	818aed               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004d8a:	e00800               	callx8	a8
42004d8d:	420a00               	l8ui	a4, a10, 0
42004d90:	b2a001               	movi	a11, 1
42004d93:	a2c170               	addi	a10, a1, 112
42004d96:	8186ed               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004d99:	e00800               	callx8	a8
42004d9c:	320a00               	l8ui	a3, a10, 0
42004d9f:	0c2b                	movi.n	a11, 2
42004da1:	a2c170               	addi	a10, a1, 112
42004da4:	8183ed               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004da7:	e00800               	callx8	a8
42004daa:	f20a00               	l8ui	a15, a10, 0
42004dad:	b2a003               	movi	a11, 3
42004db0:	a2c170               	addi	a10, a1, 112
42004db3:	f26107               	s32i	a15, a1, 28
42004db6:	817eed               	l32r	a8, 0x420003b0 (0x420a5ce0)
42004db9:	e00800               	callx8	a8
42004dbc:	820a00               	l8ui	a8, a10, 0
42004dbf:	f871                	l32i.n	a15, a1, 28
42004dc1:	c178ed               	l32r	a12, 0x420003a4 (0x3c0ca7fa)
42004dc4:	8901                	s32i.n	a8, a1, 0
42004dc6:	ed03                	mov.n	a14, a3
42004dc8:	dd04                	mov.n	a13, a4
42004dca:	1c0b                	movi.n	a11, 16
42004dcc:	a2c144               	addi	a10, a1, 68
42004dcf:	8161ed               	l32r	a8, 0x42000354 (0x4209d890)
42004dd2:	e00800               	callx8	a8
42004dd5:	a1fbec               	l32r	a10, 0x420001c4 (0x3fca0110)
42004dd8:	101120               	or	a1, a1, a1
42004ddb:	a5ae03               	call8	0x420088c4
42004dde:	d2cafd               	addi	a13, a10, -3
42004de1:	8207a3               	l8ui	a8, a7, 163
42004de4:	d0fd40               	nsau	a13, a13
42004de7:	d0d541               	srli	a13, a13, 5
42004dea:	f2c124               	addi	a15, a1, 36
42004ded:	77e802               	bbsi	a8, 7, 0x42004df3
42004df0:	f22725               	l32i	a15, a7, 148
42004df3:	82c144               	addi	a8, a1, 68
42004df6:	826100               	s32i	a8, a1, 0
42004df9:	c16bed               	l32r	a12, 0x420003a8 (0x3c0ca806)
42004dfc:	82a08c               	movi	a8, 140
42004dff:	b2a100               	movi	a11, 0x100
42004e02:	e2c134               	addi	a14, a1, 52
42004e05:	80a180               	add	a10, a1, a8
42004e08:	807820               	or	a7, a8, a8
42004e0b:	8152ed               	l32r	a8, 0x42000354 (0x4209d890)
42004e0e:	e00800               	callx8	a8
42004e11:	b13aed               	l32r	a11, 0x420002fc (0x3c0c0b4c)
42004e14:	ad02                	mov.n	a10, a2
42004e16:	813ced               	l32r	a8, 0x42000308 (0x420a7498)
42004e19:	e00800               	callx8	a8
42004e1c:	c13fed               	l32r	a12, 0x42000318 (0x3c0dd73d)
42004e1f:	b13fed               	l32r	a11, 0x4200031c (0x3c0c0ba4)
42004e22:	ad02                	mov.n	a10, a2
42004e24:	101120               	or	a1, a1, a1
42004e27:	25be43               	call8	0x42048a08
42004e2a:	7a71                	add.n	a7, a1, a7
42004e2c:	70a720               	or	a10, a7, a7
42004e2f:	8161ed               	l32r	a8, 0x420003b4 (0x40001248)
42004e32:	e00800               	callx8	a8
42004e35:	cd0a                	mov.n	a12, a10
42004e37:	bd07                	mov.n	a11, a7
42004e39:	ad02                	mov.n	a10, a2
42004e3b:	101120               	or	a1, a1, a1
42004e3e:	e5c043               	call8	0x42048a4c
42004e41:	2d0a                	mov.n	a2, a10
42004e43:	a2c124               	addi	a10, a1, 36
42004e46:	101120               	or	a1, a1, a1
42004e49:	a5aa06               	call8	0x4200b8f4
42004e4c:	c02000               	memw
42004e4f:	9805                	l32i.n	a9, a5, 0
42004e51:	c02000               	memw
42004e54:	8806                	l32i.n	a8, a6, 0
42004e56:	871923               	beq	a9, a8, 0x42004e7d
42004e59:	81fdec               	l32r	a8, 0x42000250 (0x4037d2e8)
42004e5c:	e00800               	callx8	a8
42004e5f:	a2c124               	addi	a10, a1, 36
42004e62:	101120               	or	a1, a1, a1
42004e65:	e5a806               	call8	0x4200b8f4
42004e68:	c02000               	memw
42004e6b:	9805                	l32i.n	a9, a5, 0
42004e6d:	c02000               	memw
42004e70:	8806                	l32i.n	a8, a6, 0
42004e72:	8799e3               	bne	a9, a8, 0x42004e59
42004e75:	ad02                	mov.n	a10, a2
42004e77:	8174ec               	l32r	a8, 0x42000048 (0x420959b0)
42004e7a:	e00800               	callx8	a8
42004e7d:	1df0                	retw.n
	...
