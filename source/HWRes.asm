; *** Resident part: Hardware dependent ***

include	NDISdef.inc
include	dp83815.inc
include	MIIdef.inc
include	misc.inc
include	DrvRes.inc

extern	DosIODelayCnt : far16

public	DrvMajVer, DrvMinVer
DrvMajVer	equ	1
DrvMinVer	equ	7

.386

_REGSTR	segment	use16 dword AT 'RGST'
	org	0
Reg	DP83815_Register <>
_REGSTR	ends

_DATA	segment	public word use16 'DATA'

; --- DMA Descriptor management ---
public	VTxHead, VTxTail, VTxFreeHead, VTxFreeTail
public	TxTail, TxFreeHead, TxFreeTail
VTxHead		dw	0
VTxTail		dw	0
VTxFreeHead	dw	0
VTxFreeTail	dw	0
TxTail		dw	0
TxFreeHead	dw	0
TxFreeTail	dw	0

public	RxHead, RxTail, RxBusyHead, RxBusyTail, RxInProg
RxHead		dw	0
RxTail		dw	0
RxBusyHead	dw	0
RxBusyTail	dw	0
RxInProg	dw	0

; --- System(PCI) Resource ---
public	IOaddr, MEMSel, MEMaddr, IRQlevel
IOaddr		dw	?
MEMSel		dw	?
MEMaddr		dd	?
IRQlevel	db	?

align	2
; --- Physical information ---
PhyInfo		_PhyInfo <>

MediaSpeed	db	0
MediaDuplex	db	0
MediaPause	db	0
MediaLink	db	0

; --- Register Contents ---
regIntStatus	dd	0
regIntMask	dd	0
regReceiveMode	dd	0
regHashTable	dw	32 dup (0)

; --- ReceiveChain Frame Descriptor ---
RxFrameLen	dw	0
RxDesc		RxFrameDesc	<>


; --- Configuration Memory Image Parameters ---
public	cfgSLOT, cfgTXQUEUE, cfgRXQUEUE, cfgMAXFRAMESIZE
public	cfgTxMXDMA, cfgTxFLTH, cfgTxDRTH
public	cfgRxMXDMA, cfgRxDRTH, cfgIHR
cfgSLOT		db	0
cfgTXQUEUE	db	8
cfgRXQUEUE	db	16


cfgTxDRTH	db	1536/32	; n*32byte  [0..3f]
cfgTxFLTH	db	288/32	; n*32byte  [0..3f]
cfgTxMXDMA	db	111b	; 256bytes  [0..7]

cfgRxDRTH	db	(248/8) shl 1	; n*8byte [0..1f]
cfgRxMXDMA	db	111b	; 256bytes  [0..7]

cfgIHR		dw	0	; bit8 mode, n*100us [0..ff]
cfgMAXFRAMESIZE		dw	1514


; --- Receive Buffer address ---
public	RxBufferLin, RxBufferPhys, RxBufferSize, RxBufferSelCnt, RxBufferSel
RxBufferLin	dd	?
RxBufferPhys	dd	?
RxBufferSize	dd	?
RxBufferSelCnt	dw	?
RxBufferSel	dw	2 dup (?)	; max is 2.


; --- Vendor Adapter Description ---
public	AdapterDesc
AdapterDesc	db	'National Semiconductor DP83815 Fast Ethernet Adapter',0


_DATA	ends

_TEXT	segment	public word use16 'CODE'
	assume	ds:_DATA, gs:_REGSTR
	
; USHORT hwTxChain(TxFrameDesc *txd, USHORT rqh, USHORT pid)
_hwTxChain	proc	near
	push	bp
	xor	ax,ax
	mov	bp,sp
	les	bx,[bp+4]
	cmp	ax,es:[bx].TxFrameDesc.TxImmedLen
	adc	ax,es:[bx].TxFrameDesc.TxDataCount  ; desc count required

	push	offset semTx
	call	_EnterCrit
	mov	bx,[VTxFreeHead]
	or	bx,bx
	jz	short loc_2
	mov	cx,[bx].txdesc.vlink
	mov	si,[TxFreeHead]
	mov	[VTxFreeHead],cx
	mov	[bx].txdesc.deschead,si
loc_1:
	mov	di,si
	dec	ax
	mov	si,[si].bufdesc.vlink
	jnz	short loc_1

	mov	[bx].txdesc.desctail,di
	mov	[TxFreeHead],si
loc_2:
	call	_LeaveCrit

	or	bx,bx
	jnz	short loc_3
	mov	ax,OUT_OF_RESOURCE
	pop	bx	; stack adjust
	pop	bp
	retn

loc_3:
	push	gs
	mov	ax,[bp+8]
	mov	dx,[bp+10]
	mov	[bx].txdesc.reqhandle,ax
	mov	[bx].txdesc.protid,dx
	lgs	bp,[bp+4]
	mov	cx,gs:[bp].TxFrameDesc.TxImmedLen
	mov	si,[bx].txdesc.deschead
	or	cx,cx
	jz	short loc_4	; No Immediate Data

	push	si
	push	fs
	push	ds
	pop	es
	lfs	si,gs:[bp].TxFrameDesc.TxImmedPtr
	lea	di,[bx].txdesc.immed
	mov	dx,cx
	shr	cx,2
	rep	movsd	es:[di],fs:[si]
	mov	cl,dl
	and	cl,3
	rep	movsb	es:[di],fs:[si]
	pop	fs
	pop	si
	mov	eax,[bx].txdesc.immedphy
	mov	[si].bufdesc.bufptr,eax
	mov	word ptr [si].bufdesc.cmdsts,dx
	mov	word ptr [si].bufdesc.cmdsts[2],highword(OWN or MORE)
	mov	si,[si].bufdesc.vlink
loc_4:
	mov	cx,gs:[bp].TxFrameDesc.TxDataCount
	or	cx,cx
	jz	short loc_7
	lea	bp,[bp].TxFrameDesc.TxBufDesc1
loc_5:
	cmp	gs:[bp].TxBufDesc.TxPtrType,0
	mov	eax,gs:[bp].TxBufDesc.TxDataPtr
	jz	short loc_6
	push	eax
	call	_VirtToPhys
	add	sp,4
loc_6:
	mov	dx,gs:[bp].TxBufDesc.TxDataLen
	mov	[si].bufdesc.bufptr,eax
	mov	word ptr [si].bufdesc.cmdsts,dx
	mov	word ptr [si].bufdesc.cmdsts[2],highword(OWN or MORE)
	add	bp,sizeof(TxBufDesc)
	mov	si,[si].bufdesc.vlink
	dec	cx
	jnz	short loc_5

loc_7:
	xor	eax,eax
	mov	si,[bx].txdesc.deschead
	mov	di,[bx].txdesc.desctail
	mov	[di].bufdesc.link,eax	; buf link tail
	mov	word ptr [di].bufdesc.cmdsts[2],highword(OWN) ; last
	mov	[di].bufdesc.vlink,ax	; buf vlink tail
	mov	[bx].txdesc.vlink,ax	; vtx vlink tail
	mov	cx,bx
	mov	eax,[si].bufdesc.phyaddr ; buf link chain

	pop	gs
	call	_EnterCrit
	cmp	[VTxHead],0
	jnz	short loc_9
	mov	bx,[TxTail]
	mov	[VTxHead],cx		; queue empty
	or	bx,bx
	jz	short loc_8		; first tx
	mov	[bx].bufdesc.link,eax	; link to previous tail.
	jmp	short loc_10
loc_8:
	mov	gs:[Reg.TXDP],eax	; set txdp
	jmp	short loc_10
loc_9:
	mov	bx,[VTxTail]		; queue not empty
	mov	[bx].txdesc.vlink,cx	; vtx chain
	mov	bx,[bx].txdesc.desctail
	mov	[bx].bufdesc.link,eax	; link chain
	mov	[bx].bufdesc.vlink,si	; buf chain

loc_10:
	mov	[VTxTail],cx
	mov	[TxTail],di
	mov	gs:[Reg.CR],TXE
	call	_LeaveCrit
	pop	cx	; stack adjust

	mov	ax,REQUEST_QUEUED
	pop	bp
	retn
_hwTxChain	endp


_hwRxRelease	proc	near
	push	bp
	mov	bp,sp
	push	si
	push	di
	push	offset semRx
	call	_EnterCrit
	mov	bx,[RxInProg]
	mov	ax,[bp+4]
	or	bx,bx		; exist frame in progress?
	jz	short loc_0
	cmp	ax,[bx].bufdesc.deschandle
	jnz	short loc_0
	mov	[RxInProg],0
	jmp	short loc_4

loc_0:
	mov	bx,[RxBusyHead]
loc_1:
	or	bx,bx		; waiting queue empty/tail?
	jz	short loc_5
	mov	si,[bx].bufdesc.desctail
	cmp	ax,[bx].bufdesc.deschandle
	jz	short loc_2
	mov	di,bx
	mov	bx,[si].bufdesc.vlink
	jmp	short loc_1
loc_2:
	cmp	bx,[RxBusyHead]
	mov	ax,[si].bufdesc.vlink
	jnz	short loc_3		; midle/tail
	mov	[RxBusyHead],ax		; head
	jmp	short loc_4
loc_3:
	cmp	si,[RxBusyTail]
	mov	[di].bufdesc.vlink,ax	; middle/tail
	jnz	short loc_4		; middle
	mov	[RxBusyTail],di		; tail
loc_4:
	call	__RxFreeFrame
loc_5:
	call	_LeaveCrit
	pop	bp	; stack adjust
	mov	ax,SUCCESS
	pop	di
	pop	si
	pop	bp
	retn
_hwRxRelease	endp


__RxFreeFrame	proc	near
	mov	di,[bx].bufdesc.desctail
	mov	si,bx
loc_1:
	cmp	bx,di
	mov	word ptr [bx].bufdesc.cmdsts,1536
	jz	short loc_2
	mov	word ptr [bx].bufdesc.cmdsts[2],highword(MORE or INCCRC)
	mov	bx,[bx].bufdesc.vlink
	jmp	short loc_1
loc_2:
	mov	bx,[RxTail]
	mov	word ptr [di].bufdesc.cmdsts[2],highword(OWN or MORE or INCCRC)
	mov	eax,[si].bufdesc.phyaddr
	mov	[bx].bufdesc.link,eax
	mov	word ptr [bx].bufdesc.cmdsts[2],highword(MORE or INCCRC)
	mov	[di].bufdesc.vlink,0
	mov	[bx].bufdesc.vlink,si
	mov	[RxTail],di
	mov	gs:[Reg.CR],RXE
	retn
__RxFreeFrame	endp


_ServiceIntTx	proc	near
	enter	2,0
it_txd	equ	bp-2

	push	offset semTx
loc_0:
	call	_EnterCrit
	xor	cx,cx
	mov	bx,[VTxHead]
	mov	[it_txd],cx
	or	bx,bx
	jz	short loc_1		; empty
	mov	si,[bx].txdesc.desctail
	mov	ax,word ptr [si].bufdesc.cmdsts[2]
	test	ah,high(highword(OWN))
	jnz	short loc_1		; in progress
	mov	dx,[bx].txdesc.vlink
	mov	[it_txd],bx
	mov	[VTxHead],dx
loc_1:
	call	_LeaveCrit
	cmp	[it_txd],cx
	jnz	short loc_2
	leave
	retn

loc_2:
	test	ax,highword(EC or OWC or ED or TD or CRS or TFU or TXA)
	mov	cx,[bx].txdesc.reqhandle
	setnz	al
	mov	dx,[bx].txdesc.protid
	or	cx,cx			; handle = 0 ?
	mov	di,[ProtDS]
	jz	short loc_3
	neg	al
	mov	si,[CommonChar.moduleID]
	and	ax,GENERAL_FAILURE	; SUCCESS / GENERAL_FAILURE

	push	dx	; ProtID
	push	si	; MACID
	push	cx	; ReqHandle
	push	ax	; Status
	push	di	; ProtDS
	call	dword ptr [LowDisp.txconfirm]
	mov	gs,[MEMSel]	; fix gs selector

loc_3:
	xor	eax,eax
	mov	bx,[it_txd]
	mov	si,[bx].txdesc.deschead
	mov	di,[bx].txdesc.desctail
	mov	[bx].txdesc.vlink,ax	; vtx vlink tail
	mov	[di].bufdesc.vlink,ax	; buf vlink tail
;	mov	[di].bufdesc.link,eax	; buf link tail
	mov	ecx,[si].bufdesc.phyaddr

	call	_EnterCrit
	cmp	ax,[TxFreeHead]
	mov	dx,di
	jz	short loc_4
	mov	di,[TxFreeTail]		; buf not empty
	mov	[di].bufdesc.vlink,si	; vlink
	mov	[di].bufdesc.link,ecx	; link
	jmp	short loc_5
loc_4:
	mov	[TxFreeHead],si		; buf empty
loc_5:
	mov	[TxFreeTail],dx
	cmp	ax,[VTxFreeHead]
	jz	short loc_6
	mov	di,[VTxFreeTail]	; vtx not empty
	mov	[di].txdesc.vlink,bx
	jmp	short loc_7
loc_6:
	mov	[VTxFreeHead],bx	; vtx empty
loc_7:
	mov	[VTxFreeTail],bx
	call	_LeaveCrit
	jmp	near ptr loc_0
_ServiceIntTx	endp


_ServiceIntRx	proc	near
	push	bp
	push	offset semRx
loc_0:
	call	_EnterCrit
	mov	bx,[RxHead]
	mov	dx,[RxTail]
	mov	di,[RxInProg]
	call	_LeaveCrit
	or	di,di
;	jnz	short loc_5
	jnz	near ptr loc_5
	cmp	bx,dx
	jz	short loc_ex
	xor	cx,cx
	mov	si,offset RxDesc.RxBufDesc1
	sub	bp,bp
loc_1:
	mov	ax,word ptr [bx].bufdesc.cmdsts[2]
	inc	cx
	test	ah,high(highword(OWN))
	jz	short loc_ex
	test	ah,high(highword(MORE))
	jz	short loc_2
	cmp	cl,8
	ja	short loc_rmv
	mov	di,1536
	mov	eax,[bx].bufdesc.virtaddr
	mov	[si].RxBufDesc.RxDataPtr,eax
	mov	[si].RxBufDesc.RxDataLen,di
	add	bp,di
	mov	bx,[bx].bufdesc.vlink
	add	si,sizeof(RxBufDesc)
	cmp	bx,dx
	jnz	short loc_1
loc_ex:
	pop	ax	; stack adjust
	pop	bp
	retn

loc_rmv:
	call	_EnterCrit
	mov	di,[RxHead]
	mov	ax,[bx].bufdesc.vlink
	mov	[RxHead],ax
	mov	[di].bufdesc.desctail,bx
	mov	bx,di
	call	__RxFreeFrame
	call	_LeaveCrit
	jmp	short loc_0

loc_2:
	test	ax,highword(OK)
	jz	short loc_rmv
	mov	ax,word ptr [bx].bufdesc.cmdsts
	sub	ax,4		; substract CRC length
	cmp	ax,[cfgMAXFRAMESIZE]
	ja	short loc_rmv
	mov	di,ax
	sub	ax,bp		; framgent length
	ja	short loc_3
	dec	cx
	jbe	short loc_rmv
	add	[si-sizeof(RxBufDesc)].RxBufDesc.RxDataLen,ax
	jmp	short loc_4
loc_3:
	cmp	cl,8
	ja	short loc_rmv
	mov	[si].RxBufDesc.RxDataLen,ax
	mov	eax,[bx].bufdesc.virtaddr
	mov	[si].RxBufDesc.RxDataPtr,eax
loc_4:
	mov	[RxDesc.RxDataCount],cx
	mov	[RxFrameLen],di
	mov	ax,[bx].bufdesc.vlink
	call	_EnterCrit
	mov	di,[RxHead]
	mov	[RxHead],ax
	mov	[RxInProg],di
	mov	[di].bufdesc.desctail,bx
	call	_LeaveCrit
loc_5:
	call	_IndicationChkOFF
	or	ax,ax
	jz	short loc_ex	; indicate off - suspend

	push	-1		; indicate
	mov	ax,[ProtDS]
	mov	cx,[di].bufdesc.deschandle
	mov	dx,[CommonChar.moduleID]
	mov	bx,sp
	mov	si,[RxFrameLen]
	push	cx		; handle

	push	dx		; MACID
	push	si		; FrameSize
	push	cx		; ReqHandle
	push	ds
	push	offset RxDesc	; RxFrameDesc
	push	ss
	push	bx		; Indicate
	push	ax
	cld
	call	dword ptr [LowDisp.rxchain]
	mov	gs,[MEMSel]	; fix gs selector
lock	or	[drvflags],mask df_idcp
	cmp	ax,WAIT_FOR_RELEASE
	jz	short loc_6
	call	_hwRxRelease
	jmp	short loc_10

loc_6:
	xor	ax,ax
	push	offset semRx
	call	_EnterCrit
	mov	bx,[RxInProg]
	mov	[RxInProg],ax
	or	bx,bx
	jz	short loc_9
	mov	di,[bx].bufdesc.desctail
	cmp	ax,[RxBusyHead]
	jnz	short loc_7
	mov	[RxBusyHead],bx
	jmp	short loc_8
loc_7:
	mov	si,[RxBusyTail]
	mov	[si].bufdesc.vlink,bx
loc_8:
	mov	[RxBusyTail],di
	mov	[di].bufdesc.vlink,ax
loc_9:
	call	_LeaveCrit
	pop	dx	; stack adjust

loc_10:
	pop	cx	; handle
	pop	ax	; indicate
	cmp	al,-1
	jnz	short loc_11
	call	_IndicationON
	jmp	near ptr loc_0
loc_11:
lock	or	[drvflags],mask df_rxsp
	pop	ax	; stack adjust
	pop	bp
	retn
_ServiceIntRx	endp


_hwServiceInt	proc	near
	enter	4,0
loc_0:
	mov	eax,gs:[Reg.ISR]
lock	or	[regIntStatus],eax
	mov	eax,[regIntStatus]
	and	eax,[regIntMask]
	jnz	short loc_1
	leave
	retn

loc_1:
	mov	[bp-4],eax

	mov	eax,TXOK or TXERR or TXURN or SWI
	test	[bp-4],eax
	jz	short loc_n1
	not	eax
lock	and	[regIntStatus],eax
	call	_ServiceIntTx

loc_n1:
	mov	eax,RXOK or RXERR or RXORN or RXSOVR or SWI
	test	[bp-4],eax
	jz	short loc_n2
	not	eax
lock	and	[regIntStatus],eax
	call	_ServiceIntRx
	test	dword ptr [bp-4],RXSOVR
	jz	short loc_n2
	call	_ResetRx

loc_n2:
	mov	eax,MIB
	test	[bp-4],eax
	jz	short loc_n3
	not	eax
lock	and	[regIntStatus],eax
	call	_hwUpdateStat
loc_n3:
	jmp	short loc_0
_hwServiceInt	endp

_hwCheckInt	proc	near
	mov	eax,gs:[Reg.ISR]
lock	or	[regIntStatus],eax
	mov	eax,[regIntStatus]
	test	eax,[regIntMask]
	setnz	al
	mov	ah,0
	retn
_hwCheckInt	endp

_hwEnableInt	proc	near
	mov	eax,[regIntMask]
	mov	gs:[Reg.IMR],eax	; set IMR
	mov	gs:[Reg.IER],IE		; int enable
	retn
_hwEnableInt	endp

_hwDisableInt	proc	near
	xor	eax,eax
	mov	gs:[Reg.IER],eax	; clear IER
	mov	gs:[Reg.IMR],eax	; clear IMR
	retn
_hwDisableInt	endp

_hwIntReq	proc	near
	mov	gs:[Reg.CR],SWIR
	retn
_hwIntReq	endp

_hwEnableRxInd	proc	near
	push	eax
lock	or	[regIntMask],RXOK or RXERR or RXORN or RXSOVR
	cmp	semInt,0
	jnz	short loc_1
	mov	eax,[regIntMask]
	mov	gs:[Reg.IMR],eax
loc_1:
	pop	eax
	retn
_hwEnableRxInd	endp

_hwDisableRxInd	proc	near
	push	eax
lock	and	[regIntMask],not(RXOK or RXERR or RXORN or RXSOVR)
	cmp	[semInt],0
	jnz	short loc_1
	mov	eax,[regIntMask]
	mov	gs:[Reg.IMR],eax
loc_1:
	pop	eax
	retn
_hwDisableRxInd	endp

_hwPollLink	proc	near
	call	_ChkLink
	test	al,MediaLink
	jz	short loc_0	; Link status change/down
	retn
loc_0:
	or	al,al
	mov	MediaLink,al
	jnz	short loc_1	; Link Active
	call	_ChkLink	; check again (current status)
	or	al,al
	mov	MediaLink,al
	jnz	short loc_1	; short time down - 
	retn
loc_1:
	cli
	mov	al,1
	xchg	al,[semInt]	; get interrupt semaphore
	or	al,al
	jz	short loc_2
	call	_Delay1ms
	jmp	short loc_1
loc_2:
	call	_hwDisableInt
	sti
	call	_GetPhyMode
	cmp	al,MediaSpeed
	jnz	short loc_3
	cmp	ah,MediaDuplex
	jnz	short loc_3
	cmp	dl,MediaPause
	jz	short loc_4
loc_3:
	mov	MediaSpeed,al
	mov	MediaDuplex,ah
	mov	MediaPause,dl

	call	_SetMacEnv
loc_4:
	cli
	mov	eax,[regIntMask]
	test	eax,[regIntStatus]	; pending interrupt?
	jz	short loc_5
	call	_hwIntReq	; software interrupt
loc_5:
	call	_hwEnableInt
	mov	al,0
	xchg	al,[semInt]	; release interrupt semaphore
	sti
	retn
_hwPollLink	endp

_hwOpen		proc	near	; call in protocol bind process?
	call	_ResetPhy
	cmp	ax,SUCCESS
	jnz	short loc_e
	call	_AutoNegotiate
	mov	MediaSpeed,al
	mov	MediaDuplex,ah
	mov	MediaPause,dl

	call	_ChkLink
	mov	MediaLink,al
	call	_SetMacEnv

	mov	bx,[RxHead]
	mov	eax,[bx].bufdesc.phyaddr
	mov	gs:[Reg.RXDP],eax	; set rx head pointer

	mov	eax,gs:[Reg.SRR]
	and	ah,0fh
	cmp	ah,05
	jc	short loc_1		; DP83815
	xor	eax,eax
	mov	ax,[cfgIHR]
	mov	gs:[Reg.IHR],eax	; DP83816 only
loc_1:
	xor	eax,eax
	mov	[regIntStatus],eax
	mov	eax,gs:[Reg.ISR]	; clear interrupt status
	mov	eax,RXOK or RXERR or RXORN or \
		  TXOK or TXERR or TXURN or \
		  MIB or SWI or RXSOVR
	mov	[regIntMask],eax
	mov	gs:[Reg.IMR],eax	; set interrupt mask
	mov	gs:[Reg.CR],RXE		; enable rx
	mov	gs:[Reg.IER],IE		; enable interrupt
	mov	ax,SUCCESS
loc_e:
	retn
_hwOpen		endp

_SetMacEnv	proc	near
	mov	al,cfgTxMXDMA
	mov	ah,high(highword(ATP)) shr 4
	or	al,low(highword(ECRETRY)) shr 4
	shl	eax,20
	mov	al,cfgTxDRTH
	mov	ah,cfgTxFLTH

	mov	cl,cfgRxMXDMA
	mov	ch,0
	shl	ecx,20
	mov	cl,cfgRxDRTH

	cmp	MediaSpeed,1	; 100
	jna	short loc_1
	or	eax,HBI		; heartbeat ignore
loc_1:
	xor	bx,bx
	cmp	MediaDuplex,1
	jc	short loc_2
	or	eax,CSI or HBI	; carrier sense/ heartbeat ignore
	or	ecx,ATX		; accept transmit packet

	test	MediaPause,2
	jz	short loc_2
	mov	bx,highword(PSEN or PS_MCAST)
loc_2:
	shl	ebx,16
	mov	gs:[Reg.TXCFG],eax
	mov	gs:[Reg.RXCFG],ecx
	mov	gs:[Reg.PCR],ebx

	call	_SetSpeedStat
	retn
_SetMacEnv	endp

_ResetRx	proc	near
	push	offset semRx
	call	_EnterCrit

	xor	eax,eax
lock	and	[regIntStatus],not RXRCMP
	mov	gs:[Reg.CR],RXR		; rx reset
loc_1:
	test	gs:[Reg.CR],RXE		; wait rx disabled
	jnz	short loc_1

	mov	gs:[Reg.RXDP],eax	; clear queue head

	mov	si,[RxHead]
	mov	di,[RxTail]
	mov	bx,si
loc_2:
	cmp	bx,di
	mov	word ptr [bx].bufdesc.cmdsts,1536
	mov	word ptr [bx].bufdesc.cmdsts[2],highword(MORE or INCCRC)
	mov	bx,[bx].bufdesc.vlink
	jnz	short loc_2
	mov	word ptr [di].bufdesc.cmdsts[2],highword(OWN or MORE or INCCRC)
	mov	[di].bufdesc.link,eax
loc_3:
	mov	eax,gs:[Reg.ISR]
lock	or	[regIntStatus],eax
	test	[regIntStatus],RXRCMP	; wait for rx reset completion
	jz	short loc_3

	mov	eax,[si].bufdesc.phyaddr
	mov	gs:[Reg.RXDP],eax	; set rx queue head
	mov	gs:[Reg.CR],RXE		; rx enabled

	call	_LeaveCrit
	pop	ax
	retn
_ResetRx	endp


_ChkLink	proc	near
	push	miiBMSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	and	ax,miiBMSR_LinkStat
	add	sp,2*2
	shr	ax,2
	retn
_ChkLink	endp


_AutoNegotiate	proc	near
	enter	2,0
	push	0
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite		; clear ANEnable bit
	add	sp,3*2

	call	_Delay1ms
	push	miiBMCR_ANEnable or miiBMCR_RestartAN
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite		; restart Auto-Negotiation
	add	sp,3*2

	mov	word ptr [bp-2],12*30	; about 12sec.
loc_1:
	call	_Delay1ms
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	test	ax,miiBMCR_RestartAN	; AN in progress?
	jz	short loc_2
	dec	word ptr [bp-2]
	jnz	short loc_1
	jmp	short loc_f
loc_2:
	call	_Delay1ms
	push	miiBMSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	test	ax,miiBMSR_ANComp	; AN Base Page exchange complete?
	jnz	short loc_3
	dec	word ptr [bp-2]
	jnz	short loc_2
	jmp	short loc_f
loc_3:
	call	_Delay1ms
	push	miiBMSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	test	ax,miiBMSR_LinkStat	; link establish?
	jnz	short loc_4
	dec	word ptr [bp-2]
	jnz	short loc_3
loc_f:
	xor	ax,ax			; AN failure.
	xor	dx,dx
	leave
	retn
loc_4:
	call	_GetPhyMode
	leave
	retn
_AutoNegotiate	endp

_GetPhyMode	proc	near
	push	miiANLPAR
	push	[PhyInfo.Phyaddr]
	call	_miiRead		; read base page
	add	sp,2*2
	mov	[PhyInfo.ANLPAR],ax

	test	[PhyInfo.BMSR],miiBMSR_ExtStat
	jz	short loc_2

	push	mii1KSTSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.GSTSR],ax
	shl	ax,2
	and	ax,[PhyInfo.GSCR]
	test	ax,mii1KSCR_1KTFD
	jz	short loc_1
	mov	al,3			; media speed - 1000Mb
	mov	ah,1			; media duplex - full
	jmp	short loc_p
loc_1:
	test	ax,mii1KSCR_1KTHD
	jz	short loc_2
	mov	al,3			; 1000Mb
	mov	ah,0			; half duplex
	jmp	short loc_p
loc_2:
	mov	ax,[PhyInfo.ANAR]
	and	ax,[PhyInfo.ANLPAR]
	test	ax,miiAN_100FD
	jz	short loc_3
	mov	al,2			; 100Mb
	mov	ah,1			; full duplex
	jmp	short loc_p
loc_3:
	test	ax,miiAN_100HD
	jz	short loc_4
	mov	al,2			; 100Mb
	mov	ah,0			; half duplex
	jmp	short loc_p
loc_4:
	test	ax,miiAN_10FD
	jz	short loc_5
	mov	al,1			; 10Mb
	mov	ah,1			; full duplex
	jmp	short loc_p
loc_5:
	test	ax,miiAN_10HD
	jz	short loc_e
	mov	al,1			; 10Mb
	mov	ah,0			; half duplex
	jmp	short loc_p
loc_e:
	xor	ax,ax
	sub	dx,dx
	retn
loc_p:
	cmp	ah,1			; full duplex?
	jnz	short loc_np
	mov	cx,[PhyInfo.ANLPAR]
	test	cx,miiAN_PAUSE		; symmetry
	mov	dl,3			; tx/rx pause
	jnz	short loc_ex
	test	cx,miiAN_ASYPAUSE	; asymmetry
	mov	dl,2			; rx pause
	jnz	short loc_ex
loc_np:
	mov	dl,0			; no pause
loc_ex:
	retn
_GetPhyMode	endp


_ResetPhy	proc	near
	enter	2,0
	call	_miiReset	; Reset Interface
	push	miiPHYID2
	push	1fh		; phyaddr 1fh
	call	_miiRead
	add	sp,2*2
	or	ax,ax		; ID2 = 0
	jz	short loc_1
	inc	ax		; ID2 = -1
	jnz	short loc_2
loc_1:
	mov	ax,HARDWARE_FAILURE
	leave
	retn
loc_2:
	mov	[PhyInfo.Phyaddr],1fh
	push	miiBMCR_Reset
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite	; Reset PHY
	add	sp,3*2
	mov	word ptr [bp-2],64  ; reset wait about 2sec.
loc_21:
	call	_Delay1ms
	dec	word ptr [bp-2]
	jnz	short loc_21

	call	_miiReset	; interface reset again
	mov	word ptr [bp-2],64  ; about 2sec.
loc_3:
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	test	ax,miiBMCR_Reset
	jz	short loc_4
	call	_Delay1ms	; wait reset complete. 33ms:-)
	dec	word ptr [bp-2]
	jnz	short loc_3
	jmp	short loc_1	; PHY Reset Failure
loc_4:
	call	_InitPHY_RR	; performance recommended?

	push	miiBMSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.BMSR],ax
	push	miiANAR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.ANAR],ax
	test	[PhyInfo.BMSR],miiBMSR_ExtStat
	jz	short loc_5	; extended status exist?
	push	mii1KTCR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.GTCR],ax
	push	mii1KSCR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.GSCR],ax
	xor	cx,cx
	test	ax,mii1KSCR_1KTFD
	jz	short loc_41
	or	cx,mii1KTCR_1KTFD
loc_41:
	test	ax,mii1KSCR_1KTHD
	jz	short loc_42
	or	cx,mii1KTCR_1KTHD
loc_42:
	mov	ax,[PhyInfo.GTCR]
	and	ax,not (mii1KTCR_MSE or mii1KTCR_Port or \
		  mii1KTCR_1KTFD or mii1KTCR_1KTHD)
	or	ax,cx
	mov	[PhyInfo.GTCR],ax
	push	ax
	push	mii1KTCR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite
	add	sp,2*2
loc_5:
	mov	ax,[PhyInfo.BMSR]
	mov	cx,miiAN_PAUSE
	test	ax,miiBMSR_100FD
	jz	short loc_61
	or	cx,miiAN_100FD
loc_61:
	test	ax,miiBMSR_100HD
	jz	short loc_62
	or	cx,miiAN_100HD
loc_62:
	test	ax,miiBMSR_10FD
	jz	short loc_63
	or	cx,miiAN_10FD
loc_63:
	test	ax,miiBMSR_10HD
	jz	short loc_64
	or	cx,miiAN_10HD
loc_64:
	mov	ax,[PhyInfo.ANAR]
	and	ax,not (miiAN_ASYPAUSE + miiAN_T4 + \
	  miiAN_100FD + miiAN_100HD + miiAN_10FD + miiAN_10HD)
	or	ax,cx
	mov	[PhyInfo.ANAR],ax
	push	ax
	push	miiANAR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite
	add	sp,3*2
	mov	ax,SUCCESS
	leave
	retn
_ResetPhy	endp

_InitPHY_RR	proc	near
	mov	eax,gs:[Reg.SRR]
	and	ah,0fh
	cmp	ah,3
	ja	short loc_45
	jb	short loc_2
loc_3:
	mov	gs:[Reg.PGSEL],1
	mov	gs:[Reg.reg_e4],189ch
	mov	gs:[Reg.reg_fc],0
	mov	gs:[Reg.reg_f4],5040h
	mov	gs:[Reg.reg_f8],8ch
	mov	gs:[Reg.PGSEL],0
loc_ex:
	retn
loc_45:
	cmp	ah,5
	ja	short loc_ex
	mov	gs:[Reg.PGSEL],1
	mov	gs:[Reg.reg_e4],189ch
	mov	gs:[Reg.PGSEL],0
	retn
loc_2:
	cmp	ah,2
	jnz	short loc_ex
	mov	gs:[Reg.PGSEL],1
	mov	gs:[Reg.reg_f4],802h
	mov	gs:[Reg.reg_d0],10
	mov	gs:[Reg.reg_f8],333h
	mov	gs:[Reg.reg_e8],860h
	mov	gs:[Reg.reg_d4],2100h
	mov	gs:[Reg.reg_e0],4f48h
	mov	gs:[Reg.PGSEL],0
	mov	gs:[Reg.reg_e8],4
	retn
_InitPHY_RR	endp


_hwUpdateMulticast	proc	near
	enter	2,0
	push	si
	push	di
	push	offset semFlt
	call	_EnterCrit
	mov	cx,64/4
	mov	di,offset regHashTable
	push	ds
	pop	es
	xor	eax,eax
	rep	stosd		; clear hash table

	mov	cx,MCSTList.curnum
	dec	cx
	jl	short loc_2
	mov	[bp-2],cx
loc_1:
	mov	ax,[bp-2]
	shl	ax,4		; 16bytes
	add	ax,offset MCSTList.multicastaddr1
	push	ax
	call	_CRC32
	pop	cx
	shr	eax,23		; the 9 most significant bits
	mov	di,ax
	and	ax,0fh		; the bit index in word
	shr	di,4
	add	di,di		; the word index (2byte)
	bts	word ptr regHashTable[di],ax
	dec	word ptr [bp-2]
	jge	short loc_1
loc_2:
	mov	si,offset regHashTable -200h
	mov	cx,64/2
	mov	ebx,200h	; hash table index
	xor	eax,eax
	mov	edx,gs:[Reg.RFCR]
loc_3:
	mov	ax,[bx+si]
	mov	gs:[Reg.RFCR],ebx
	mov	gs:[Reg.RFDR],eax
	add	bx,2
	dec	cx
	jnz	short loc_3
	mov	gs:[Reg.RFCR],edx

	call	_LeaveCrit
	pop	cx
	mov	ax,SUCCESS
	pop	di
	pop	si
	leave
	retn
_hwUpdateMulticast	endp

_CRC32		proc	near
POLYNOMIAL_be   equ  04C11DB7h
POLYNOMIAL_le   equ 0EDB88320h

	push	bp
	mov	bp,sp

	push	si
	push	di
	or	ax,-1
	mov	bx,[bp+4]
	mov	ch,3
	cwd

loc_1:
	mov	bp,[bx]
	mov	cl,10h
	inc	bx
loc_2:
IF 1
		; big endian

	ror	bp,1
	mov	si,dx
	xor	si,bp
	shl	ax,1
	rcl	dx,1
	sar	si,15
	mov	di,si
	and	si,highword POLYNOMIAL_be
	and	di,lowword POLYNOMIAL_be
ELSE
		; litte endian
	mov	si,ax
	ror	bp,1
	ror	si,1
	shr	dx,1
	rcr	ax,1
	xor	si,bp
	sar	si,15
	mov	di,si
	and	si,highword POLYNOMIAL_le
	and	di,lowword POLYNOMIAL_le
ENDIF
	xor	dx,si
	xor	ax,di
	dec	cl
	jnz	short loc_2
	inc	bx
	dec	ch
	jnz	short loc_1
	push	dx
	push	ax
	pop	eax
	pop	di
	pop	si
	pop	bp
	retn
_CRC32		endp


_hwUpdatePktFlt	proc	near
	push	offset semFlt
	call	_EnterCrit
	mov	cx,MacStatus.sstRxFilter
	xor	eax,eax
	test	cl,mask fltdirect
	jz	short loc_1
	or	eax,rfAPM or rfMHEN	; pmatch and mulicasthash
loc_1:
	test	cl,mask fltbroad
	jz	short loc_2
	or	eax,rfAAB		; broadcast
loc_2:
	test	cl,mask fltprms
	jz	short loc_3
	mov	eax,rfAAB or rfAAM or rfAAU	; promiscous - all
loc_3:
	test	eax,eax
	jz	short loc_4		; all reject
	or	eax,RFEN		; rx filter enable
loc_4:
	mov	[regReceiveMode],eax
	mov	gs:[Reg.RFCR],eax
	call	_LeaveCrit
	pop	cx
	mov	ax,SUCCESS
	retn
_hwUpdatePktFlt	endp

_hwSetMACaddr	proc	near
	push	si
	push	offset semFlt
	call	_EnterCrit
	mov	si,offset MacChar.mctcsa
	mov	ax,[si]
	or	ax,[si+2]
	or	ax,[si+4]
	jnz	short loc_1
	mov	si,offset MacChar.mctpsa
loc_1:
	sub	ebx,ebx
	xor	eax,eax
	mov	edx,gs:[Reg.RFCR]	; backup
	mov	bx,4
loc_2:
	mov	ax,[bx+si]
	mov	gs:[Reg.RFCR],ebx	; address 0..4
	mov	gs:[Reg.RFDR],eax
	sub	bx,2
	jge	short loc_2
	mov	gs:[Reg.RFCR],edx	; restore
	call	_LeaveCrit
	pop	cx
	mov	ax,SUCCESS
	pop	si
	retn
_hwSetMACaddr	endp

_hwUpdateStat	proc	near
	push	offset semStat
	call	_EnterCrit

	mov	bx,offset MacStatus

	mov	eax,gs:[Reg.MIB_RXErroredPkts]
	add	[bx].mst.rxframehw,eax

	mov	eax,gs:[Reg.MIB_RXFCSErrors]
	add	[bx].mst.rxframecrc,eax

	mov	eax,gs:[Reg.MIB_RXMsdPktErrors]
	add	[bx].mst.rxframebuf,eax

	mov	eax,gs:[Reg.MIB_RXFAErrors]
	add	[bx].mst.rxframecrc,eax

	mov	eax,gs:[Reg.MIB_RXSymbolErrors]
	add	[bx].mst.rxframehw,eax

	mov	eax,gs:[Reg.MIB_RXFrameTooLong]
	add	[bx].mst.rxframebuf,eax

	mov	eax,gs:[Reg.MIB_TXSQEErrors]
	add	[bx].mst.txframehw,eax

	call	_LeaveCrit
	pop	ax
	retn
_hwUpdateStat	endp

_hwClearStat	proc	near
	mov	gs:[Reg.MIBC],ACLR
	retn
_hwClearStat	endp

_SetSpeedStat	proc	near
	mov	al,MediaSpeed
	mov	ah,0
	dec	ax
	jz	short loc_10M
	dec	ax
	jz	short loc_100M
	dec	ax
	jz	short loc_1G
	xor	eax,eax
	jmp	short loc_1
loc_10M:
	mov	eax,10000000
	jmp	short loc_1
loc_100M:
	mov	eax,100000000
	jmp	short loc_1
loc_1G:
	mov	eax,1000000000
loc_1:
	mov	[MacChar.linkspeed],eax
	retn
_SetSpeedStat	endp


_hwClose	proc	near
	push	offset semTx
	call	_EnterCrit
	push	offset semRx
	call	_EnterCrit

	xor	eax,eax
	mov	gs:[Reg.IER],eax
	mov	[regIntMask],eax
	mov	gs:[Reg.IMR],eax
	mov	gs:[Reg.CR],TXD or RXD
	mov	gs:[Reg.TXDP],eax
	mov	gs:[Reg.RXDP],eax
	mov	eax,gs:[Reg.ISR]	; clear

	call	_LeaveCrit
	pop	dx
	call	_LeaveCrit
	pop	dx

	mov	ax,SUCCESS
	retn
_hwClose	endp

_hwReset	proc	near	; call in bind process
	enter	6,0

	mov	gs:[Reg.CR],RST		; reset
	mov	byte ptr [bp-2],64	; about 2 second.
loc_1:
	call	_Delay1ms
	test	gs:[Reg.CR],RST		; reset complete?
	jz	short loc_2
	dec	byte ptr [bp-2]
	jnz	short loc_1
	mov	ax,HARDWARE_FAILURE
	leave
	retn
loc_2:
		; parity error action, internal PHY mode select
	mov	gs:[Reg.CFG],PESEL or ANEG_SEL or PAUSE_ADV

	xor	eax,eax
	mov	gs:[Reg.CCSR],PMESTS	; kill CLKRUN
	mov	gs:[Reg.WCSR],eax	; kill Wake on Lan

		; get Station address for EEPROM
	push	6		; PMATCH[0]
	call	_eepRead
	ror	ax,1
	mov	[bp-6],ax
	
	push	7		; PMATCH[1:16]
	call	_eepRead
	mov	dx,[bp-6]
	mov	cx,15
loc_s1:
	shl	ax,1
	rcr	dx,1
	dec	cx
	jnz	short loc_s1
	mov	[bp-6],dx
	mov	[bp-4],ax
	push	8		; PMATCH[17:32]
	call	_eepRead
	mov	dx,[bp-4]
	mov	cx,15
loc_s2:
	shl	ax,1
	rcr	dx,1
	dec	cx
	jnz	short loc_s2
	mov	[bp-4],dx
	mov	[bp-2],ax
	push	9		; PMATCH[33:47]
	call	_eepRead
	mov	dx,[bp-2]
	mov	cx,15
loc_s3:
	shl	ax,1
	rcr	dx,1
	dec	cx
	jnz	short loc_s3
	mov	[bp-2],dx
	add	sp,4*2

	push	offset semFlt
	call	_EnterCrit
	mov	ax,[bp-6]
	mov	cx,[bp-4]
	mov	dx,[bp-2]
	mov	word ptr MacChar.mctpsa,ax	; parmanent
	mov	word ptr MacChar.mctpsa[2],cx
	mov	word ptr MacChar.mctpsa[4],dx
	mov	word ptr MacChar.mctcsa,ax	; current
	mov	word ptr MacChar.mctcsa[2],cx
	mov	word ptr MacChar.mctcsa[4],dx
	mov	word ptr MacChar.mctVendorCode,ax ; vendor
	mov	byte ptr MacChar.mctVendorCode,cl
	call	_LeaveCrit
	add	sp,2
	call	_hwSetMACaddr		; update PMATCH in Receive Filter
	mov	ax,SUCCESS
	leave
	retn
_hwReset	endp


; USHORT miiRead( UCHAR phyaddr, UCHAR phyreg)
_miiRead	proc	near
	push	bp
	mov	bp,sp
	push	offset semMii
	call	_EnterCrit
	mov	bx,offset [Reg.MEAR]
;	push	8
	push	1
	xor	eax,eax

	mov	al,MDIO or MDDIR
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax	; idle
	call	__IODelayCnt

	mov	dl,[bp+4]	; physaddr (5bit)
	mov	cl,[bp+6]	; phyreg   (5bit)
	shl	dx,5
	and	cl,1fh
;	and	dx,3E0h
	and	dh,3
	or	dl,cl
;	or	dx,0110b shl 10	; start(01) + opcode(10)
	or	dh,0110b shl 2
	mov	cx,13

loc_1:
	mov	al,0
	bt	dx,cx
	rcl	al,5		; MDIO
	or	al,MDDIR
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax
	call	__IODelayCnt
	dec	cx
	jge	short loc_1

	mov	al,0
	mov	gs:[bx],eax	; TA (z0)
	mov	al,MDC
	mov	gs:[bx],eax

	mov	cx,16
loc_2:
	mov	al,0
	mov	gs:[bx],eax
	call	__IODelayCnt
	mov	al,MDC
	mov	gs:[bx],eax
	call	__IODelayCnt
	mov	edx,gs:[bx]
	bt	dx,4		; MgmtData
	rcl	bp,1
	dec	cx
	jnz	short loc_2

	mov	al,0
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax	; idle
	call	__IODelayCnt
	mov	ax,bp
	pop	cx
	call	_LeaveCrit
	pop	cx
	pop	bp
	retn
_miiRead	endp

; VOID miiWrite( UCHAR phyaddr, UCHAR phyreg, USHORT value)
_miiWrite	proc	near
	push	bp
	mov	bp,sp
	push	offset semMii
	call	_EnterCrit
	mov	bx,offset [Reg.MEAR]
;	push	8
	push	1
	xor	eax,eax

	mov	al,MDIO or MDDIR
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax	; idle
	call	__IODelayCnt

	mov	dl,[bp+4]	; physaddr (5bit)
	mov	cl,[bp+6]	; phyreg   (5bit)
	shl	dx,5
	and	cl,1fh
	and	dx,3E0h
	or	dl,cl
	or	dx,0101b shl 10	; start(01) + opcode(01)
	mov	cx,14-1

loc_1:
	mov	al,0
	bt	dx,cx
	rcl	al,5		; MDIO
	or	al,MDDIR
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax
	call	__IODelayCnt
	dec	cx
	jge	short loc_1

	mov	al,MDIO or MDDIR	; TA (10)
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax
	call	__IODelayCnt
	mov	al,MDDIR
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax
	call	__IODelayCnt

	mov	dx,[bp+8]
	mov	cx,15
loc_2:
	bt	dx,cx
	mov	al,0
	rcl	al,5
	or	al,MDDIR
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax
	call	__IODelayCnt
	dec	cx
	jge	short loc_2

	mov	al,MDIO or MDDIR
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax	; idle
	call	__IODelayCnt
	pop	bx	;stack adjust
	call	_LeaveCrit
	leave
	retn
_miiWrite	endp

; VOID miiReset( VOID )
_miiReset	proc	near
	push	offset semMii
	call	_EnterCrit
	mov	bx,offset [Reg.MEAR]
	xor	eax,eax
	mov	cx,32		; 32clock high
;	push	8		; _DelayShort loop count
	push	1
loc_1:
	mov	al,MDIO or MDDIR
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,MDC
	mov	gs:[bx],eax
	call	__IODelayCnt
	dec	cx
	jnz	short loc_1
	pop	cx	; stack adjust
;	mov	al,0
;	mov	gs:[bx],eax
	call	_LeaveCrit
	pop	ax	; stack adjust
	retn
_miiReset	endp

IF 0
; VOID _DelayShort( UCHAR count)
__DelayShort	proc	near
	push	bp
	mov	bp,sp
	push	eax
	mov	bp,[bp+4]
loc_1:
	dec	bp
	mov	eax,gs:[Reg.SRR]
	jnz	short loc_1
	pop	eax
	pop	bp
	retn
__DelayShort	endp
ENDIF

; USHORT eepRead( UCHAR addr )
; read opcode 01b
; address mask 3fh(6bit)
_eepRead	proc	near
	push	bp
	mov	bp,sp
	mov	bx,offset Reg.MEAR

	xor	eax,eax
	mov	gs:[bx],eax	; chip select - low
;	push	1
	push	4
	call	__IODelayCnt
	mov	al,EECLK
	mov	gs:[bx],al
	call	__IODelayCnt

	mov	dl,[bp+4]
	mov	dh,0
	and	dl,3fh
	mov	cx,(1 + 2 + 6) -1
	or	dx,110b shl 6
loc_1:
	bt	dx,cx
	setc	al
	or	al,EESEL
	mov	gs:[bx],eax
	call	__IODelayCnt
	or	al,EECLK
	mov	gs:[bx],eax
	call	__IODelayCnt
	dec	cx
	jge	short loc_1

	mov	cx,16
	xor	dx,dx
loc_2:
	mov	al,EESEL
	mov	gs:[bx],eax
	call	__IODelayCnt
	mov	al,EESEL or EECLK
	mov	gs:[bx],eax
	call	__IODelayCnt
	bt	dword ptr gs:[bx],1	; EEDO
	rcl	dx,1
	dec	cx
	jnz	short loc_2

	mov	al,0
	mov	gs:[bx],eax
	call	__IODelayCnt
	mov	al,EECLK
	call	__IODelayCnt
	pop	cx
	mov	ax,dx
	pop	bp
	retn
_eepRead	endp

; void _IODelayCnt( USHORT count )
__IODelayCnt	proc	near
	push	bp
	mov	bp,sp
	push	cx
	mov	bp,[bp+4]
loc_1:
	mov	cx,offset DosIODelayCnt
	dec	bp
	loop	$
	jnz	short loc_1
	pop	cx
	pop	bp
	retn
__IODelayCnt	endp


_TEXT	ends
end
