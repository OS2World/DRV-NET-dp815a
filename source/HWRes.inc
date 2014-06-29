; *** Hardware dependent resident part definition

; --- DMA Descriptor management ---
extern	VTxHead : word
extern	VTxTail : word
extern	VTxFreeHead : word
extern	VTxFreeTail : word
extern	TxTail : word
extern	TxFreeHead : word
extern	TxFreeTail : word

extern	RxHead : word
extern	RxTail : word
extern	RxBusyHead : word
extern	RxBusyTail : word
extern	RxInProg : word

; --- System(PCI) Resource ---
extern	IOaddr : word
extern	MEMSel : word		; selector for MEMaddr
extern	MEMaddr : dword
extern	IRQlevel : byte

; --- Configuration Memory Image Parameters ---
extern	cfgSLOT : byte
extern	cfgTXQUEUE : byte
extern	cfgRXQUEUE : byte
extern	cfgMAXFRAMESIZE : word

extern	cfgTxDRTH : byte
extern	cfgTxFLTH : byte
extern	cfgTxMXDMA : byte
extern	cfgRxDRTH : byte
extern	cfgRxMXDMA : byte
extern	cfgIHR : word

; --- Receive Buffer Address ---
extern	RxBufferLin : dword
extern	RxBufferPhys : dword
extern	RxBufferSize : dword
extern	RxBufferSelCnt : word
extern	RxBufferSel : word

; --- Vendor Adapter Description ---
extern	AdapterDesc : byte

; --- Driver Version ---
extern	DrvMajVer : abs
extern	DrvMinVer : abs


; --- Hardware specific services ---
extern	_hwTxChain : near16
extern	_hwRxRelease : near16
extern	_hwEnableRxInd : near16
extern	_hwDisableRxInd : near16
extern	_hwSetMACaddr : near16
extern	_hwOpen : near16
extern	_hwClose : near16
extern	_hwIntReq : near16
extern	_hwUpdatePktFlt : near16
extern	_hwUpdateMulticast : near16
extern	_hwUpdateStat : near16
extern	_hwClearStat : near16
extern	_hwReset : near16

extern	_hwCheckInt : near16
extern	_hwDisableInt : near16
extern	_hwEnableInt : near16
extern	_hwServiceInt : near16
extern	_hwPollLink : near16
