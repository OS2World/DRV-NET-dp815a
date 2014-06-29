; misc...

; --- driver flags (DrvRes.asm) ---
driver_flags	record dfres:11=0,
		df_i15o:1=0,
		df_rstreq:1=0,
		df_intreq:1=0,
		df_rxsp:1=0,
		df_idcp:1=0

; --- PHY Information (HWRes.asm) ---
_PhyInfo	struc
Phyaddr		dw	?	; =1
BMSR		dw	?	; Basic Mode Status
ANAR		dw	?	; Auto Negotiation Advertisement
ANLPAR		dw	?	; Auto Nego. Link Partner Ability
GTCR		dw	?	; 1K control
GSTSR		dw	?	; 1K status
GSCR		dw	?	; 1K Extended status
_PhyInfo	ends

