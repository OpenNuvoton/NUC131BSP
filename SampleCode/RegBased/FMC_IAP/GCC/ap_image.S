;/******************************************************************************
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

	.syntax	unified
	.arch	armv6-m

	.globl	loaderImageBase
	.globl	loaderImageLimit

	.align	4
 
	.text
        
loaderImageBase:
    .incbin  "../../LDROM_iap/LDROM_iap.bin"
loaderImageLimit:
    .space   4
    
    .end
   
