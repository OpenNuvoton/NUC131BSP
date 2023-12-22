;/******************************************************************************
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

	.syntax	unified
	.arch	armv6-m

	.globl	loaderImage1Base
	.globl	loaderImage1Limit

	.align	4
 
	.text
        
loaderImage1Base:
    .incbin  "../../LDROM_iap/LDROM_iap.bin"
loaderImage1Limit:
    .space   4
    
    .end
   
