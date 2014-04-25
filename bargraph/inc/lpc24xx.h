
#ifndef __ASM_ARCH_LPC24xx_H
#define __ASM_ARCH_LPC24xx_H

/* Vectored Interrupt Controller (VIC) */
#define VICIRQStatus    0xFFFFF000
#define VICFIQStatus    0xFFFFF004
#define VICRawIntr      0xFFFFF008
#define VICIntSelect    0xFFFFF00C
#define VICIntEnable    0xFFFFF010
#define VICIntEnClr     0xFFFFF014
#define VICSoftInt      0xFFFFF018
#define VICSoftIntClear 0xFFFFF01C
#define VICProtection   0xFFFFF020
#define VICSWPrioMask   0xFFFFF024
#define VICVectAddr     0xFFFFFF00
#define VICVectAddr0    0xFFFFF100
#define VICVectAddr1    0xFFFFF104
#define VICVectAddr2    0xFFFFF108
#define VICVectAddr3    0xFFFFF10C
#define VICVectAddr4    0xFFFFF110
#define VICVectAddr5    0xFFFFF114
#define VICVectAddr6    0xFFFFF118
#define VICVectAddr7    0xFFFFF11C
#define VICVectAddr8    0xFFFFF120
#define VICVectAddr9    0xFFFFF124
#define VICVectAddr10   0xFFFFF128
#define VICVectAddr11   0xFFFFF12C
#define VICVectAddr12   0xFFFFF130
#define VICVectAddr13   0xFFFFF134
#define VICVectAddr14   0xFFFFF138
#define VICVectAddr15   0xFFFFF13C
#define VICVectAddr16   0xFFFFF140
#define VICVectAddr17   0xFFFFF144
#define VICVectAddr18   0xFFFFF148
#define VICVectAddr19   0xFFFFF14C
#define VICVectAddr20   0xFFFFF150
#define VICVectAddr21   0xFFFFF154
#define VICVectAddr22   0xFFFFF158
#define VICVectAddr23   0xFFFFF15C
#define VICVectAddr24   0xFFFFF160
#define VICVectAddr25   0xFFFFF164
#define VICVectAddr26   0xFFFFF168
#define VICVectAddr27   0xFFFFF16C
#define VICVectAddr28   0xFFFFF170
#define VICVectAddr29   0xFFFFF174
#define VICVectAddr30   0xFFFFF178
#define VICVectAddr31   0xFFFFF17C
#define VICVectCntl0    0xFFFFF200
#define VICVectCntl1    0xFFFFF204
#define VICVectCntl2    0xFFFFF208
#define VICVectCntl3    0xFFFFF20C
#define VICVectCntl4    0xFFFFF210
#define VICVectCntl5    0xFFFFF214
#define VICVectCntl6    0xFFFFF218
#define VICVectCntl7    0xFFFFF21C
#define VICVectCntl8    0xFFFFF220
#define VICVectCntl9    0xFFFFF224
#define VICVectCntl10   0xFFFFF228
#define VICVectCntl11   0xFFFFF22C
#define VICVectCntl12   0xFFFFF230
#define VICVectCntl13   0xFFFFF234
#define VICVectCntl14   0xFFFFF238
#define VICVectCntl15   0xFFFFF23C
#define VICVectCntl16   0xFFFFF240
#define VICVectCntl17   0xFFFFF244
#define VICVectCntl18   0xFFFFF248
#define VICVectCntl19   0xFFFFF24C
#define VICVectCntl20   0xFFFFF250
#define VICVectCntl21   0xFFFFF254
#define VICVectCntl22   0xFFFFF258
#define VICVectCntl23   0xFFFFF25C
#define VICVectCntl24   0xFFFFF260
#define VICVectCntl25   0xFFFFF264
#define VICVectCntl26   0xFFFFF268
#define VICVectCntl27   0xFFFFF26C
#define VICVectCntl28   0xFFFFF270
#define VICVectCntl29   0xFFFFF274
#define VICVectCntl30   0xFFFFF278
#define VICVectCntl31   0xFFFFF27C
#define VICVectPriority0    0xFFFFF200
#define VICVectPriority1    0xFFFFF204
#define VICVectPriority2    0xFFFFF208
#define VICVectPriority3    0xFFFFF20C
#define VICVectPriority4    0xFFFFF210
#define VICVectPriority5    0xFFFFF214
#define VICVectPriority6    0xFFFFF218
#define VICVectPriority7    0xFFFFF21C
#define VICVectPriority8    0xFFFFF220
#define VICVectPriority9    0xFFFFF224
#define VICVectPriority10   0xFFFFF228
#define VICVectPriority11   0xFFFFF22C
#define VICVectPriority12   0xFFFFF230
#define VICVectPriority13   0xFFFFF234
#define VICVectPriority14   0xFFFFF238
#define VICVectPriority15   0xFFFFF23C
#define VICVectPriority16   0xFFFFF240
#define VICVectPriority17   0xFFFFF244
#define VICVectPriority18   0xFFFFF248
#define VICVectPriority19   0xFFFFF24C
#define VICVectPriority20   0xFFFFF250
#define VICVectPriority21   0xFFFFF254
#define VICVectPriority22   0xFFFFF258
#define VICVectPriority23   0xFFFFF25C
#define VICVectPriority24   0xFFFFF260
#define VICVectPriority25   0xFFFFF264
#define VICVectPriority26   0xFFFFF268
#define VICVectPriority27   0xFFFFF26C
#define VICVectPriority28   0xFFFFF270
#define VICVectPriority29   0xFFFFF274
#define VICVectPriority30   0xFFFFF278
#define VICVectPriority31   0xFFFFF27C

/*
	Register define for constant
*/	
#define	REG_U0RBR				0xE000C000
#define REG_U1RBR				0xE0010000


/* Pin Connect Block */
#define PINSEL_BASE_ADDR	0xE002C000
#define PINSEL0        0xE002C000
#define PINSEL1        0xE002C004
#define PINSEL2        0xE002C008
#define PINSEL3        0xE002C00C
#define PINSEL4        0xE002C010
#define PINSEL5        0xE002C014
#define PINSEL6        0xE002C018
#define PINSEL7        0xE002C01C
#define PINSEL8        0xE002C020
#define PINSEL9        0xE002C024
#define PINSEL10       0xE002C028

#define PINMODE0        0xE002C040
#define PINMODE1        0xE002C044
#define PINMODE2        0xE002C048
#define PINMODE3        0xE002C04C
#define PINMODE4        0xE002C050
#define PINMODE5        0xE002C054
#define PINMODE6        0xE002C058
#define PINMODE7        0xE002C05C
#define PINMODE8        0xE002C060
#define PINMODE9        0xE002C064

/* General Purpose Input/Output (GPIO) */
#define GPIO_BASE_ADDR		0xE0028000
#define IOPIN0         0xE0028000
#define IOSET0         0xE0028004
#define IODIR0         0xE0028008
#define IOCLR0         0xE002800C
#define IOPIN1         0xE0028010
#define IOSET1         0xE0028014
#define IODIR1         0xE0028018
#define IOCLR1         0xE002801C

/* GPIO Interrupt Registers */
#define IO0_INT_EN_R    0xE0028090 
#define IO0_INT_EN_F    0xE0028094
#define IO0_INT_STAT_R  0xE0028084
#define IO0_INT_STAT_F  0xE0028088
#define IO0_INT_CLR     0xE002808C

#define IO2_INT_EN_R    0xE00280B0 
#define IO2_INT_EN_F    0xE00280B4
#define IO2_INT_STAT_R  0xE00280A4
#define IO2_INT_STAT_F  0xE00280A8
#define IO2_INT_CLR     0xE00280AC

#define IO_INT_STAT     0xE0028080

#define PARTCFG_BASE_ADDR		0x3FFF8000
#define PARTCFG        0x3FFF8000

/* Fast I/O setup */
#define FIO_BASE_ADDR		0x3FFFC000
#define FIO0DIR        0x3FFFC000 
#define FIO0MASK       0x3FFFC010
#define FIO0PIN        0x3FFFC014
#define FIO0SET        0x3FFFC018
#define FIO0CLR        0x3FFFC01C

#define FIO1DIR        0x3FFFC020 
#define FIO1MASK       0x3FFFC030
#define FIO1PIN        0x3FFFC034
#define FIO1SET        0x3FFFC038
#define FIO1CLR        0x3FFFC03C

#define FIO2DIR        0x3FFFC040 
#define FIO2MASK       0x3FFFC050
#define FIO2PIN        0x3FFFC054
#define FIO2SET        0x3FFFC058
#define FIO2CLR        0x3FFFC05C

#define FIO3DIR        0x3FFFC060 
#define FIO3MASK       0x3FFFC070
#define FIO3PIN        0x3FFFC074
#define FIO3SET        0x3FFFC078
#define FIO3CLR        0x3FFFC07C

#define FIO4DIR        0x3FFFC080 
#define FIO4MASK       0x3FFFC090
#define FIO4PIN        0x3FFFC094
#define FIO4SET        0x3FFFC098
#define FIO4CLR        0x3FFFC09C

/* FIOs can be accessed through WORD, HALF-WORD or BYTE. */
#define FIO0DIR0       0x3FFFC001 
#define FIO1DIR0       0x3FFFC021 
#define FIO2DIR0       0x3FFFC041 
#define FIO3DIR0       0x3FFFC061 
#define FIO4DIR0       0x3FFFC081 

#define FIO0DIR1       0x3FFFC002 
#define FIO1DIR1       0x3FFFC022 
#define FIO2DIR1       0x3FFFC042 
#define FIO3DIR1       0x3FFFC062 
#define FIO4DIR1       0x3FFFC082 

#define FIO0DIR2       0x3FFFC003 
#define FIO1DIR2       0x3FFFC023 
#define FIO2DIR2       0x3FFFC043 
#define FIO3DIR2       0x3FFFC063 
#define FIO4DIR2       0x3FFFC083 

#define FIO0DIR3       0x3FFFC004 
#define FIO1DIR3       0x3FFFC024 
#define FIO2DIR3       0x3FFFC044 
#define FIO3DIR3       0x3FFFC064 
#define FIO4DIR3       0x3FFFC084 

#define FIO0DIRL       0x3FFFC000 
#define FIO1DIRL       0x3FFFC020 
#define FIO2DIRL       0x3FFFC040 
#define FIO3DIRL       0x3FFFC060 
#define FIO4DIRL       0x3FFFC080 

#define FIO0DIRU       0x3FFFC002 
#define FIO1DIRU       0x3FFFC022 
#define FIO2DIRU       0x3FFFC042 
#define FIO3DIRU       0x3FFFC062 
#define FIO4DIRU       0x3FFFC082 

#define FIO0MASK0      0x3FFFC010 
#define FIO1MASK0      0x3FFFC030 
#define FIO2MASK0      0x3FFFC050 
#define FIO3MASK0      0x3FFFC070 
#define FIO4MASK0      0x3FFFC090 

#define FIO0MASK1      0x3FFFC011 
#define FIO1MASK1      0x3FFFC021 
#define FIO2MASK1      0x3FFFC051 
#define FIO3MASK1      0x3FFFC071 
#define FIO4MASK1      0x3FFFC091 

#define FIO0MASK2      0x3FFFC012 
#define FIO1MASK2      0x3FFFC032 
#define FIO2MASK2      0x3FFFC052 
#define FIO3MASK2      0x3FFFC072 
#define FIO4MASK2      0x3FFFC092 

#define FIO0MASK3      0x3FFFC013 
#define FIO1MASK3      0x3FFFC033 
#define FIO2MASK3      0x3FFFC053 
#define FIO3MASK3      0x3FFFC073 
#define FIO4MASK3      0x3FFFC093 

#define FIO0MASKL      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x10)) 
#define FIO1MASKL      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x30)) 
#define FIO2MASKL      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x50)) 
#define FIO3MASKL      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x70)) 
#define FIO4MASKL      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x90)) 

#define FIO0MASKU      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x12)) 
#define FIO1MASKU      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x32)) 
#define FIO2MASKU      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x52)) 
#define FIO3MASKU      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x72)) 
#define FIO4MASKU      (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x92)) 

#define FIO0PIN0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x14)) 
#define FIO1PIN0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x34)) 
#define FIO2PIN0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x54)) 
#define FIO3PIN0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x74)) 
#define FIO4PIN0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x94)) 

#define FIO0PIN1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x15)) 
#define FIO1PIN1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x25)) 
#define FIO2PIN1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x55)) 
#define FIO3PIN1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x75)) 
#define FIO4PIN1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x95)) 

#define FIO0PIN2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x16)) 
#define FIO1PIN2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x36)) 
#define FIO2PIN2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x56)) 
#define FIO3PIN2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x76)) 
#define FIO4PIN2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x96)) 

#define FIO0PIN3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x17)) 
#define FIO1PIN3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x37)) 
#define FIO2PIN3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x57)) 
#define FIO3PIN3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x77)) 
#define FIO4PIN3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x97)) 

#define FIO0PINL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x14)) 
#define FIO1PINL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x34)) 
#define FIO2PINL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x54)) 
#define FIO3PINL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x74)) 
#define FIO4PINL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x94)) 

#define FIO0PINU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x16)) 
#define FIO1PINU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x36)) 
#define FIO2PINU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x56)) 
#define FIO3PINU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x76)) 
#define FIO4PINU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x96)) 

#define FIO0SET0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x18)) 
#define FIO1SET0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x38)) 
#define FIO2SET0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x58)) 
#define FIO3SET0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x78)) 
#define FIO4SET0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x98)) 

#define FIO0SET1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x19)) 
#define FIO1SET1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x29)) 
#define FIO2SET1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x59)) 
#define FIO3SET1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x79)) 
#define FIO4SET1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x99)) 

#define FIO0SET2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x1A)) 
#define FIO1SET2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x3A)) 
#define FIO2SET2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x5A)) 
#define FIO3SET2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x7A)) 
#define FIO4SET2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x9A)) 

#define FIO0SET3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x1B)) 
#define FIO1SET3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x3B)) 
#define FIO2SET3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x5B)) 
#define FIO3SET3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x7B)) 
#define FIO4SET3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x9B)) 

#define FIO0SETL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x18)) 
#define FIO1SETL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x38)) 
#define FIO2SETL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x58)) 
#define FIO3SETL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x78)) 
#define FIO4SETL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x98)) 

#define FIO0SETU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x1A)) 
#define FIO1SETU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x3A)) 
#define FIO2SETU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x5A)) 
#define FIO3SETU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x7A)) 
#define FIO4SETU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x9A)) 

#define FIO0CLR0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x1C)) 
#define FIO1CLR0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x3C)) 
#define FIO2CLR0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x5C)) 
#define FIO3CLR0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x7C)) 
#define FIO4CLR0       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x9C)) 

#define FIO0CLR1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x1D)) 
#define FIO1CLR1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x2D)) 
#define FIO2CLR1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x5D)) 
#define FIO3CLR1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x7D)) 
#define FIO4CLR1       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x9D)) 

#define FIO0CLR2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x1E)) 
#define FIO1CLR2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x3E)) 
#define FIO2CLR2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x5E)) 
#define FIO3CLR2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x7E)) 
#define FIO4CLR2       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x9E)) 

#define FIO0CLR3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x1F)) 
#define FIO1CLR3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x3F)) 
#define FIO2CLR3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x5F)) 
#define FIO3CLR3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x7F)) 
#define FIO4CLR3       (*(volatile unsigned char *)(FIO_BASE_ADDR + 0x9F)) 

#define FIO0CLRL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x1C)) 
#define FIO1CLRL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x3C)) 
#define FIO2CLRL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x5C)) 
#define FIO3CLRL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x7C)) 
#define FIO4CLRL       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x9C)) 

#define FIO0CLRU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x1E)) 
#define FIO1CLRU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x3E)) 
#define FIO2CLRU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x5E)) 
#define FIO3CLRU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x7E)) 
#define FIO4CLRU       (*(volatile unsigned short *)(FIO_BASE_ADDR + 0x9E)) 


/* System Control Block(SCB) modules include Memory Accelerator Module,
Phase Locked Loop, VPB divider, Power Control, External Interrupt, 
Reset, and Code Security/Debugging */
#define SCB_BASE_ADDR	0xE01FC000

/* Memory Accelerator Module (MAM) */
#define MAMCR          0xE01FC000
#define MAMTIM         0xE01FC004
#define MEMMAP         0xE01FC040

/* Phase Locked Loop (PLL) */
#define PLLCON         0xE01FC080
#define PLLCFG         0xE01FC084
#define PLLSTAT        0xE01FC088
#define PLLFEED        0xE01FC08C

/* Power Control */
#define PCON           (SCB_BASE_ADDR + 0x0C0)
#define PCONP          (SCB_BASE_ADDR + 0x0C4)

/* Clock Divider */
// #define APBDIV         (*(volatile unsigned long *)(SCB_BASE_ADDR + 0x100))
#define CCLKCFG        0xE01FC104
#define USBCLKCFG      0xE01FC108
#define CLKSRCSEL      0xE01FC10C
#define PCLKSEL0       0xE01FC1A8
#define PCLKSEL1       0xE01FC1AC
	
/* External Interrupts */
#define EXTINT         0xE01FC140
#define INTWAKE        0xE01FC144
#define EXTMODE        0xE01FC148
#define EXTPOLAR       0xE01FC14C

/* Reset, reset source identification */
#define RSIR           0xE01FC180

/* RSID, code security protection */
#define CSPR           0xE01FC184

/* AHB configuration */
#define AHBCFG1        0xE01FC188
#define AHBCFG2        0xE01FC18C

/* System Controls and Status */
#define SCS            0xE01FC1A0	


/* External Memory Controller (EMC) */
#define EMC_BASE_ADDR  0xFFE08000
#define EMC_CTRL       0xFFE08000
#define EMC_STAT       0xFFE08004
#define EMC_CONFIG     0xFFE08008

/* Dynamic RAM access registers */
#define EMC_DYN_CTRL     0xFFE08020
#define EMC_DYN_RFSH     0xFFE08024
#define EMC_DYN_RD_CFG   0xFFE08028
#define EMC_DYN_RP       0xFFE08030
#define EMC_DYN_RAS      0xFFE08034
#define EMC_DYN_SREX     0xFFE08038
#define EMC_DYN_APR      0xFFE0803C
#define EMC_DYN_DAL      0xFFE08040
#define EMC_DYN_WR       0xFFE08044
#define EMC_DYN_RC       0xFFE08048
#define EMC_DYN_RFC      0xFFE0804C
#define EMC_DYN_XSR      0xFFE08050
#define EMC_DYN_RRD      0xFFE08054
#define EMC_DYN_MRD      0xFFE08058

#define EMC_DYN_CFG0     0xFFE08100
#define EMC_DYN_RASCAS0  0xFFE08104
#define EMC_DYN_CFG1     0xFFE08140
#define EMC_DYN_RASCAS1  0xFFE08144
#define EMC_DYN_CFG2     0xFFE08160
#define EMC_DYN_RASCAS2  0xFFE08164
#define EMC_DYN_CFG3     0xFFE08180
#define EMC_DYN_RASCAS3  0xFFE08184

/* static RAM access registers */
#define EMC_STA_CFG0      0xFFE08200
#define EMC_STA_WAITWEN0  0xFFE08204
#define EMC_STA_WAITOEN0  0xFFE08208
#define EMC_STA_WAITRD0   0xFFE0820C
#define EMC_STA_WAITPAGE0 0xFFE08210
#define EMC_STA_WAITWR0   0xFFE08214
#define EMC_STA_WAITTURN0 0xFFE08218

#define EMC_STA_CFG1      0xFFE08220
#define EMC_STA_WAITWEN1  0xFFE08224
#define EMC_STA_WAITOEN1  0xFFE08228
#define EMC_STA_WAITRD1   0xFFE0822C
#define EMC_STA_WAITPAGE1 0xFFE08230
#define EMC_STA_WAITWR1   0xFFE08234
#define EMC_STA_WAITTURN1 0xFFE08238

#define EMC_STA_CFG2      0xFFE08240
#define EMC_STA_WAITWEN2  0xFFE08244
#define EMC_STA_WAITOEN2  0xFFE08248
#define EMC_STA_WAITRD2   0xFFE0824C
#define EMC_STA_WAITPAGE2 0xFFE08250
#define EMC_STA_WAITWR2   0xFFE08254
#define EMC_STA_WAITTURN2 0xFFE08258

#define EMC_STA_CFG3      0xFFE08260
#define EMC_STA_WAITWEN3  0xFFE08264
#define EMC_STA_WAITOEN3  0xFFE08268
#define EMC_STA_WAITRD3   0xFFE0826C
#define EMC_STA_WAITPAGE3 0xFFE08270
#define EMC_STA_WAITWR3   0xFFE08274
#define EMC_STA_WAITTURN3 0xFFE08278

#define EMC_STA_EXT_WAIT  0xFFE08880

	
/* Timer 0 */
#define TMR0_BASE_ADDR		0xE0004000
#define T0IR           0xE0004000
#define T0TCR          0xE0004004
#define T0TC           0xE0004008
#define T0PR           0xE000400C
#define T0PC           0xE0004010
#define T0MCR          0xE0004014
#define T0MR0          0xE0004018
#define T0MR1          0xE000401C
#define T0MR2          0xE0004020
#define T0MR3          0xE0004024
#define T0CCR          0xE0004028
#define T0CR0          0xE000402C
#define T0CR1          0xE0004030
#define T0CR2          0xE0004034
#define T0CR3          0xE0004038
#define T0EMR          0xE000403C
#define T0CTCR         0xE0004070

/* Timer 1 */
#define TMR1_BASE_ADDR		0xE0008000
#define T1IR           0xE0008000
#define T1TCR          0xE0008004
#define T1TC           0xE0008008
#define T1PR           0xE000800C
#define T1PC           0xE0008010
#define T1MCR          0xE0008014
#define T1MR0          0xE0008018
#define T1MR1          0xE000801C
#define T1MR2          0xE0008020
#define T1MR3          0xE0008024
#define T1CCR          0xE0008028
#define T1CR0          0xE000802C
#define T1CR1          0xE0008030
#define T1CR2          0xE0008034
#define T1CR3          0xE0008038
#define T1EMR          0xE000803C
#define T1CTCR         0xE0008070

/* Timer 2 */
#define TMR2_BASE_ADDR		0xE0070000
#define T2IR           0xE0070000
#define T2TCR          0xE0070004
#define T2TC           0xE0070008
#define T2PR           0xE007000C
#define T2PC           0xE0070010
#define T2MCR          0xE0070014
#define T2MR0          0xE0070018
#define T2MR1          0xE007001C
#define T2MR2          0xE0070020
#define T2MR3          0xE0070024
#define T2CCR          0xE0070028
#define T2CR0          0xE007002C
#define T2CR1          0xE0070030
#define T2CR2          0xE0070034
#define T2CR3          0xE0070038
#define T2EMR          0xE007003C
#define T2CTCR         0xE0070070

/* Timer 3 */
#define TMR3_BASE_ADDR		0xE0074000
#define T3IR           0xE0074000
#define T3TCR          0xE0074004
#define T3TC           0xE0074008
#define T3PR           0xE007400C
#define T3PC           0xE0074010
#define T3MCR          0xE0074014
#define T3MR0          0xE0074018
#define T3MR1          0xE007401C
#define T3MR2          0xE0074020
#define T3MR3          0xE0074024
#define T3CCR          0xE0074028
#define T3CR0          0xE007402C
#define T3CR1          0xE0074030
#define T3CR2          0xE0074034
#define T3CR3          0xE0074038
#define T3EMR          0xE007403C
#define T3CTCR         0xE0074070


/* Pulse Width Modulator (PWM) */
#define PWM0_BASE_ADDR		0xE0014000
#define PWM0IR          0xE0014000
#define PWM0TCR         0xE0014004
#define PWM0TC          0xE0014008
#define PWM0PR          0xE001400C
#define PWM0PC          0xE0014010
#define PWM0MCR         0xE0014014
#define PWM0MR0         0xE0014018
#define PWM0MR1         0xE001401C
#define PWM0MR2         0xE0014020
#define PWM0MR3         0xE0014024
#define PWM0CCR         0xE0014028
#define PWM0CR0         0xE001402C
#define PWM0CR1         0xE0014030
#define PWM0CR2         0xE0014034
#define PWM0CR3         0xE0014038
#define PWM0EMR         0xE001403C
#define PWM0MR4         0xE0014040
#define PWM0MR5         0xE0014044
#define PWM0MR6         0xE0014048
#define PWM0PCR         0xE001404C
#define PWM0LER         0xE0014050
#define PWM0CTCR        0xE0014070

#define PWM1_BASE_ADDR		0xE0018000
#define PWM1IR          0xE0018000
#define PWM1TCR         0xE0018004
#define PWM1TC          0xE0018008
#define PWM1PR          0xE001800C
#define PWM1PC          0xE0018010
#define PWM1MCR         0xE0018014
#define PWM1MR0         0xE0018018
#define PWM1MR1         0xE001801C
#define PWM1MR2         0xE0018020
#define PWM1MR3         0xE0018024
#define PWM1CCR         0xE0018028
#define PWM1CR0         0xE001802C
#define PWM1CR1         0xE0018030
#define PWM1CR2         0xE0018034
#define PWM1CR3         0xE0018038
#define PWM1EMR         0xE001803C
#define PWM1MR4         0xE0018040
#define PWM1MR5         0xE0018044
#define PWM1MR6         0xE0018048
#define PWM1PCR         0xE001804C
#define PWM1LER         0xE0018050
#define PWM1CTCR        0xE0018070


/* Universal Asynchronous Receiver Transmitter 0 (UART0) */
#define UART0_BASE_ADDR		0xE000C000
#define U0RBR          0xE000C000
#define U0THR          0xE000C000
#define U0DLL          0xE000C000
#define U0DLM          0xE000C004
#define U0IER          0xE000C004
#define U0IIR          0xE000C008
#define U0FCR          0xE000C008
#define U0LCR          0xE000C00C
#define U0LSR          0xE000C014
#define U0SCR          0xE000C01C
#define U0ACR          0xE000C020
#define U0ICR          0xE000C024
#define U0FDR          0xE000C028
#define U0TER          0xE000C030

/* Universal Asynchronous Receiver Transmitter 1 (UART1) */
#define UART1_BASE_ADDR		0xE0010000
#define U1RBR          0xE0010000
#define U1THR          0xE0010000
#define U1DLL          0xE0010000
#define U1DLM          0xE0010004
#define U1IER          0xE0010004
#define U1IIR          0xE0010008
#define U1FCR          0xE0010008
#define U1LCR          0xE001000C
#define U1MCR          0xE0010010
#define U1LSR          0xE0010014
#define U1MSR          0xE0010018
#define U1SCR          0xE001001C
#define U1ACR          0xE0010020
#define U1FDR          0xE0010028
#define U1TER          0xE0010030

/* Universal Asynchronous Receiver Transmitter 2 (UART2) */
#define UART2_BASE_ADDR		0xE0078000
#define U2RBR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x00))
#define U2THR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x00))
#define U2DLL          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x00))
#define U2DLM          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x04))
#define U2IER          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x04))
#define U2IIR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x08))
#define U2FCR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x08))
#define U2LCR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x0C))
#define U2LSR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x14))
#define U2SCR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x1C))
#define U2ACR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x20))
#define U2ICR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x24))
#define U2FDR          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x28))
#define U2TER          (*(volatile unsigned long *)(UART2_BASE_ADDR + 0x30))

/* Universal Asynchronous Receiver Transmitter 3 (UART3) */
#define UART3_BASE_ADDR		0xE007C000
#define U3RBR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x00))
#define U3THR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x00))
#define U3DLL          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x00))
#define U3DLM          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x04))
#define U3IER          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x04))
#define U3IIR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x08))
#define U3FCR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x08))
#define U3LCR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x0C))
#define U3LSR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x14))
#define U3SCR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x1C))
#define U3ACR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x20))
#define U3ICR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x24))
#define U3FDR          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x28))
#define U3TER          (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x30))

/* I2C Interface 0 */
#define I2C0_BASE_ADDR		0xE001C000
#define I20CONSET      0xE001C000
#define I20STAT        0xE001C004
#define I20DAT         0xE001C008
#define I20ADR         0xE001C00C
#define I20SCLH        0xE001C010
#define I20SCLL        0xE001C014
#define I20CONCLR      0xE001C018

/* I2C Interface 1 */
#define I2C1_BASE_ADDR		0xE005C000
#define I21CONSET      (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x00))
#define I21STAT        (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x04))
#define I21DAT         (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x08))
#define I21ADR         (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x0C))
#define I21SCLH        (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x10))
#define I21SCLL        (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x14))
#define I21CONCLR      (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x18))

/* I2C Interface 2 */
#define I2C2_BASE_ADDR		0xE0080000
#define I22CONSET      (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x00))
#define I22STAT        (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x04))
#define I22DAT         (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x08))
#define I22ADR         (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x0C))
#define I22SCLH        (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x10))
#define I22SCLL        (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x14))
#define I22CONCLR      (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x18))

/* SPI0 (Serial Peripheral Interface 0) */
#define SPI0_BASE_ADDR		0xE0020000
#define S0SPCR         (*(volatile unsigned long *)(SPI0_BASE_ADDR + 0x00))
#define S0SPSR         (*(volatile unsigned long *)(SPI0_BASE_ADDR + 0x04))
#define S0SPDR         (*(volatile unsigned long *)(SPI0_BASE_ADDR + 0x08))
#define S0SPCCR        (*(volatile unsigned long *)(SPI0_BASE_ADDR + 0x0C))
#define S0SPINT        (*(volatile unsigned long *)(SPI0_BASE_ADDR + 0x1C))

/* SSP0 Controller */
#define SSP0_BASE_ADDR		0xE0068000
#define SSP0CR0        (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x00))
#define SSP0CR1        (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x04))
#define SSP0DR         (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x08))
#define SSP0SR         (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x0C))
#define SSP0CPSR       (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x10))
#define SSP0IMSC       (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x14))
#define SSP0RIS        (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x18))
#define SSP0MIS        (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x1C))
#define SSP0ICR        (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x20))
#define SSP0DMACR      (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x24))

/* SSP1 Controller */
#define SSP1_BASE_ADDR		0xE0030000
#define SSP1CR0        (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x00))
#define SSP1CR1        (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x04))
#define SSP1DR         (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x08))
#define SSP1SR         (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x0C))
#define SSP1CPSR       (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x10))
#define SSP1IMSC       (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x14))
#define SSP1RIS        (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x18))
#define SSP1MIS        (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x1C))
#define SSP1ICR        (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x20))
#define SSP1DMACR      (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x24))


/* Real Time Clock */
#define RTC_BASE_ADDR		0xE0024000
#define RTC_ILR         (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x00))
#define RTC_CTC         (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x04))
#define RTC_CCR         (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x08))
#define RTC_CIIR        (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x0C))
#define RTC_AMR         (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x10))
#define RTC_CTIME0      (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x14))
#define RTC_CTIME1      (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x18))
#define RTC_CTIME2      (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x1C))
#define RTC_SEC         (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x20))
#define RTC_MIN         (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x24))
#define RTC_HOUR        (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x28))
#define RTC_DOM         (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x2C))
#define RTC_DOW         (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x30))
#define RTC_DOY         (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x34))
#define RTC_MONTH       (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x38))
#define RTC_YEAR        (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x3C))
#define RTC_CISS        (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x40))
#define RTC_ALSEC       (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x60))
#define RTC_ALMIN       (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x64))
#define RTC_ALHOUR      (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x68))
#define RTC_ALDOM       (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x6C))
#define RTC_ALDOW       (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x70))
#define RTC_ALDOY       (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x74))
#define RTC_ALMON       (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x78))
#define RTC_ALYEAR      (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x7C))
#define RTC_PREINT      (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x80))
#define RTC_PREFRAC     (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x84))


/* A/D Converter 0 (AD0) */
#define AD0_BASE_ADDR		0xE0034000
#define AD0CR          0xE0034000
#define AD0GDR         0xE0034004
#define AD0INTEN       0xE003400C
#define AD0DR0         0xE0034010
#define AD0DR1         0xE0034014
#define AD0DR2         0xE0034018
#define AD0DR3         0xE003401C
#define AD0DR4         0xE0034020
#define AD0DR5         0xE0034024
#define AD0DR6         0xE0034028
#define AD0DR7         0xE003402C
#define AD0STAT        0xE0034030


/* D/A Converter */
#define DAC_BASE_ADDR		0xE006C000
#define DACR           0xE006C000


/* Watchdog */
#define WDG_BASE_ADDR		0xE0000000
#define WDMOD          0xE0000000
#define WDTC           0xE0000004
#define WDFEED         0xE0000008
#define WDTV           0xE000000C
#define WDCLKSEL       0xE0000010

/* CAN CONTROLLERS AND ACCEPTANCE FILTER */
#define CAN_ACCEPT_BASE_ADDR		0xE003C000
#define CAN_AFMR		(*(volatile unsigned long *)(CAN_ACCEPT_BASE_ADDR + 0x00))  	
#define CAN_SFF_SA 		(*(volatile unsigned long *)(CAN_ACCEPT_BASE_ADDR + 0x04))  	
#define CAN_SFF_GRP_SA 	(*(volatile unsigned long *)(CAN_ACCEPT_BASE_ADDR + 0x08))
#define CAN_EFF_SA 		(*(volatile unsigned long *)(CAN_ACCEPT_BASE_ADDR + 0x0C))
#define CAN_EFF_GRP_SA 	(*(volatile unsigned long *)(CAN_ACCEPT_BASE_ADDR + 0x10))  	
#define CAN_EOT 		(*(volatile unsigned long *)(CAN_ACCEPT_BASE_ADDR + 0x14))
#define CAN_LUT_ERR_ADR (*(volatile unsigned long *)(CAN_ACCEPT_BASE_ADDR + 0x18))  	
#define CAN_LUT_ERR 	(*(volatile unsigned long *)(CAN_ACCEPT_BASE_ADDR + 0x1C))

#define CAN_CENTRAL_BASE_ADDR		0xE0040000  	
#define CAN_TX_SR 	(*(volatile unsigned long *)(CAN_CENTRAL_BASE_ADDR + 0x00))  	
#define CAN_RX_SR 	(*(volatile unsigned long *)(CAN_CENTRAL_BASE_ADDR + 0x04))  	
#define CAN_MSR 	(*(volatile unsigned long *)(CAN_CENTRAL_BASE_ADDR + 0x08))

#define CAN1_BASE_ADDR		0xE0044000
#define CAN1MOD 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x00))  	
#define CAN1CMR 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x04))  	
#define CAN1GSR 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x08))  	
#define CAN1ICR 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x0C))  	
#define CAN1IER 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x10))
#define CAN1BTR 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x14))  	
#define CAN1EWL 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x18))  	
#define CAN1SR 		(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x1C))  	
#define CAN1RFS 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x20))  	
#define CAN1RID 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x24))
#define CAN1RDA 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x28))  	
#define CAN1RDB 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x2C))
  	
#define CAN1TFI1 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x30))  	
#define CAN1TID1 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x34))  	
#define CAN1TDA1 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x38))
#define CAN1TDB1 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x3C))  	
#define CAN1TFI2 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x40))  	
#define CAN1TID2 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x44))  	
#define CAN1TDA2 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x48))  	
#define CAN1TDB2 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x4C))
#define CAN1TFI3 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x50))  	
#define CAN1TID3 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x54))  	
#define CAN1TDA3 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x58))  	
#define CAN1TDB3 	(*(volatile unsigned long *)(CAN1_BASE_ADDR + 0x5C))

#define CAN2_BASE_ADDR		0xE0048000
#define CAN2MOD 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x00))  	
#define CAN2CMR 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x04))  	
#define CAN2GSR 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x08))  	
#define CAN2ICR 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x0C))  	
#define CAN2IER 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x10))
#define CAN2BTR 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x14))  	
#define CAN2EWL 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x18))  	
#define CAN2SR 		(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x1C))  	
#define CAN2RFS 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x20))  	
#define CAN2RID 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x24))
#define CAN2RDA 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x28))  	
#define CAN2RDB 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x2C))
  	
#define CAN2TFI1 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x30))  	
#define CAN2TID1 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x34))  	
#define CAN2TDA1 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x38))
#define CAN2TDB1 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x3C))  	
#define CAN2TFI2 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x40))  	
#define CAN2TID2 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x44))  	
#define CAN2TDA2 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x48))  	
#define CAN2TDB2 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x4C))
#define CAN2TFI3 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x50))  	
#define CAN2TID3 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x54))  	
#define CAN2TDA3 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x58))  	
#define CAN2TDB3 	(*(volatile unsigned long *)(CAN2_BASE_ADDR + 0x5C))


/* MultiMedia Card Interface(MCI) Controller */
#define MCI_BASE_ADDR		0xE008C000
#define MCI_POWER      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x00))
#define MCI_CLOCK      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x04))
#define MCI_ARGUMENT   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x08))
#define MCI_COMMAND    (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x0C))
#define MCI_RESP_CMD   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x10))
#define MCI_RESP0      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x14))
#define MCI_RESP1      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x18))
#define MCI_RESP2      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x1C))
#define MCI_RESP3      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x20))
#define MCI_DATA_TMR   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x24))
#define MCI_DATA_LEN   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x28))
#define MCI_DATA_CTRL  (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x2C))
#define MCI_DATA_CNT   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x30))
#define MCI_STATUS     (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x34))
#define MCI_CLEAR      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x38))
#define MCI_MASK0      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x3C))
#define MCI_MASK1      (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x40))
#define MCI_FIFO_CNT   (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x48))
#define MCI_FIFO       (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x80))


/* I2S Interface Controller (I2S) */
#define I2S_BASE_ADDR		0xE0088000
#define I2S_DAO        (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x00))
#define I2S_DAI        (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x04))
#define I2S_TX_FIFO    (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x08))
#define I2S_RX_FIFO    (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x0C))
#define I2S_STATE      (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x10))
#define I2S_DMA1       (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x14))
#define I2S_DMA2       (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x18))
#define I2S_IRQ        (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x1C))
#define I2S_TXRATE     (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x20))
#define I2S_RXRATE     (*(volatile unsigned long *)(I2S_BASE_ADDR + 0x24))


/* General-purpose DMA Controller */
#define DMA_BASE_ADDR		0xFFE04000
#define GPDMA_INT_STAT         (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x000))
#define GPDMA_INT_TCSTAT       (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x004))
#define GPDMA_INT_TCCLR        (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x008))
#define GPDMA_INT_ERR_STAT     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x00C))
#define GPDMA_INT_ERR_CLR      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x010))
#define GPDMA_RAW_INT_TCSTAT   (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x014))
#define GPDMA_RAW_INT_ERR_STAT (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x018))
#define GPDMA_ENABLED_CHNS     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x01C))
#define GPDMA_SOFT_BREQ        (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x020))
#define GPDMA_SOFT_SREQ        (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x024))
#define GPDMA_SOFT_LBREQ       (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x028))
#define GPDMA_SOFT_LSREQ       (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x02C))
#define GPDMA_CONFIG           (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x030))
#define GPDMA_SYNC             (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x034))

/* DMA channel 0 registers */
#define GPDMA_CH0_SRC      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x100))
#define GPDMA_CH0_DEST     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x104))
#define GPDMA_CH0_LLI      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x108))
#define GPDMA_CH0_CTRL     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x10C))
#define GPDMA_CH0_CFG      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x110))

/* DMA channel 1 registers */
#define GPDMA_CH1_SRC      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x120))
#define GPDMA_CH1_DEST     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x124))
#define GPDMA_CH1_LLI      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x128))
#define GPDMA_CH1_CTRL     (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x12C))
#define GPDMA_CH1_CFG      (*(volatile unsigned long *)(DMA_BASE_ADDR + 0x130))


/* USB Controller */
#define USB_INT_BASE_ADDR	0xE01FC1C0
#define USB_BASE_ADDR		0xFFE0C200		/* USB Base Address */

#define USB_INT_STAT    0xE01FC1C0

/* USB Device Interrupt Registers */
#define DEV_INT_STAT    0xFFE0C200
#define DEV_INT_EN      0xFFE0C204
#define DEV_INT_CLR     0xFFE0C208
#define DEV_INT_SET     0xFFE0C20C
#define DEV_INT_PRIO    0xFFE0C22C

/* USB Device Endpoint Interrupt Registers */
#define EP_INT_STAT     0xFFE0C230
#define EP_INT_EN       0xFFE0C234
#define EP_INT_CLR      0xFFE0C238
#define EP_INT_SET      0xFFE0C23C
#define EP_INT_PRIO     0xFFE0C240

/* USB Device Endpoint Realization Registers */
#define REALIZE_EP      0xFFE0C244
#define EP_INDEX        0xFFE0C248
#define MAXPACKET_SIZE  0xFFE0C24C

/* USB Device Command Reagisters */
#define CMD_CODE        0xFFE0C210
#define CMD_DATA        0xFFE0C214

/* USB Device Data Transfer Registers */
#define RX_DATA         0xFFE0C218
#define TX_DATA         0xFFE0C21C
#define RX_PLENGTH      0xFFE0C220
#define TX_PLENGTH      0xFFE0C224
#define USB_CTRL        0xFFE0C228

/* USB Device DMA Registers */
#define DMA_REQ_STAT        0xFFE0C250
#define DMA_REQ_CLR         0xFFE0C254
#define DMA_REQ_SET         0xFFE0C258
#define UDCA_HEAD           0xFFE0C280
#define EP_DMA_STAT         0xFFE0C284
#define EP_DMA_EN           0xFFE0C288
#define EP_DMA_DIS          0xFFE0C28C
#define DMA_INT_STAT        0xFFE0C290
#define DMA_INT_EN          0xFFE0C294
#define EOT_INT_STAT        0xFFE0C2A0
#define EOT_INT_CLR         0xFFE0C2A4
#define EOT_INT_SET         0xFFE0C2A8
#define NDD_REQ_INT_STAT    0xFFE0C2AC
#define NDD_REQ_INT_CLR     0xFFE0C2B0
#define NDD_REQ_INT_SET     0xFFE0C2B4
#define SYS_ERR_INT_STAT    0xFFE0C2B8
#define SYS_ERR_INT_CLR     0xFFE0C2BC
#define SYS_ERR_INT_SET     0xFFE0C2C0


/* USB Host Controller */
#define USBHC_BASE_ADDR		0xFFE0C000
#define HC_REVISION         0xFFE0C000
#define HC_CONTROL          0xFFE0C004
#define HC_CMD_STAT         0xFFE0C008
#define HC_INT_STAT         0xFFE0C00C
#define HC_INT_EN           0xFFE0C010
#define HC_INT_DIS          0xFFE0C014
#define HC_HCCA             0xFFE0C018
#define HC_PERIOD_CUR_ED    0xFFE0C01C
#define HC_CTRL_HEAD_ED     0xFFE0C020
#define HC_CTRL_CUR_ED      0xFFE0C024
#define HC_BULK_HEAD_ED     0xFFE0C028
#define HC_BULK_CUR_ED      0xFFE0C02C
#define HC_DONE_HEAD        0xFFE0C030
#define HC_FM_INTERVAL      0xFFE0C034
#define HC_FM_REMAINING     0xFFE0C038
#define HC_FM_NUMBER        0xFFE0C03C
#define HC_PERIOD_START     0xFFE0C040
#define HC_LS_THRHLD        0xFFE0C044
#define HC_RH_DESCA         0xFFE0C048
#define HC_RH_DESCB         0xFFE0C04C
#define HC_RH_STAT          0xFFE0C050
#define HC_RH_PORT_STAT1    0xFFE0C054
#define HC_RH_PORT_STAT2    0xFFE0C058

/* USB OTG Controller */
#define USBOTG_BASE_ADDR	0xFFE0C100
#define OTG_INT_STAT        0xFFE0C100
#define OTG_INT_EN          0xFFE0C104
#define OTG_INT_SET         0xFFE0C108
#define OTG_INT_CLR         0xFFE0C10C
#define OTG_STAT_CTRL       0xFFE0C110
#define OTG_TIMER           0xFFE0C114

#define USBOTG_I2C_BASE_ADDR	0xFFE0C300
#define OTG_I2C_RX          0xFFE0C300
#define OTG_I2C_TX          0xFFE0C300
#define OTG_I2C_STS         0xFFE0C304
#define OTG_I2C_CTL         0xFFE0C308
#define OTG_I2C_CLKHI       0xFFE0C30C
#define OTG_I2C_CLKLO       0xFFE0C310

#define USBOTG_CLK_BASE_ADDR	0xFFE0CFF0
#define OTG_CLK_CTRL        0xFFE0CFF4
#define OTG_CLK_STAT        0xFFE0CFF8


/* Ethernet MAC (32 bit data bus) -- all registers are RW unless indicated in parentheses */
#define MAC_BASE_ADDR		0xFFE00000 /* AHB Peripheral # 0 */
#define MAC_MAC1            0xFFE00000 /* MAC config reg 1 */
#define MAC_MAC2            0xFFE00004 /* MAC config reg 2 */
#define MAC_IPGT            0xFFE00008 /* b2b InterPacketGap reg */
#define MAC_IPGR            0xFFE0000C /* non b2b InterPacketGap reg */
#define MAC_CLRT            0xFFE00010 /* CoLlision window/ReTry reg */
#define MAC_MAXF            0xFFE00014 /* MAXimum Frame reg */
#define MAC_SUPP            0xFFE00018 /* PHY SUPPort reg */
#define MAC_TEST            0xFFE0001C /* TEST reg */
#define MAC_MCFG            0xFFE00020 /* MII Mgmt ConFiG reg */
#define MAC_MCMD            0xFFE00024 /* MII Mgmt CoMmanD reg */
#define MAC_MADR            0xFFE00028 /* MII Mgmt ADdRess reg */
#define MAC_MWTD            0xFFE0002C /* MII Mgmt WriTe Data reg (WO) */
#define MAC_MRDD            0xFFE00030 /* MII Mgmt ReaD Data reg (RO) */
#define MAC_MIND            0xFFE00034 /* MII Mgmt INDicators reg (RO) */

#define MAC_SA0             0xFFE00040 /* Station Address 0 reg */
#define MAC_SA1             0xFFE00044 /* Station Address 1 reg */
#define MAC_SA2             0xFFE00048 /* Station Address 2 reg */

#define MAC_COMMAND         0xFFE00100 /* Command reg */
#define MAC_STATUS          0xFFE00104 /* Status reg (RO) */
#define MAC_RXDESCRIPTOR    0xFFE00108 /* Rx descriptor base address reg */
#define MAC_RXSTATUS        0xFFE0010C /* Rx status base address reg */
#define MAC_RXDESCRIPTORNUM 0xFFE00110 /* Rx number of descriptors reg */
#define MAC_RXPRODUCEINDEX  0xFFE00114 /* Rx produce index reg (RO) */
#define MAC_RXCONSUMEINDEX  0xFFE00118 /* Rx consume index reg */
#define MAC_TXDESCRIPTOR    0xFFE0011C /* Tx descriptor base address reg */
#define MAC_TXSTATUS        0xFFE00120 /* Tx status base address reg */
#define MAC_TXDESCRIPTORNUM 0xFFE00124 /* Tx number of descriptors reg */
#define MAC_TXPRODUCEINDEX  0xFFE00128 /* Tx produce index reg */
#define MAC_TXCONSUMEINDEX  0xFFE0012C /* Tx consume index reg (RO) */

#define MAC_TSV0            0xFFE00158 /* Tx status vector 0 reg (RO) */
#define MAC_TSV1            0xFFE0015C /* Tx status vector 1 reg (RO) */
#define MAC_RSV             0xFFE00160 /* Rx status vector reg (RO) */

#define MAC_FLOWCONTROLCNT  0xFFE00170 /* Flow control counter reg */
#define MAC_FLOWCONTROLSTS  0xFFE00174 /* Flow control status reg */

#define MAC_RXFILTERCTRL    0xFFE00200 /* Rx filter ctrl reg */
#define MAC_RXFILTERWOLSTS  0xFFE00204 /* Rx filter WoL status reg (RO) */
#define MAC_RXFILTERWOLCLR  0xFFE00208 /* Rx filter WoL clear reg (WO) */

#define MAC_HASHFILTERL     0xFFE00210 /* Hash filter LSBs reg */
#define MAC_HASHFILTERH     0xFFE00214 /* Hash filter MSBs reg */

#define MAC_INTSTATUS       0xFFE00FE0 /* Interrupt status reg (RO) */
#define MAC_INTENABLE       0xFFE00FE4 /* Interrupt enable reg  */
#define MAC_INTCLEAR        0xFFE00FE8 /* Interrupt clear reg (WO) */
#define MAC_INTSET          0xFFE00FEC /* Interrupt set reg (WO) */

#define MAC_POWERDOWN       0xFFE00FF4 /* Power-down reg */
#define MAC_MODULEID        0xFFE00FFC /* Module ID reg (RO) */












/*#define PCLK	(CONFIG_ARM_CLK/(VPBDIV==0?4:VPBDIV))*/
#define PCLK	(57600000)
/* TO-DO EA */
//#define	LPC24xx_Fcclk	(CONFIG_LPC24xx_Fosc * 1) /* system clk frequecy,<=60Mhz*/
#define	LPC24xx_Fcclk	(57600000)  /* system clk frequecy,<=60Mhz*/
#define LPC24xx_Fcco	(288000000) /*CCO frequency,1,2,4,8 times of Fcclk*/ 
#define LPC24xx_Fpclk	(57600000)  /*VPB clk frequency,1,1/2,1/4 times of Fcclk */

#endif
/*********************************************************************************************************
**                            End Of File
********************************************************************************************************/
