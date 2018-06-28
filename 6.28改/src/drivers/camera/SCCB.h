#ifndef __SCCB_H
#define __SCCB_H


#define SCL_H()         PTC17_OUT = 1
#define SCL_L()         PTC17_OUT = 0
#define	SCL_DDR_OUT() 	DDRC17 = 1
#define	SCL_DDR_IN() 	DDRC17 = 0

#define SDA_H()         PTC19_OUT = 1
#define SDA_L()         PTC19_OUT = 0
#define SDA_IN()      	PTC19_IN
#define SDA_DDR_OUT()	DDRC19 = 1
#define SDA_DDR_IN()	DDRC19 = 0

#define ADDR_OV7725   0x42

#define SCCB_DELAY()	SCCB_delay(100)	


void SCCB_GPIO_init(void);
int SCCB_WriteByte( u16 WriteAddress , u8 SendByte);
int SCCB_ReadByte(u8* pBuffer,   u16 length,   u8 ReadAddress);
static void SCCB_delay(u16 i);


#endif 
