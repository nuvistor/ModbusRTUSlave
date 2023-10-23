/*-----------------------------------------------------------------------
Modbus RTU Slave Functions
mbrtuslave.c
------------------------------------------------------------------------*/
#include <stdint.h>
#include "mbrtucrc16.h"
#include "main.h"
#include <string.h>
#include "mbrtuslave.h"


/*-----------------------------------------------------------------------
Serial Frame
------------------------------------------------------------------------*/

#define MB_RTU_MAX_FRAME_SZ	256 // bytes
/*-----------------------------------------------------------------------
PDU Frame: [Adress=1byte][Function=1byte][Data=0-252bytes][CRC16=2bytes]
------------------------------------------------------------------------*/
uint8_t  MB_RX_Buf[MB_RTU_MAX_FRAME_SZ];
uint16_t MB_RX_ByteCnt=0;

uint8_t  MB_TX_Buf[MB_RTU_MAX_FRAME_SZ];
uint16_t MB_TX_ByteCnt=0;
uint8_t *pTX_Buf;

enum{
	MB_STA_NOINIT=0,
	MB_STA_IDLE, // wait for end of Rx frame
	MB_STA_RECEIVING, // receiving in progress
	MB_STA_RECEIVING_FRAME_ERR, // broken frame
	MB_STA_END_OF_RX_FRAME, // request for processing of new frame
	MB_STA_TRANSMITTING, // transmitting in progress
	MB_STA_END_OF_TX_FRAME, // end of transmitting process
}MB_State = MB_STA_NOINIT;

#define MB_RX_MIN_FRAME_SZ	4 // adr + func + [data] + crchi + crclo

#define MB_SERIAL_BAUD				19200//9600
#define MB_SERIAL_FRAME_BITS	10 // 1 start + 8 data + 1 stop


/*-----------------------------------------------------------------------
------------------------------------------------------------------------*/
void MB_SerialTX_Enable( void );
void MB_SerialTX_Disable( void );
void MB_SerialRX_Enable( void );
void MB_SerialRX_Disable( void );


/*-----------------------------------------------------------------------
Modbus Timer
------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------
------------------------------------------------------------------------*/
void MB_TimerInit( uint16_t usTim1Timerout50us )
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; // tim16 clock enable
	TIM16->CR1 = 0; // tim16 stop
	TIM16->CNT = 0;
	TIM16->ARR = usTim1Timerout50us;
	TIM16->PSC = F_CPU/20000; // 20KHz = 50uS tick
	TIM16->DIER = 0; // timer irq disabled
	TIM16->EGR = TIM_EGR_UG; // update ARR
	TIM16->SR = 0; // clear flags
	TIM16->DIER = TIM_DIER_UIE; // update irq enable
	//
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn); // enable NVIC irq channel for timer
	//
}

/*-----------------------------------------------------------------------
Enable Timer
------------------------------------------------------------------------*/
void MB_TimerRestart( void )
{
	TIM16->CR1 = 0; // tim16 stop
	TIM16->SR = 0; // clear flags
	TIM16->CNT = 0;
	TIM16->CR1 |= TIM_CR1_CEN; // tim16 start
}

/*-----------------------------------------------------------------------
Disable Timer
------------------------------------------------------------------------*/
void MB_TimerStop( void )
{
	TIM16->CR1 = 0; // tim16 stop
	TIM16->SR = 0; // clear flags
}

/*-----------------------------------------------------------------------
Timer Interrupt Handler
------------------------------------------------------------------------*/
void MB_TimerHandler( void )
{
	switch( MB_State )
	{
		case MB_STA_RECEIVING:
			if( MB_RX_ByteCnt >= MB_RX_MIN_FRAME_SZ )
			{
				MB_SerialRX_Disable();
				MB_State = MB_STA_END_OF_RX_FRAME;
			}else // frame error
			{
				MB_RX_ByteCnt = 0;
				MB_State = MB_STA_IDLE;
			};
			break;
		case MB_STA_RECEIVING_FRAME_ERR: 			// RX frame is broken
			MB_RX_ByteCnt = 0;
			MB_State = MB_STA_IDLE;
			break;
		case MB_STA_TRANSMITTING: // end of transmission
			MB_State = MB_STA_IDLE;
			//MB_SerialTX_Disable();
			MB_SerialRX_Enable();
			break;
		default:
			break;
	};
	
	MB_TimerStop();
}

/*-----------------------------------------------------------------------
------------------------------------------------------------------------*/
void TIM1_UP_TIM16_IRQHandler( void )
{
		if( TIM16->SR & TIM_SR_UIF )
		{
			TIM16->SR &= ~TIM_SR_UIF;
			MB_TimerHandler();
		};
}

/*-----------------------------------------------------------------------
Modbus Serial Port
------------------------------------------------------------------------*/
void MB_RX_ByteHandler( uint8_t data );
void MB_TX_ByteHandler( void );

/*-----------------------------------------------------------------------
Init Serial Port
------------------------------------------------------------------------*/
void MB_SerialInit( void )
{
	uint16_t udiv_int;
	
	MB_TX_ByteCnt=0;
	
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
	USART1->CR1 = 0; // disable usart1
	udiv_int = F_CPU/(MB_SERIAL_BAUD*16);
	USART1->BRR = (udiv_int<<4);
	USART1->SR = 0;
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; // enable rx, tx, uart
	NVIC_EnableIRQ(USART1_IRQn);
}

/*-----------------------------------------------------------------------
------------------------------------------------------------------------*/
void USART1_IRQHandler( void )
{
	if( USART1->SR & USART_SR_TXE )
	{
		USART1->SR = ~USART_SR_TXE;
		//
		MB_TX_ByteHandler();
	};
	//
	if( USART1->SR & USART_SR_RXNE )
	{
		USART1->SR = ~ USART_SR_RXNE;
		//
		MB_RX_ByteHandler( USART1->DR );
	};
}

/*-----------------------------------------------------------------------
Put data into transmit register
------------------------------------------------------------------------*/
void MB_Usart_PutByte( uint8_t d )
{
		USART1->DR = d;
}

/*-----------------------------------------------------------------------
Usart RX Not Empty
------------------------------------------------------------------------*/
void MB_RX_ByteHandler( uint8_t data )
{
		if( MB_RX_ByteCnt < MB_RTU_MAX_FRAME_SZ )
		{
			MB_RX_Buf[ MB_RX_ByteCnt++ ] = data;
			MB_State = MB_STA_RECEIVING;
		}else
		{
			MB_State = MB_STA_RECEIVING_FRAME_ERR;
		};
		MB_TimerRestart();
}

/*-----------------------------------------------------------------------
Usart TX Complete/data register empty
------------------------------------------------------------------------*/
void MB_TX_ByteHandler( void )
{
	if( MB_TX_ByteCnt > 0 )
	{
		MB_Usart_PutByte(*pTX_Buf++); // put data into TX register
		MB_TX_ByteCnt--;
		MB_State = MB_STA_TRANSMITTING;
	}else
	{
			MB_TimerRestart();
			MB_SerialTX_Disable();
	};
}

/*-----------------------------------------------------------------------
RS485 RX-TX drive pin
------------------------------------------------------------------------*/
void MB_SerialTX_Enable( void )
{
	USART1->SR = ~USART_SR_TXE; // clear TXE request
	USART1->CR1 |= USART_CR1_TXEIE; // tx buf empty irq enable
}

void MB_SerialTX_Disable( void )
{
	USART1->CR1 &= ~ USART_CR1_TXEIE; // tx buf empty irq disable
	USART1->SR = ~USART_SR_TXE;
}

void MB_SerialRX_Enable( void )
{
	USART1->SR = ~USART_SR_RXNE; // clear RXNE request
	USART1->CR1 |= USART_CR1_RXNEIE; // rx not empty irq enable
}

void MB_SerialRX_Disable( void )
{
	USART1->CR1 &= ~USART_CR1_RXNEIE; // rx not empty irq disable
	USART1->SR = ~USART_SR_RXNE; // clear RXNE request
}


/*-----------------------------------------------------------------------
------------------------------------------------------------------------*/
#define MB_BROADCAST_ADRESS	0
#define MB_SLAVE_ADRESS	10

/*-----------------------------------------------------------------------
Registers definition
------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------
Holding Registers
------------------------------------------------------------------------*/
#if (MB_HOLDING_REGISTERS_ENABLED>0)
uint16_t RegHoldingBuf[ REG_HOLDING_CNT ];

#if (MB_HOLDING_WRITE_EVENTS_ENABLED>0)
uint8_t MB_HoldingWriteEvent = 0;
#endif /* MB_HOLDING_WRITE_EVENTS_ENABLED */	
#endif /* MB_HOLDING_REGISTERS_ENABLED */

/*-----------------------------------------------------------------------
Discrete inputs
------------------------------------------------------------------------*/
#if (MB_DISCRETE_INPUTS_ENABLED>0)
uint8_t RegDiscreteBuf[ REG_DISCRETE_CNT ]; // 1 bit for each array element
#endif

/*-----------------------------------------------------------------------
Input registers (16 bit)
------------------------------------------------------------------------*/
#if (MB_READ_INPUT_REGISTERS_ENABLED>0)
uint16_t RegInputBuf[ REG_INPUT_CNT ];
#endif

/*-----------------------------------------------------------------------
Coil registers
------------------------------------------------------------------------*/
#if (MB_COIL_REGISTERS_ENABLED>0)
uint8_t RegCoilsBuf[ REG_COILS_CNT ]; // 1 bit for each byte in array

#if (MB_COIL_WRITE_EVENTS_ENABLED>0)
uint8_t MB_CoilWriteEvent = 0;
#endif /* MB_COIL_WRITE_EVENTS_ENABLED */
#endif /* MB_COIL_REGISTERS_ENABLED */

// Discrete IO
#define MB_FUNC_READ_DISCRETE_INPUTS			0x02
#define MB_FUNC_READ_COILS								0x01 // read discrete outputs
#define MB_FUNC_WRITE_SINGLE_COIL					0x05
#define MB_FUNC_WRITE_MULTIPLE_COILS			0x0F
// Input registers
#define MB_FUNC_READ_INPUT_REGISTER				0x04
// Holding Registers
#define MB_FUNC_READ_HOLDING_REGISTERS		0x03
#define MB_FUNC_WRITE_SINGLE_REGISTER			0x06
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS	0x10


typedef enum{
	MB_NO_EXCEPTION=0x00,
	MB_EXC_ILLEGAL_FUNC=0x01,
	MB_EXC_ILLEGAL_DATA_ADR=0x02,
	MB_EXC_ILLEGAL_DATA_VALUE=0x03,
	MB_EXC_SLAVE_FAILURE=0x04,
	MB_EXC_ACKNOWLEDGE=0x05,
	MB_EXC_SALVE_BUSY=0x06	
}MB_ExceptionCode;

/*-----------------------------------------------------------------------
Init modbus 
------------------------------------------------------------------------*/
void MB_Init( void )
{
	MB_SerialInit();
	if( MB_SERIAL_BAUD > 19200 ) // fixed baud 1750uS -> ~1800uS
	{
		MB_TimerInit( 1800/50 );
	}else
	{
		MB_TimerInit( (20000UL * MB_SERIAL_FRAME_BITS * 3.5f) / MB_SERIAL_BAUD );
	};
	MB_State = MB_STA_IDLE;
	MB_SerialRX_Enable();
}

/*-----------------------------------------------------------------------
------------------------------------------------------------------------*/
MB_ExceptionCode MB_ReadDiscreteInputsOrCoils( uint8_t func, uint16_t RegStart, uint16_t RegsTotalCnt, uint8_t *Regs );
MB_ExceptionCode MB_WriteSingleCoil( void );
MB_ExceptionCode MB_WriteMultipleCoils( void );
MB_ExceptionCode MB_ReadHoldingOrInputRegs( uint8_t func, uint16_t RegStart, uint16_t RegsTotalCnt, uint16_t *Regs );
MB_ExceptionCode MB_WriteSingleHoldingReg( void );
MB_ExceptionCode MB_WriteMultipleHoldingRegs( void );

/*-----------------------------------------------------------------------
Function is called from application to process modbus states
------------------------------------------------------------------------*/
void MB_Poll( void )
{
	uint8_t sla_adr; // slave adress
	uint8_t func_code;
	uint16_t crc;
	MB_ExceptionCode ErrCode = MB_NO_EXCEPTION;
	
	switch( MB_State )
	{
		case MB_STA_END_OF_RX_FRAME:
			MB_TX_ByteCnt = 0;
			sla_adr = MB_RX_Buf[0];
			if( ((sla_adr == MB_SLAVE_ADRESS) || (sla_adr == MB_BROADCAST_ADRESS)) && (MBCRC16(MB_RX_Buf, MB_RX_ByteCnt) == 0) ) // crc valid ?
			{
					
				func_code = MB_RX_Buf[1];
				switch(func_code)
				{
						#if (MB_DISCRETE_INPUTS_ENABLED>0)
						case MB_FUNC_READ_DISCRETE_INPUTS:
							if( sla_adr != MB_BROADCAST_ADRESS )
							{
								ErrCode = MB_ReadDiscreteInputsOrCoils( MB_FUNC_READ_DISCRETE_INPUTS, REG_DISCRETE_START, REG_DISCRETE_CNT, RegDiscreteBuf );	
							};
							break;
						#endif
							
						#if (MB_COIL_REGISTERS_ENABLED>0)	
						case MB_FUNC_READ_COILS:
							if( sla_adr != MB_BROADCAST_ADRESS )
							{
								ErrCode = MB_ReadDiscreteInputsOrCoils( MB_FUNC_READ_COILS, REG_COILS_START, REG_COILS_CNT, RegCoilsBuf );
							};
							break;
						case MB_FUNC_WRITE_SINGLE_COIL:
							ErrCode = MB_WriteSingleCoil();
							break;
						case MB_FUNC_WRITE_MULTIPLE_COILS:
							ErrCode = MB_WriteMultipleCoils();
							break;
						#endif
						
						#if(MB_READ_INPUT_REGISTERS_ENABLED>0)
						case MB_FUNC_READ_INPUT_REGISTER:
							if( sla_adr != MB_BROADCAST_ADRESS )
							{
								ErrCode = MB_ReadHoldingOrInputRegs( MB_FUNC_READ_INPUT_REGISTER, REG_INPUT_START, REG_INPUT_CNT, RegInputBuf );
							};
							break;
						#endif
							
						#if (MB_HOLDING_REGISTERS_ENABLED>0)	
						case MB_FUNC_READ_HOLDING_REGISTERS:
							if( sla_adr != MB_BROADCAST_ADRESS )
							{
								ErrCode = MB_ReadHoldingOrInputRegs( MB_FUNC_READ_HOLDING_REGISTERS, REG_HOLDING_START, REG_HOLDING_CNT, RegHoldingBuf );
							};
							break;
						case MB_FUNC_WRITE_SINGLE_REGISTER:
							ErrCode = MB_WriteSingleHoldingReg();
							break;
						case MB_FUNC_WRITE_MULTIPLE_REGISTERS:
							ErrCode = MB_WriteMultipleHoldingRegs();
							break;
						#endif
						
						default:
							ErrCode = MB_EXC_ILLEGAL_FUNC;
							break;
				};
				//
				MB_RX_ByteCnt = 0;
				// if it is not broadcast request then need to do response
				if( sla_adr != MB_BROADCAST_ADRESS )
				{
					pTX_Buf = &MB_TX_Buf[0];
					if( ErrCode == MB_NO_EXCEPTION )
					{
						if( MB_TX_ByteCnt > 0 )
						{
							MB_SerialTX_Enable();
						};
					}else
					{
						MB_TX_Buf[0] = MB_RX_Buf[0]; // slave adress
						MB_TX_Buf[1] = MB_RX_Buf[1] | 0x80; // exception marker
						MB_TX_Buf[2] = ErrCode;
						crc = MBCRC16( MB_TX_Buf, 3 );
						MB_TX_Buf[3] = crc & 0xFF;
						MB_TX_Buf[4] = crc >> 8;
						MB_TX_ByteCnt = 5;
						MB_SerialTX_Enable();
					};
				};
			}else // crc or adress mismatch
			{
				MB_State = MB_STA_IDLE;
				MB_RX_ByteCnt = 0;
				MB_SerialRX_Enable();
			};
			break;
		default:
			break;
	}
}

/*-----------------------------------------------------------------------
Read 1 bit Discrete Inputs
Or
Read 1 bit Coils Status
------------------------------------------------------------------------*/
#if (MB_DISCRETE_INPUTS_ENABLED>0) || (MB_COIL_REGISTERS_ENABLED>0)
MB_ExceptionCode
MB_ReadDiscreteInputsOrCoils( uint8_t func, uint16_t RegStart, uint16_t RegsTotalCnt, uint8_t *Regs )
{
	MB_ExceptionCode ErrCode = MB_NO_EXCEPTION;
	uint16_t regAdr, regCnt, i, crc;
	uint8_t  bitCnt;
	uint8_t *pBuf;
	
	if( MB_RX_ByteCnt < 8 )
		return MB_EXC_ILLEGAL_DATA_VALUE;
	
	regAdr  = MB_RX_Buf[2] << 8U;
	regAdr |= MB_RX_Buf[3];
	
	regCnt	= MB_RX_Buf[4] << 8U;
	regCnt |= MB_RX_Buf[5];
	
	if( (regAdr >= RegStart) && (regCnt <= RegsTotalCnt ) && (regCnt > 0) )
	{
		MB_TX_Buf[0] = MB_SLAVE_ADRESS;
		MB_TX_Buf[1] = func;
		pBuf = &MB_TX_Buf[3];
		*pBuf = 0;
		bitCnt = 0;
		MB_TX_ByteCnt = 1;
		for( i=regAdr-RegStart; regCnt>0; i++ )
		{
				if( bitCnt >= 8 )
				{
					bitCnt = 0;
					MB_TX_ByteCnt++;
					pBuf++;
					*pBuf = 0;
				};
			
				if( Regs[i] ) // ON ?
				{
					*pBuf |= 1<<bitCnt;
				};
				
				bitCnt++;
				regCnt--;
		};
		pBuf++;
		MB_TX_Buf[2] = MB_TX_ByteCnt;
		MB_TX_ByteCnt += 3;
		crc = MBCRC16( MB_TX_Buf, MB_TX_ByteCnt );
		*pBuf++ = crc & 0xFF;
		*pBuf++ = crc >> 8;
		MB_TX_ByteCnt += 2;
		
	}else
	{
		ErrCode = MB_EXC_ILLEGAL_DATA_ADR;
	};
	
	return ErrCode;
}
#endif

/*-----------------------------------------------------------------------
Write Single 1 bit Coil
------------------------------------------------------------------------*/
#if (MB_COIL_REGISTERS_ENABLED>0)
MB_ExceptionCode
MB_WriteSingleCoil( void )
{
	MB_ExceptionCode ErrCode = MB_NO_EXCEPTION;
	uint16_t regAdr, coilValue;
	
	if( MB_RX_ByteCnt < 8 )
		return MB_EXC_ILLEGAL_DATA_VALUE;
	
	regAdr  = MB_RX_Buf[2] << 8U; // Adr Hi
	regAdr |= MB_RX_Buf[3]; // Adr Lo
	
	coilValue  = MB_RX_Buf[4] << 8U;
	coilValue |= MB_RX_Buf[5];
	
	if( (regAdr >= REG_COILS_START) && (regAdr <= (REG_COILS_START+REG_COILS_CNT) ) )
	{
		if( (coilValue!=0x0000) && (coilValue!=0xFF00) )
			return MB_EXC_ILLEGAL_DATA_VALUE;
		memcpy( MB_TX_Buf, MB_RX_Buf, MB_RX_ByteCnt ); 
		MB_TX_ByteCnt = MB_RX_ByteCnt;
		
		//
#if (MB_COIL_WRITE_EVENTS_ENABLED>0)
		MB_CoilWriteEvent = 1;
#endif // MB_COIL_WRITE_EVENTS_ENABLED 
		//
		
		if( coilValue )
		{
				RegCoilsBuf[regAdr-REG_COILS_START] = 1;
		}else
		{
				RegCoilsBuf[regAdr-REG_COILS_START] = 0;
		};
		
	}else
	{
		ErrCode = MB_EXC_ILLEGAL_DATA_ADR;
	};
	
	return ErrCode;	
}
#endif

/*-----------------------------------------------------------------------
Write Multiple 1 bit Coils
------------------------------------------------------------------------*/
#if (MB_COIL_REGISTERS_ENABLED>0)
MB_ExceptionCode
MB_WriteMultipleCoils( void )
{
	MB_ExceptionCode ErrCode = MB_NO_EXCEPTION;
	uint16_t regAdr, coilCnt, i, crc;
	uint8_t  bitCnt;
	uint8_t *pBuf;
	
	if( MB_RX_ByteCnt < 11 )
		return MB_EXC_ILLEGAL_DATA_VALUE;
	
	regAdr  = MB_RX_Buf[2] << 8U;
	regAdr |= MB_RX_Buf[3];
	
	coilCnt	 = MB_RX_Buf[4] << 8U;
	coilCnt |= MB_RX_Buf[5];
	
	//byteCnt = MB_RX_Buf[6];
	
	if( (regAdr >= REG_COILS_START) && (coilCnt <= REG_COILS_CNT ) && (coilCnt > 0) )
	{
		//
		memcpy( MB_TX_Buf, MB_RX_Buf, 6 );
		crc = MBCRC16( MB_TX_Buf, 6 );
		MB_TX_Buf[6] = crc & 0xFF; // Lo
		MB_TX_Buf[7] = crc >> 8;   // Hi
		MB_TX_ByteCnt = 8;
		//
#if (MB_COIL_WRITE_EVENTS_ENABLED>0)
		MB_CoilWriteEvent = 1;
#endif // MB_COIL_WRITE_EVENTS_ENABLED 
		//
		pBuf = &MB_RX_Buf[7];
		bitCnt = 0;

		for( i=regAdr-REG_COILS_START; i<coilCnt; i++ )
		{
				if( bitCnt >= 8 )
				{
					bitCnt = 0;
					pBuf++;
				};
			
				RegCoilsBuf[i] = (*pBuf >> bitCnt) & 0x01;
				
				bitCnt++;
		};		
	}else
	{
		ErrCode = MB_EXC_ILLEGAL_DATA_ADR;
	};
	
	return ErrCode;
}
#endif

/*-----------------------------------------------------------------------
Read 16 bit Holding Registers
Or
Read 16 bit Input Registers
------------------------------------------------------------------------*/
#if (MB_HOLDING_REGISTERS_ENABLED>0) || (MB_READ_INPUT_REGISTERS_ENABLED>0)
MB_ExceptionCode
MB_ReadHoldingOrInputRegs( uint8_t func, uint16_t RegStart, uint16_t RegsTotalCnt, uint16_t *Regs )
{
	MB_ExceptionCode ErrCode = MB_NO_EXCEPTION;
	uint16_t regAdr, regCnt, i, crc;
	uint8_t *pBuf;
	//uint8_t byteCnt = 0;
	
	if( MB_RX_ByteCnt < 8 )
		return MB_EXC_ILLEGAL_DATA_VALUE;
	
	regAdr  = MB_RX_Buf[2] << 8U;
	regAdr |= MB_RX_Buf[3];
	
	regCnt	= MB_RX_Buf[4] << 8U;
	regCnt |= MB_RX_Buf[5];
	
	if( (regAdr >= RegStart) && (regCnt <= RegsTotalCnt ) && (regCnt > 0) )
	{
		MB_TX_ByteCnt = 0;
		MB_TX_Buf[0] = MB_SLAVE_ADRESS;
		MB_TX_Buf[1] = func;
		pBuf = &MB_TX_Buf[3];
		i=regAdr-RegStart;
		while( (regCnt--) > 0 )
		//for( i=regAdr-RegStart; i<regCnt; i++ )
		{
				*pBuf++ = Regs[i] >> 8; // Hi
				*pBuf++ = Regs[i++] & 0xFF; // Lo
				MB_TX_ByteCnt += 2;		
		};
		MB_TX_Buf[2] = MB_TX_ByteCnt;
		MB_TX_ByteCnt += 3;
		crc = MBCRC16( MB_TX_Buf, MB_TX_ByteCnt );
		*pBuf++ = crc & 0xFF;
		*pBuf++ = crc >> 8;
		MB_TX_ByteCnt += 2;
		
	}else
	{
		ErrCode = MB_EXC_ILLEGAL_DATA_ADR;
	};
	
	return ErrCode;	
}
#endif

/*-----------------------------------------------------------------------
Write 16 bit Single Holding Register
------------------------------------------------------------------------*/
#if (MB_HOLDING_REGISTERS_ENABLED>0)
MB_ExceptionCode
MB_WriteSingleHoldingReg( void )
{
	MB_ExceptionCode ErrCode = MB_NO_EXCEPTION;
	uint16_t regAdr, regVal;
	
	if( MB_RX_ByteCnt < 8 )
		return MB_EXC_ILLEGAL_DATA_VALUE;
	
	regAdr  = MB_RX_Buf[2] << 8U; // Adr Hi
	regAdr |= MB_RX_Buf[3]; // Adr Lo
	
	regVal  = MB_RX_Buf[4] << 8U; // Val Hi
	regVal |= MB_RX_Buf[5]; // Val Lo
		
	if( (regAdr >= REG_HOLDING_START) && (regAdr <= (REG_HOLDING_START+REG_HOLDING_CNT) ) )
	{
		memcpy( MB_TX_Buf, MB_RX_Buf, MB_RX_ByteCnt ); 
		MB_TX_ByteCnt = MB_RX_ByteCnt;
		
		//
#if (MB_HOLDING_WRITE_EVENTS_ENABLED>0)
		MB_HoldingWriteEvent = 1;
#endif // MB_HOLDING_WRITE_EVENTS_ENABLED
		//
		
		RegHoldingBuf[regAdr-REG_HOLDING_START] = regVal;
		
	}else
	{
		ErrCode = MB_EXC_ILLEGAL_DATA_ADR;
	};
	
	return ErrCode;	
}
#endif

/*-----------------------------------------------------------------------
Write Multiple 16 bit Holding registers
------------------------------------------------------------------------*/
#if (MB_HOLDING_REGISTERS_ENABLED>0)
MB_ExceptionCode
MB_WriteMultipleHoldingRegs( void )
{
	MB_ExceptionCode ErrCode = MB_NO_EXCEPTION;
	uint16_t regAdr, regCnt, i, crc;
	uint8_t *pBuf;
	uint8_t byteCnt;
	
	if( MB_RX_ByteCnt < 8 )
		return MB_EXC_ILLEGAL_DATA_VALUE;
	
	regAdr  = MB_RX_Buf[2] << 8U;
	regAdr |= MB_RX_Buf[3];
	
	regCnt	= MB_RX_Buf[4] << 8U;
	regCnt |= MB_RX_Buf[5];
	
	byteCnt = MB_RX_Buf[6];
	
	if( (regAdr >= REG_HOLDING_START) && (regCnt <= REG_HOLDING_CNT ) )
	{
		if( (byteCnt!=regCnt*2) || (byteCnt==0) || (regCnt == 0) )
			return MB_EXC_ILLEGAL_DATA_VALUE;

		pBuf = &MB_RX_Buf[7];
		regCnt += regAdr-REG_HOLDING_START;
		for( i=regAdr-REG_HOLDING_START; i<regCnt; i++ )
		{
				RegHoldingBuf[i] =  (*pBuf++) << 8U;  // Hi;
				RegHoldingBuf[i] |= (*pBuf++) & 0xFF; // Lo	
		};
		
		memcpy( MB_TX_Buf, MB_RX_Buf, 6 );
		crc = MBCRC16( MB_TX_Buf, 6 );
		MB_TX_Buf[6] = crc & 0xFF; // Lo
		MB_TX_Buf[7] = crc >> 8;   // Hi
		MB_TX_ByteCnt = 8;
		
		//
#if (MB_HOLDING_WRITE_EVENTS_ENABLED>0)
		MB_HoldingWriteEvent = 1;
#endif // MB_HOLDING_WRITE_EVENTS_ENABLED
		//
	}else
	{
		ErrCode = MB_EXC_ILLEGAL_DATA_ADR;
	};
	
	return ErrCode;	
}
#endif

/*-----------------------------------------------------------------------
Put 32 bit value into multiple registers
Register order: Low-High
------------------------------------------------------------------------*/
void MB_PutHolding32LH( uint16_t adr, uint32_t data )
{
	RegHoldingBuf[ adr ]   = (uint16_t) data;
	RegHoldingBuf[ adr+1 ] = data>>16;
}

/*-----------------------------------------------------------------------
Put 32 bit value into multiple registers
Register order: High-Low
------------------------------------------------------------------------*/
void MB_PutHolding32HL( uint16_t adr, uint32_t data )
{
	RegHoldingBuf[ adr ]   = data>>16;
	RegHoldingBuf[ adr+1 ] = (uint16_t) data;
}

/*-----------------------------------------------------------------------
Get 32 bit value, Low-High register order
------------------------------------------------------------------------*/
uint32_t MB_GetHolding32LH( uint16_t adr )
{
	uint32_t d;
	
	d = RegHoldingBuf[ adr ];
	d|= ((uint32_t)RegHoldingBuf[ adr+1 ]) << 16;
	
	return d;
}

/*-----------------------------------------------------------------------
Get 32 bit value, High-Low register order
------------------------------------------------------------------------*/
uint32_t MB_GetHolding32HL( uint16_t adr )
{
	uint32_t d;

	d = ((uint32_t)RegHoldingBuf[ adr ]) << 16;	
	d|= RegHoldingBuf[ adr+1 ];

	return d;
}








