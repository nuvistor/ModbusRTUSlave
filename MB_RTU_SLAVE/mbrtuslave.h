/*-----------------------------------------------------------------------
Modbus RTU Slave
mbrtuslave.h
------------------------------------------------------------------------*/

#ifndef _MBRTUSLAVE_H
#define _MBRTUSLAVE_H

void MB_Init( void );
void MB_Poll( void );

// manipulations with 32 bit data:
void MB_PutHolding32LH( uint16_t adr, uint32_t data );
void MB_PutHolding32HL( uint16_t adr, uint32_t data );
uint32_t MB_GetHolding32LH( uint16_t adr );
uint32_t MB_GetHolding32HL( uint16_t adr );

/*-----------------------------------------------------------------------
Configuration of Modbus Functions
------------------------------------------------------------------------*/
#define MB_DISCRETE_INPUTS_ENABLED			(0)
#define MB_COIL_REGISTERS_ENABLED				(1)
#define MB_HOLDING_REGISTERS_ENABLED		(1)
#define MB_READ_INPUT_REGISTERS_ENABLED	(0)

// Modbus Events
#define MB_COIL_WRITE_EVENTS_ENABLED		(1)
#define MB_HOLDING_WRITE_EVENTS_ENABLED	(0)

/*-----------------------------------------------------------------------
Registers definition
------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------
Holding Registers
------------------------------------------------------------------------*/

#define REG_HOLDING_START	1000
#define REG_HOLDING_CNT		30

#if (MB_HOLDING_REGISTERS_ENABLED>0)
extern 
uint16_t RegHoldingBuf[ REG_HOLDING_CNT ];

#if (MB_HOLDING_WRITE_EVENTS_ENABLED>0)
extern
uint8_t MB_HoldingWriteEvent;
#endif /* MB_HOLDING_WRITE_EVENTS_ENABLED */

#endif /* MB_HOLDING_REGISTERS_ENABLED */

/*-----------------------------------------------------------------------
Discrete inputs
------------------------------------------------------------------------*/
#define REG_DISCRETE_START	1000
#define REG_DISCRETE_CNT		10

#if (MB_DISCRETE_INPUTS_ENABLED>0)
extern 
uint8_t RegDiscreteBuf[ REG_DISCRETE_CNT ]; // 1 bit for each array element
#endif

/*-----------------------------------------------------------------------
Input registers (16 bit)
------------------------------------------------------------------------*/
#define REG_INPUT_START	1000
#define REG_INPUT_CNT		10

#if (MB_READ_INPUT_REGISTERS_ENABLED>0)
extern 
uint16_t RegInputBuf[ REG_INPUT_CNT ];
#endif

/*-----------------------------------------------------------------------
Coil registers
------------------------------------------------------------------------*/
#define REG_COILS_START	1000
#define REG_COILS_CNT		40

#if (MB_COIL_REGISTERS_ENABLED>0)
extern 
uint8_t RegCoilsBuf[ REG_COILS_CNT ]; // 1 bit for each byte in array

#if (MB_COIL_WRITE_EVENTS_ENABLED>0)
extern
uint8_t MB_CoilWriteEvent;
#endif /* MB_COIL_WRITE_EVENTS_ENABLED */

#endif /* MB_COIL_REGISTERS_ENABLED */


#endif
