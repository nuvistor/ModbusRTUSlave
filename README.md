# ModbusRTUSlave
Simple RTU slave implementation
Simple and lightweight code for STM32. Can be ported to other 32- or 8-bit platform.

How to use:
1. File mbrtuslave.h:
1.1. edit macro to configure supported functions(MB_DISCRETE_INPUTS_ENABLED ... MB_READ_INPUT_REGISTERS_ENABLED)
1.2. set starting adress and number of required registers(REG_HOLDING_START, REG_HOLDING_CNT) for each function
1.3. enable event flags(MB_COIL_WRITE_EVENTS_ENABLED) if you wish to check in application when registers are changed.
2. File mbrtuslave.c:
2.1. set baudrate for UART with macro MB_SERIAL_BAUD
2.2. port functions MB_TimerInit, MB_TimerRestart, MB_TimerStop, MB_SerialInit, USART1_IRQHandler, MB_Usart_PutByte, MB_SerialTX_Enable to your platform
2.3. set adress of your slave device with macro MB_SLAVE_ADRESS
3. In your application:
3.1. call MB_Init() first
3.2. in while(1) loop call MB_Poll() function to process modbus engine
3.3. write or read modbus registers when required
	// writing
	RegCoilsBuf[RUN_STOP_STATUS_COIL_REG] = 1; 
	RegHoldingBuf[PROD_CNT_L_HOLD_REG] = ProductCounter & 0xFFFF;
	RegHoldingBuf[PROD_CNT_H_HOLD_REG] = ProductCounter >> 16;
	MB_PutHolding32LH( ABS_SERVO_CUR_PULS_POSL_HOLD_REG, actualServoPosPulseCnt );  
	// read
	Servo_ppr = RegHoldingBuf[SERVO_CAL_PPR10L_HOLD_REG];
