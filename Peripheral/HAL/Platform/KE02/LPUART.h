static inline void LPUART0_WriteChar(uint8_t data)
{
    *(volatile uint8_t *)(&(LPUART0->DATA)) = data; // This register is two separate registers. Reads return the contents of the read-only receive data buffer and writes go to the write-only transmit data buffer
}

static inline void LPUART0_ReadChar(uint8_t * readData)
{
    *readData = (uint8_t)(LPUART0->DATA); // This register is two separate registers. Reads return the contents of the read-only receive data buffer and writes go to the write-only transmit data buffer
}

static inline void LPUART0_SetTransmitterCmd(bool enable)
{
	LPUART0->CTRL = (LPUART0->CTRL & ~LPUART_CTRL_TE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_TE_SHIFT);
    /* Wait for the register write operation to complete */
    //while((bool)((base->CTRL & LPUART_CTRL_TE_MASK) != 0U) != enable) {}
}

static inline void LPUART0_SetReceiverCmd(bool enable)
{
	LPUART0->CTRL = (LPUART0->CTRL & ~LPUART_CTRL_RE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_RE_SHIFT);
    /* Wait for the register write operation to complete */
    //while((bool)((base->CTRL & LPUART_CTRL_RE_MASK) != 0U) != enable) {}
}

static inline void LPUART0_EnableTxDataISR(void)
{
	LPUART0_SetTransmitterCmd(true);
	LPUART0->CTRL = (LPUART0->CTRL | LPUART_CTRL_TIE_MASK);
}

static inline void LPUART0_DisableTxDataISR(void)
{
	LPUART0_SetTransmitterCmd(false);
	LPUART0->CTRL = (LPUART0->CTRL & ~LPUART_CTRL_TIE_MASK);
}

static inline void LPUART0_EnableRxDataISR(void)
{
	LPUART0_SetReceiverCmd(true);
	LPUART0->CTRL = (LPUART0->CTRL | LPUART_CTRL_RIE_MASK);
}

static inline void LPUART0_DisableRxDataISR(void)
{
	LPUART0_SetReceiverCmd(false);
	LPUART0->CTRL = (LPUART0->CTRL & ~LPUART_CTRL_RIE_MASK);
}

static inline bool LPUART0_IsTxDataRegEmpty(void)
{
	return (bool)(LPUART0->STAT & LPUART_STAT_TDRE_MASK);
}

static inline bool LPUART0_IsRxDataRegFull(void)
{
	return (bool)(LPUART0->STAT & LPUART_STAT_RDRF_MASK);
}
