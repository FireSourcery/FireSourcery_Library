static inline void HAL_UART_WriteTxChar(HAL_UART_T * p_hal, uint8_t txChar){}
static inline uint8_t HAL_UART_ReadRxChar(const HAL_UART_T * p_hal){}

static inline bool _HAL_UART_ReadIsTxRegEmpty(const HAL_UART_T * p_hal){}
static inline bool _HAL_UART_ReadIsRxRegFull(const HAL_UART_T * p_hal){}
static inline uint8_t HAL_UART_ReadTxEmptyCount(const HAL_UART_T * p_hal){}
static inline uint8_t HAL_UART_ReadRxFullCount(const HAL_UART_T * p_hal){}

static inline bool HAL_UART_ReadRxOverrun(HAL_UART_T * p_hal){}
static inline void HAL_UART_ClearRxErrors(HAL_UART_T * p_hal){}

static inline void HAL_UART_EnableTxInterrupt(HAL_UART_T * p_hal){}
static inline void HAL_UART_DisableTxInterrupt(HAL_UART_T * p_hal){}
static inline void HAL_UART_EnableRxInterrupt(HAL_UART_T * p_hal){}
static inline void HAL_UART_DisableRxInterrupt(HAL_UART_T * p_hal){}

static inline void HAL_UART_WriteTxSwitch(HAL_UART_T * p_hal, bool enable){}
static inline void HAL_UART_WriteRxSwitch(HAL_UART_T * p_hal, bool enable){}
static inline bool HAL_UART_ConfigBaudRate(HAL_UART_T * p_hal, uint32_t baudRate_Bps){}

static inline void HAL_UART_Init(HAL_UART_T * p_hal){}
static inline void HAL_UART_Deinit(HAL_UART_T * p_hal){}