
/*
	Protocol, by its nature, uses global scope. TxRx specified with awareness of global scope.
	instance scope var map may not be necessary. i.e 2 protocols may use singleton map without collisions
 */
#define PROTOCOL_G_VAR_MAP_ENTRY(var) ({&(#var), sizeof(var)})

typedef const struct
{
	union
	{
		void * const P_VAR;
		const uint32_t VAR_LITERAL;
	};
	const uint8_t SIZE;
	//type read/write/literal
} VarMap_Var_T;


typedef   struct
{
	VarMap_Var_T * const P_VAR_TABLE;
	void (*const BUILD_TX_VAR) 	(volatile void * p_subState, volatile uint8_t * p_txPacket, volatile size_t * p_txSize, const uint8_t * p_var, uint8_t varSize);

//	uint32_t * p_VarBuffer;
}
VarMap_T;

typedef void (*const BuildTxVar_T) (volatile void * p_subState, volatile uint8_t * p_txPacket, volatile size_t * p_txSize, uint8_t * p_var, uint8_t varSize);


bool VarMap_ReadVar(VarMap_T * p_varMap, uint16_t varId, p_subState)
{
	void * p_value;
	uint8_t valueSize;

	if (varId < TableLength)
	{
		p_value = (void *)p_varMap->p_Table[varId].P_VAR;
		valueSize = p_varMap->p_Table[varId].SIZE;

		if(p_value != 0U)
		{
			p_varMap->BUILD_TX_VAR(p_subState, p_txPacket , p_txSize, p_value, valueSize);
//			memcpy(p_varMap->p_VarBuffer, p_value, valueSize);
		}
	}

}
