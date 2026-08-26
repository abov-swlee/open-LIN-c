/**
 * @file open_lin_frame.c
 * @brief LIN frame layer definition
 */

#include "open_lin_data_layer.h"
#include "open_lin_types.h"
#include "open_lin_hw.h"

open_lin_pid_t open_lin_data_layer_parity(open_lin_pid_t pid) {
	open_lin_pid_t result = 0;
	result = (open_lin_pid_t)(OPEN_LIN_GET_PID_BIT(pid,0) ^ OPEN_LIN_GET_PID_BIT(pid,1) ^
			 OPEN_LIN_GET_PID_BIT(pid,2) ^ OPEN_LIN_GET_PID_BIT(pid,4)) << OPEN_LIN_P0_FLAG;

    result |= (open_lin_pid_t)(~(OPEN_LIN_GET_PID_BIT(pid,1) ^ OPEN_LIN_GET_PID_BIT(pid,3) ^
    			  OPEN_LIN_GET_PID_BIT(pid,4) ^ OPEN_LIN_GET_PID_BIT(pid,5)) << OPEN_LIN_P1_FLAG);

    result |= (open_lin_pid_t)(OPEN_LIN_ID_MASK & pid);
    return result;
}

open_lin_checksum_t open_lin_data_layer_checksum(open_lin_pid_t pid, open_lin_checksum_type_t checksum_type, l_u8 length, const l_u8* data_ptr)
{
	l_u8 i = 0;
	l_u16 result;

	/* diagnostic frames always use classic checksum regardless of checksum_type -
	 * this is a protocol rule (LIN-Spec 2.2A Section 2.3.1.2), not an LDF choice,
	 * so it overrides whatever the caller's slot happens to be configured with. */
	if ((pid == OPEN_LIN_DIAG_REQUEST) || (pid == OPEN_LIN_DIAG_RESPONSE) || (checksum_type == OPEN_LIN_CHECKSUM_CLASSIC))
	{
		result = 0;
	} else {
		result = open_lin_data_layer_parity(pid);
	}

	for (i = 0; i < length; i++)
	{
		result += data_ptr[i];
		if (result > 0xFF)
		{
			result -= 0xFF;
		} else
		{
			/* do nothing */
		}
	}
	return (open_lin_checksum_t)(~result);
}
