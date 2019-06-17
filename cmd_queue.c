/*
 * cmd_queue.c
 *
 *  Created on: 2018. 6. 29.
 *      Author: hrjung
 */

/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "parameters.h"
#include "cmd_queue.h"



S_QUEUE		cmd_in_q;


/* Private function prototypes -----------------------------------------------*/

void QUE_init(void)
{
	uint16_t i;

	cmd_in_q.rp = 0;
	cmd_in_q.wp = 0;

	for(i=0; i<S_QUEUE_SIZE; i++) memset((void *)&cmd_in_q.buf[i], 0, sizeof(cmd_type_st));
}

int QUE_isFull(void)
{
	uint16_t	pos;

    pos = cmd_in_q.wp + 1;
    if(pos>=S_QUEUE_SIZE) pos=0;

    if (cmd_in_q.rp == pos)
    	return 1;
    else
    	return 0;
}

int QUE_isEmpty(void)
{
    if (cmd_in_q.rp == cmd_in_q.wp)
    	return 1;
    else
    	return 0;
}

int QUE_count(void)
{
    if (cmd_in_q.rp <= cmd_in_q.wp)
    	return (cmd_in_q.wp - cmd_in_q.rp);
    else
		return (S_QUEUE_SIZE + cmd_in_q.wp - cmd_in_q.rp);
}

int QUE_putCmd(cmd_type_st cmd_in)
{
	memcpy(&cmd_in_q.buf[cmd_in_q.wp++], &cmd_in, sizeof(cmd_type_st));
	if(cmd_in_q.wp>=S_QUEUE_SIZE) cmd_in_q.wp = 0;
    return 1;
}

cmd_type_st QUE_getCmd(void)
{
	cmd_type_st temp;

	memcpy(&temp, &cmd_in_q.buf[cmd_in_q.rp++], sizeof(cmd_type_st));
	if(cmd_in_q.rp>=S_QUEUE_SIZE) cmd_in_q.rp = 0;
	return temp;
}
