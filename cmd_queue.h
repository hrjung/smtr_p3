/*
 * cmd_queue.h
 *
 *  Created on: 2018. 10. 17.
 *      Author: hrjung
 */

#ifndef CMD_QUEUE_H_
#define CMD_QUEUE_H_

#define S_QUEUE_SIZE		32

typedef struct {
	uint16_t 	cmd;
	uint16_t 	index;
	union32_st	data;
} cmd_type_st;


typedef struct {
	uint16_t	rp, wp;
	cmd_type_st	buf[S_QUEUE_SIZE];
} S_QUEUE;

//extern S_QUEUE		cmd_in_q;

extern void QUE_init(void);
extern int QUE_isFull(void);
extern int QUE_isEmpty(void);
extern int QUE_count(void);
extern int QUE_putCmd(cmd_type_st cmd_in);
extern cmd_type_st QUE_getCmd(void);


#endif /* CMD_QUEUE_H_ */
