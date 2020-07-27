#ifndef SPP_UTILS_H
#define SPP_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bg_types.h"
#include "app.h"

typedef struct
{
	uint32_t num_pack_sent;
	uint32_t num_bytes_sent;
	uint32_t num_pack_received;
	uint32_t num_bytes_received;
	uint32_t num_writes; /* Total number of send attempts */
} tsCounters;

/*
 * Optional UART RX buffer overflow checking. This can be added into retargetserial.c
 * by creating a global variable (type int) there with name 'rxOverFlow' and incrementing
 * that variable each time the RX FIFO (the large FIFO in RAM) overflows
 *
 * If you are using the default retargetserial.c implementation without overflow tracking
 * then make sure RX_OVERFLOW_TRACKING is NOT defined
 * */
//#define RX_OVERFLOW_TRACKING
#ifdef RX_OVERFLOW_TRACKING
extern volatile int rxOverFlow;
#endif

void spp_main(void);
void printStats(tsCounters *psCounters);

/* Prototypes of SPP main functions (client / server role) */
void spp_client_main(void);
void spp_server_main(void);

#ifdef __cplusplus
}
#endif

#endif // SPP_UTILS_H
