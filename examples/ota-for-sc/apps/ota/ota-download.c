/** @file   ota-download.c
 *  @brief  OTA Image Download Mechanism
 */

#include "ota-download.h"
#include "ti-lib.h"
#include "contiki.h"
#include <stdio.h>
#include "net/rime/rime.h"

#include "ota.h"

/*******************************************************************************/
#if 0
#if OTA_DEBUG
  #include <stdio.h>
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...)
#endif
#endif

#define RECV_BUF_SIZE 140
#define SEND_BUF_SIZE 16

#define MAX_RETRANSMISSIONS 2

#define OTA_START_COMMAND 0X01
#define OTA_SEND_COMMAND 0X02
#define OTA_REQUEST_COMMAND 0X03

/*******************************************************************************/
static uint8_t recv_buf[RECV_BUF_SIZE] = {0x00, };
static uint8_t send_buf[SEND_BUF_SIZE] = {0x00, };

static uint8_t payload_size = 0x00;
static uint32_t firmware_size = 0x00;

static uint8_t server_addr[2] = {0x00, };

static uint32_t rime_request_count = 0;

/*******************************************************************************/
PROCESS(ota_download_th, "OTA Download Agent");
PROCESS(ota_rcdownload_thread, "OTA RCDownload Agent");
struct process* ota_download_th_p = &ota_download_th;
struct process* ota_rcdownload_th_p = &ota_rcdownload_thread;

/*******************************************************************************/
#if 0
static void
reset_ota_buffer() {
  uint16_t n;
  for (n=0; n<OTA_BUFFER_SIZE; n++)
  {
    ota_buffer[ n ] = 0xff;
  }
}
#endif
/*******************************************************************************/
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	static uint8_t len = 0x0;

	printf("OTA command received!\n");
	printf("broadcast message received from %d.%d: '%s'\n",
			from->u8[0], from->u8[1], (char*)packetbuf_dataptr());
	server_addr[0] = from->u8[0];
	server_addr[1] = from->u8[1];

	/* 1,OTA command parse */
	memset(recv_buf, 0xff, sizeof(recv_buf));
	len = packetbuf_copyto(recv_buf);
	if((recv_buf[0] == OTA_START_COMMAND) && (recv_buf[1] == 0x01)) 
	{
		payload_size = recv_buf[2];
		firmware_size = (recv_buf[3]<<24)||(recv_buf[4]<<16)
						||(recv_buf[5]<<8)||(recv_buf[6]);

		if(!process_is_running(&ota_rcdownload_thread))
		{
			process_start(&ota_rcdownload_thread, NULL);
			printf("ota_rcdownload_thread is started!\n");
		}
	}

}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv, };
static struct broadcast_conn broadcast;
/*******************************************************************************/
PROCESS_THREAD(ota_download_th, ev, data)
{
	static struct etimer et;
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();
	broadcast_open(&broadcast, 111, &broadcast_call);
	printf("ota_download_th is running!\n");
	while(1)
	{
		etimer_set(&et, CLOCK_SECOND*2);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		packetbuf_copyfrom("hello", 6);
		broadcast_send(&broadcast);
		printf("broadcast message sent!\n");
	}
	PROCESS_END();
}
/*******************************************************************************/
static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{
	static uint32_t request_count = 0x00;
	static uint8_t len = 0x0;

	printf("runicast message received from %d.%d, seqno %d\n",
			from->u8[0], from->u8[1], seqno);
	/* 3,Parse and flash OTA data from server! */
	memset(recv_buf, 0xff, sizeof(recv_buf));
	len = packetbuf_copyto(recv_buf);

	request_count = (recv_buf[4]<<24)||(recv_buf[5]<<16)
					||(recv_buf[6]<<8)||(recv_buf[7]<<0);

	if(recv_buf[0] == OTA_SEND_COMMAND)
	{
		while(store_firmware_data(request_count*128, recv_buf+8, 128));
		printf("stored data to flash!\n");
	}

}

/*******************************************************************************/
static const struct runicast_callbacks runicast_callbacks = {recv_runicast, };
static struct runicast_conn runicast;
/*******************************************************************************/
PROCESS_THREAD(ota_rcdownload_thread, ev, data)
{
	static struct etimer et;
	PROCESS_EXITHANDLER(runicast_close(&runicast);)
	PROCESS_BEGIN();
	runicast_open(&runicast, 123, &runicast_callbacks);

	if(linkaddr_node_addr.u8[0] == 1 &&
			linkaddr_node_addr.u8[1] == 0)
	{
		PROCESS_WAIT_EVENT_UNTIL(0);
	}
	printf("OTA is running!\n");

	metadata_received = false;
	ota_download_active = true;

	rime_request_count = 0;

	/* 2,send OTA_REQUEST_COMMAND to server */
	while(ota_download_active)
	{
		etimer_set(&et, CLOCK_SECOND*1);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		printf("------rime request %lu started-----\n", rime_request_count);

		linkaddr_t recv;
		memset(send_buf, 0, sizeof(send_buf));
		send_buf[0] = OTA_REQUEST_COMMAND;
		send_buf[1] = (rime_request_count>>24)&0xff;
		send_buf[2] = (rime_request_count>>16)&0xff;
		send_buf[3] = (rime_request_count>>8)&0xff;
		send_buf[4] = (rime_request_count>>0)&0xff;
		
		recv.u8[0] = server_addr[0];
		recv.u8[1] = server_addr[1];

		runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
		printf("client send request %lu to server!\n", rime_request_count);

		if(rime_request_count >= (firmware_size/payload_size))
		{
			etimer_set(&et, CLOCK_SECOND*3);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

			printf("OTA Finished!\n");
			ota_download_active = false;
			break;
		}

		rime_request_count++;
	}

	//while(verify_ota_slot(active_ota_download_slot));
	while(verify_ota_slot(1));

	printf("OTA download done, rebooting!\n");
	ti_lib_sys_ctrl_system_reset();
	PROCESS_END();
}




