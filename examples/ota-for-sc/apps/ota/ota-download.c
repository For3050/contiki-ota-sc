/**
 * \file
 *         ota-download.c
 * \file   
 * 	       New OTA Image Download Mechanism
 * \author
 *         pf
 */

#include "ota-download.h"
#include "ti-lib.h"
#include "contiki.h"
#include "net/rime/rime.h"
#include <stdio.h>

#include "process.h"
#include "ota.h"


#define MAX_RETRANSMISSIONS 4
#define FUNC_ACK 0x2
#define PAYLOAD_SIZE 64


static uint8_t server_addr[2] = {0x0,};

static uint16_t frame_id_saved, frame_id_recieved, total_frame = 0x0;
static uint8_t length = 0x0;

struct FRAME
{
	uint8_t func;
	uint16_t frame_id;
	uint16_t total_frame;
	uint8_t length;
	uint8_t payload[PAYLOAD_SIZE];
	uint16_t crc;
};

struct ACK
{
	uint8_t func;
	uint16_t frame_id;
	uint16_t crc;
};

static struct ACK ack;
static struct FRAME *frame_ptr;

process_event_t ota_done_event;
process_event_t ota_error_event;

PROCESS(ota_download_th, "ota client process");
PROCESS(ota_error_handle_process, "ota error handle process");
struct process* ota_download_th_p = &ota_download_th;

/*---------------------------------------------------------------------------*/
static void
reset_ota_buffer() {
	uint16_t n;
	for (n=0; n<OTA_BUFFER_SIZE; n++)
	{
		ota_buffer[ n ] = 0xff;
	}
}


/*---------------------------------------------------------------------------*/
	static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	static uint8_t i = 0;

	printf("broadcast message received from %d.%d.\n",
			from->u8[0], from->u8[1]);

	server_addr[0] = from->u8[0];
	server_addr[1] = from->u8[1];

	frame_ptr = (struct FRAME*)packetbuf_dataptr();

	total_frame = (frame_ptr->total_frame);
	frame_id_recieved = (frame_ptr->frame_id);
	length = (frame_ptr->length);

if((frame_id_recieved > frame_id_saved) && frame_id_recieved <= total_frame)
{
	if((frame_id_recieved - frame_id_saved) == 1)
	{

		printf("%d,good new frame!\n", frame_id_recieved);

		frame_id_saved = frame_id_recieved;

		length = (frame_ptr->length);
		printf("Downloaded %u frames\tinclude (%#x) bytes\n", frame_id_recieved, frame_ptr->length);
		for(i = 0; i < length; i++)
		{
			ota_buffer[ img_req_position++ ] = frame_ptr->payload[i];
		}

		if(metadata_received)
		{
			int percent = 10000.0 * ((float)( (frame_id_saved*PAYLOAD_SIZE) + img_req_position) / (float)(new_firmware_metadata.size + OTA_METADATA_SPACE));
			printf("\t%u.%u%%\n", percent/100, (percent - ((percent/100)*100)));
		}
		else if(img_req_position >= OTA_METADATA_LENGTH)
		{
			printf("\n\nDownload metadata acquired:\n");
			printf("==============================================\n");
			//  If we don't have metadata yet, get it from the first page
			//  (1) Extract metadata from the ota_buffer
			memcpy( &new_firmware_metadata, ota_buffer, OTA_METADATA_LENGTH );
			print_metadata( &new_firmware_metadata );
			printf("\n");
			metadata_received = true;

			//  (2) Check to see if we have any OTA slots already containing
			//      firmware of the same version number as the metadata has.
			active_ota_download_slot = find_matching_ota_slot( new_firmware_metadata.version );
			if ( active_ota_download_slot == -1 ) {
				//  We don't already have a copy of this firmware version, let's download
				//  to an empty OTA slot!
				active_ota_download_slot = find_empty_ota_slot();
				if ( !active_ota_download_slot ) {
					active_ota_download_slot = 1;
				}
			}
			printf("Downloading OTA update to OTA slot #%i.\n", active_ota_download_slot);

			//  (3) Erase the destination OTA download slot
			while( erase_ota_image( active_ota_download_slot ) );

			printf("==============================================\n\n");
			//  (4) Save the latest ota_buffer to flash if it's full.
			if ( img_req_position >= OTA_BUFFER_SIZE) {
				printf("==============================================\n");
				while( store_firmware_data( ( (frame_id_saved*PAYLOAD_SIZE) + (ota_images[active_ota_download_slot-1] << 12)), ota_buffer, OTA_BUFFER_SIZE) );
				//ota_bytes_saved += img_req_position;
				//frame_id_saved++;
				img_req_position = 0;
				reset_ota_buffer();
				printf("==============================================\n\n");
			}
		}

		if(frame_id_saved == total_frame)
		{
			//ready to reboot
			process_post(&ota_download_th, ota_done_event, NULL);
		}
	}
	else
	{
		printf("\nerror frame!\n");
		process_post(&ota_error_handle_process, PROCESS_EVENT_CONTINUE, NULL);

		printf("need frame id = %d\t, received frame id = %d\n", 
				frame_id_saved + 1,
				frame_ptr->frame_id);
	}
}
else 
{
	printf("\nframe has already received, discast!\n");
	printf("now frame id = %d\t, received frame id = %d\n", 
			frame_id_saved,
			frame_ptr->frame_id); 
}


}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ota_download_th, ev, data)
{
	static struct etimer et;

	//(1)Initialize Rime
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
		PROCESS_BEGIN();
	process_start(&ota_error_handle_process, NULL);

	broadcast_open(&broadcast, 129, &broadcast_call);

	//(2)Initialize download parameters
	reset_ota_buffer();
	metadata_received = false;
	ota_download_active = true;


#if 0
	while(1) {
		etimer_set(&et, CLOCK_SECOND * 1);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	}

#endif

	//ready to reboot
		PROCESS_WAIT_EVENT_UNTIL(ev == ota_done_event);
  //  (5) Save the last ota_buffer!  This may not happen in the firmware_chunk_handler
  //      if the last ota_buffer wasn't totally full, as img_req_position
  //      will never increment to OTA_BUFFER_SIZE.
  printf("==============================================\n");
  while( store_firmware_data( ( (frame_id_saved*PAYLOAD_SIZE) + (ota_images[active_ota_download_slot-1] << 12)), ota_buffer, OTA_BUFFER_SIZE ) );
  //ota_bytes_saved += img_req_position;
  printf("==============================================\n\n");

  //  (6) Recompute the CRC16 algorithm over the received data.  This value is
  //      called the "CRC Shadow." If it doesn't match the CRC value in the
  //      metadata, our download got messed up somewhere along the way!
  while( verify_ota_slot( active_ota_download_slot ) );

  //  (7) Reboot!
  printf("-----OTA download done, rebooting!-----\n");
  ti_lib_sys_ctrl_system_reset();

	PROCESS_END();
}


/*---------------------------------------------------------------------------*/
	static void
recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{

	printf("runicast message received from %d.%d, seqno %d\n",
			from->u8[0], from->u8[1], seqno);
}
static const struct runicast_callbacks runicast_callbacks = {recv_runicast,};
static struct runicast_conn runicast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ota_error_handle_process, ev, data)
{
	PROCESS_EXITHANDLER(runicast_close(&runicast);)

		PROCESS_BEGIN();

	runicast_open(&runicast, 144, &runicast_callbacks);
	/* Receiver node: do nothing */
	if(linkaddr_node_addr.u8[0] == 1 &&
			linkaddr_node_addr.u8[1] == 0) {
		PROCESS_WAIT_EVENT_UNTIL(0);
	}

	while(1) {

		PROCESS_WAIT_EVENT_UNTIL(ev == ota_error_event);

		if(!runicast_is_transmitting(&runicast)) {
			linkaddr_t recv;

			(ack.func) = FUNC_ACK;
			(ack.frame_id) = (frame_id_saved+1);
			(ack.crc) = 0xfeed;

			packetbuf_copyfrom(&ack, sizeof(struct ACK));
			recv.u8[0] = server_addr[0];
			recv.u8[1] = server_addr[1];

			printf("%u.%u: sending runicast to address %u.%u\n",
					linkaddr_node_addr.u8[0],
					linkaddr_node_addr.u8[1],
					recv.u8[0],
					recv.u8[1]);

			runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/



















