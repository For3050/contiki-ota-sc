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
#define OTA_DOWNLOAD    0x01
#define FUNC_FRAME_ERROR 0x2
#define PAYLOAD_SIZE 64
#define FRAME_RX_BUF_SIZE 80
#define FRAME_TX_BUF_SIZE 16


static uint8_t server_addr[2] = {0x0,};

static uint16_t frame_id_saved, frame_id_recieved, total_frame = 0x0;
static uint8_t length = 0x0;

static uint8_t frame_rx_buf[FRAME_RX_BUF_SIZE] = {0x0,};
static uint8_t frame_tx_buf[FRAME_TX_BUF_SIZE] = {0x0,};

#if 0
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
static struct FRAME frame;
static struct FRAME *frame_ptr = &frame;
#endif

process_event_t ota_done_event;
process_event_t ota_frame_event;
process_event_t ota_error_event;

PROCESS(ota_download_th, "ota client process");
PROCESS(ota_error_handle_process, "ota error handle process");
PROCESS(ota_parse_frame_process, "ota parse frame process");
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
	uint8_t *p_frame_rx_buf = NULL;

	printf("broadcast message received from %d.%d.\n",
			from->u8[0], from->u8[1]);

	server_addr[0] = from->u8[0];
	server_addr[1] = from->u8[1];

	p_frame_rx_buf = (uint8_t *)packetbuf_dataptr();

	printf("received frame:\n");
	for(i = 0; i < (frame_rx_buf[5]+8); i++)
	{
		frame_rx_buf[i] = *(p_frame_rx_buf+i);
		//printf("%d, ", frame_rx_buf[i]);
	}
	printf("\n");

	if(frame_rx_buf[0] == OTA_DOWNLOAD)
	{
		process_post(&ota_parse_frame_process, ota_frame_event, NULL);
	}


#if 0
	total_frame = (frame_rx_buf[3]<<8)||(frame_rx_buf[4]);
	frame_id_recieved = (frame_rx_buf[1]<<8)||(frame_rx_buf[2]);
	length = (frame_rx_buf[5]);

	ota_bytes_received += length;
	while(length--)
	{
		ota_buffer[img_req_position++] = *((frame_rx_buf+6)++)
	}

	if(metadata_received)
	{
		int percent = 10000.0 * ((float)( ota_bytes_saved + img_req_position) / (float)(new_firmware_metadata.size + OTA_METADATA_SPACE));
		printf("\t%u.%u%%\n", percent/100, (percent - ((percent/100)*100)));
	}
	else if(img_req_position >= OTA_METADATA_LENGTH)
	{
		metadata_received = true;
		active_ota_download_slot = find_empty_ota_slot();
		if(!active_ota_download_slot)
		{
			active_ota_download_slot = 1;
		}
		while(erase_ota_image(active_ota_download_slot));
	}

	if(img_req_position >= OTA_BUFFER_SIZE)
	{
		printf("img_req_position >= OTA_BUFFER_SIZE!\n");
		while(store_firmware_data(ota_bytes_saved + (ota_images[0] << 12), ota_buffer, OTA_BUFFER_SIZE))
			ota_bytes_saved += img_req_position;
		img_req_position = 0;
		reset_ota_buffer();
	}

	if(frame_id_recieved == (total_frame -1))
	{
		printf("OTA received finished!\n");
		process_post(&ota_download_th, ota_done_event, NULL);
	}


#if 0

	//if((frame_id_recieved == 0)||((frame_id_recieved > frame_id_saved) && frame_id_recieved <= total_frame))
	if(frame_id_recieved <= total_frame)
	{
		if((frame_id_recieved - frame_id_saved) == 1)
		{

			printf("%d,good new frame!\n", frame_id_recieved);

			frame_id_saved = frame_id_recieved;

			length = (frame_rx_buf[5]);
			printf("Downloaded %u frames\tinclude (%#x) bytes\n", frame_id_recieved, frame_rx_buf[5]);
			for(i = 0; i < length; i++)
			{
				ota_buffer[ img_req_position++ ] = frame_rx_buf[i+6];
			}

			if(metadata_received)
			{
				int percent = 10000.0 * ((float)( (frame_id_saved*PAYLOAD_SIZE) + img_req_position) / (float)(new_firmware_metadata.size + OTA_METADATA_SPACE));
				printf("\t%u.%u%%\n", percent/100, (percent - ((percent/100)*100)));
			}
			else if(img_req_position >= OTA_METADATA_LENGTH)
			{
				printf("\n\nDownload metadata acquired:\n");
				printf("=======================1=======================\n");
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
					printf("1,active_ota_download_slot=%d\n", active_ota_download_slot);
					//  We don't already have a copy of this firmware version, let's download
					//  to an empty OTA slot!
					active_ota_download_slot = find_empty_ota_slot();
					printf("2,active_ota_download_slot=%d\n", active_ota_download_slot);
					if ( !active_ota_download_slot ) {
						active_ota_download_slot = 1;
					}
				}
				printf("2,active_ota_download_slot=%d\n", active_ota_download_slot);
				printf("Downloading OTA update to OTA slot #%i.\n", active_ota_download_slot);

				//  (3) Erase the destination OTA download slot
				while( erase_ota_image( active_ota_download_slot ) );

				printf("========================2======================\n\n");
			}
			//  (4) Save the latest ota_buffer to flash if it's full.
			if ( img_req_position >= OTA_BUFFER_SIZE-1) {
				printf("======================3========================\n");
				while( store_firmware_data( ( (frame_id_saved*PAYLOAD_SIZE) + (ota_images[active_ota_download_slot-1] << 12)), ota_buffer, OTA_BUFFER_SIZE) );
				//ota_bytes_saved += img_req_position;
				//frame_id_saved++;
				img_req_position = 0;
				reset_ota_buffer();
				printf("========================4======================\n\n");
			}

			if(frame_id_saved == total_frame)
			{
				printf("frame_id_saved=%d\t, total_frame=%d\n");
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
					(frame_rx_buf[1]<<8)||(frame_rx_buf[2]));
		}
	}
	else 
	{
		printf("\nframe has already received, discast!\n");
		printf("now frame id = %d\t, received frame id = %d\n", 
				frame_id_saved,
				(frame_rx_buf[1]<<8)||(frame_rx_buf[2]));
	}
#endif

#endif

}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ota_download_th, ev, data)
{
//	static struct etimer et;

	//(1)Initialize Rime
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
		PROCESS_BEGIN();

	broadcast_open(&broadcast, 129, &broadcast_call);
	process_start(&ota_error_handle_process, NULL);
	process_start(&ota_parse_frame_process, NULL);
	printf("ota_download_th start...\n");

	//(2)Initialize download parameters
	reset_ota_buffer();
	metadata_received = false;
	//ota_download_active = true;

	//ready to reboot
	PROCESS_WAIT_EVENT_UNTIL(ev == ota_done_event);
#if 0
	//  (5) Save the last ota_buffer!  This may not happen in the firmware_chunk_handler
	//      if the last ota_buffer wasn't totally full, as img_req_position
	//      will never increment to OTA_BUFFER_SIZE.
	printf("========================5======================\n");
	while( store_firmware_data( ( (frame_id_recieved*PAYLOAD_SIZE) + (ota_images[0] << 12)), ota_buffer, OTA_BUFFER_SIZE ) );
	ota_bytes_saved += img_req_position;
	printf("========================6=====================\n\n");
#endif

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
	printf("ota_error_handle_process start!\n");

	while(1) {

		PROCESS_WAIT_EVENT_UNTIL(ev == ota_error_event);

		if(!runicast_is_transmitting(&runicast)) {
			linkaddr_t recv;

			frame_tx_buf[0] = FUNC_FRAME_ERROR;
			frame_tx_buf[1] = (uint8_t)((frame_id_saved+1)>>8);
			frame_tx_buf[2] = (uint8_t)(frame_id_saved+1);
			frame_tx_buf[3] = (uint8_t)((0xfeed)>>8);
			frame_tx_buf[4] = (uint8_t)(0xfeed);

			packetbuf_copyfrom(&frame_tx_buf, 5);
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
static uint16_t frame_id_request = 0x0;

PROCESS_THREAD(ota_parse_frame_process, ev, data)
{
	uint8_t i = 0;
	PROCESS_BEGIN();
	printf("ota_parse_frame_process start!\n");
	while(1)
	{
		PROCESS_WAIT_EVENT_UNTIL(ev == ota_frame_event);
		frame_id_recieved = (frame_rx_buf[1]<<8)|(frame_rx_buf[2]);
		total_frame = (frame_rx_buf[3]<<8)|(frame_rx_buf[4]);
		length = (frame_rx_buf[5]);
		printf("frame_id:%d , total_frame:%d , length:%d\n", frame_id_recieved,\
				total_frame, length);

		if(frame_id_recieved == frame_id_request)
		{
			frame_id_request++;
			ota_bytes_received += length;
			while(length--)
			{
				ota_buffer[img_req_position++] = frame_rx_buf[i+6];
				i++;
			}
			memset(frame_rx_buf, 0xff, sizeof(frame_rx_buf));

			//  (3) Handle metadata-specific information
			if ( metadata_received ) {
				//  If we have metadata already, calculate how much of the download is done.
				int percent = 10000.0 * ((float)( ota_bytes_saved + img_req_position) / (float)(new_firmware_metadata.size + OTA_METADATA_SPACE));
				printf("\t%u.%u%%\n", percent/100, (percent - ((percent/100)*100)));
			} else if ( img_req_position >= OTA_METADATA_LENGTH ) {
				printf("\n\nDownload metadata acquired:\n");
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
			}


			if((img_req_position >= 1024)||(frame_id_recieved == (total_frame-1)))
			{
				//store ota_buffer to extern_flash.
				//while( store_firmware_data( ( (ota_bytes_received) + (ota_images[0] << 12)), ota_buffer, img_req_position ) );
				while( store_firmware_data( ( (ota_bytes_saved) + (ota_images[0] << 12)), ota_buffer, img_req_position ) );
				ota_bytes_saved += img_req_position;
				img_req_position = 0x0;
				reset_ota_buffer();
				if(frame_id_recieved == (total_frame-1))
				{
					process_post(&ota_download_th, ota_done_event, NULL);
				}
			}
		}
		else 
		{
			//frame error process.
			printf("frame error!\n");
		}

	}
	PROCESS_END();
}


















