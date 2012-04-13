/*
 *Licensing information & Copyright info
 *@author: Chuka
 *@email: chuka@puppetme.com
 */

#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "psensorlogging.h"
#include "pcommon.h"
#include "buffer.h"
#include "rest.h"
/*******************************************
    Temperature Specific Include Files
********************************************/
#include "dev/sht11.h"
/*******************************************
              Some Macros
********************************************/
#include <stdio.h>
#ifdef DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SERVER_NODE(ipaddr) uip_ip6addr(ipaddr, 0xaaaa,0,0,0,0,0,0,1);
#define MAX_PAYLOAD_LEN 20

char outputBuffer[MAX_PAYLOAD_LEN];
char payload_buf[MAX_PAYLOAD_LEN];
//static char* proxy_uri = "http://devices.puppetme.com/record";
static char* proxy_uri = "/reading";
static char* service_uri = "proxy";
static unsigned int xact_id; //message id
static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
static char* server_ip = "aaaa::1"; //TODO: Refactor
PROCESS(ptemperature_client, "Temperature Sensor & Actuator");
PROCESS(sensorlogging, "Main function ");
AUTOSTART_PROCESSES(&sensorlogging);

/*******************************************
              Resource Definitions
********************************************/

/*******************************************
            Resource Handlers
********************************************/

/*******************************************
            Function Implementation
********************************************/
void ptemperature_initialize()
{
  process_exit(&ptemperature_client);
  //Start client processes next.
  process_start(&ptemperature_client, NULL);
}

static
void generate_payload(char* buf, unsigned data)
{
  int index = 0;
  index += sprintf(buf,"device_id=%s,1=%d","2",data);
}

static
void send_data()
{
  int data_size = 0;
  static unsigned temperature;

  temperature = 21;
  clear_buffer(outputBuffer);
  clear_buffer(payload_buf);
  generate_payload(payload_buf,temperature);

  if(init_buffer(COAP_DATA_BUFF_SIZE)){
    coap_packet_t* request =\
    (coap_packet_t*)allocate_buffer(sizeof(coap_packet_t));
    init_packet(request);
    coap_set_method(request, COAP_POST);
    request->tid = xact_id++;
    request->type = MESSAGE_TYPE_CON;
    coap_set_header_uri(request,service_uri);
    coap_set_option(request, Option_Type_Uri_Host,
      sizeof(char)*strlen(server_ip), (uint8_t*)server_ip);
    coap_set_option(request, Option_Type_Proxy_Uri,
    sizeof(char)*strlen(proxy_uri), (uint8_t*)proxy_uri);
    coap_set_payload(request,(uint8_t*)payload_buf,
    sizeof(char)*strlen(payload_buf));
    data_size = serialize_packet(request, (uint8_t*)outputBuffer);

    PRINTF("Now sending request to base station [");
    PRINTF(&client_conn->ripaddr);
    PRINTF("]:%u/%s\n",REMOTE_PORT,service_uri);
    uip_udp_packet_send(client_conn, outputBuffer, data_size);
    delete_buffer();
  }
}

static
void clear_buffer()
{
  memset(outputBuffer,'\x0',sizeof(char)*OUTPUT_BUFFER_SIZE);
}

/********************************************
          Process Definitions
*********************************************/

PROCESS_THREAD(ptemperature_client, ev, data)
{
  static struct etimer atimer;
  PROCESS_BEGIN();
  SERVER_NODE(&server_ipaddr);
  PRINTF("Creating connection to server");
  client_conn = udp_new(&server_ipaddr, UIP_HTONS(REMOTE_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(LOCAL_PORT));
  sht11_init();

  PRINTF("Starting ptemperature client timer");
  etimer_set(&atimer, CLOCK_SECOND*POLL_INTERVAL);
  while(1)
  {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&atimer));
    PRINTF("Timer expired. Initiating temperature send.");
    send_data();
    etimer_reset(&atimer);
  }
  PROCESS_END();
}

PROCESS_THREAD(sensorlogging, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  set_node_addresses();
  print_node_addresses();

  rest_init();
  ptemperature_initialize();

  PROCESS_END();
}
