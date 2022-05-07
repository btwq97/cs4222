#include "contiki.h"
#include "dev/leds.h"
#include "core/net/rime/rime.h"
#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "node-id.h"
#include "defs_and_types.h"
#include "net/netstack.h"
#include "random.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifdef TMOTE_SKY
#include "powertrace.h"
#endif
/*---------------------------------------------------------------------------*/
#define WAKE_TIME RTIMER_SECOND / 65 // 0.015s
/*---------------------------------------------------------------------------*/
#define SLEEP_SLOT RTIMER_SECOND / 65 // sleep slot should not be too large to prevent overflow
#define SIZE 19
/*---------------------------------------------------------------------------*/
// duty cycle = WAKE_TIME / (WAKE_TIME + SLEEP_SLOT * SLEEP_CYCLE)
/*---------------------------------------------------------------------------*/

#define MAX_NODES 50
#define DEBUG 0
#define TX_POWER -4
#define AB_THRESHOLD -70
#define DT_THRESHOLD -60

// for data structure
typedef struct Node
{
	int is_detected;			   // state
	int has_detection_in_cycle;	   // checks if node is discovered when all 100 slots are completed
	unsigned long first_discovery; // first discovery timestamp
	unsigned long total_discovery; // total discovery timestamp
	unsigned long first_absent;	   // first absent timestamp
	unsigned long total_absent;	   // total absent timestamp
} SensorTag;

// function declarations
void generateArr();
static int findIndex(int n);
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from);
char sender_scheduler(struct rtimer *t, void *ptr);
void swap(int *a, int *b);
int partition(int array[], int low, int high);
void quickSort(int array[], int low, int high);

// for data structure
void resetNode(int index);
void createNode(unsigned long tagId);
void updateDetectNode(int index);
void updateAbsentNode(int index);
int hasDiscovered(unsigned long tagId);
int findNextEmptyIndex();
void printNode(int index);

// helper functions
unsigned long timeDiff(unsigned long startTimeInSec, unsigned long rawCurrTime);
inline unsigned long ticksToSec(unsigned long timeTicks);
inline void removeSensorTag(int index);

// sender timer
static struct rtimer rt;
static struct pt pt;
/*---------------------------------------------------------------------------*/
static data_packet_struct received_packet;
static data_packet_struct data_packet;
unsigned long curr_timestamp;
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
static int NUM = 10;
static int NUM2 = 100;
int counter = -1;
int ROW = -1;
int COL = -1;
int AWAKE_SLOTS[SIZE] = {0};

// for data structure
SensorTag SensorTags[MAX_NODES];
unsigned long SensorTagIndex[MAX_NODES] = {0};
/*---------------------------------------------------------------------------*/
PROCESS(cc2650_nbr_discovery_process, "cc2650 neighbour discovery process");
AUTOSTART_PROCESSES(&cc2650_nbr_discovery_process);
/*---------------------------------------------------------------------------*/
void createNode(unsigned long tagId)
{
	int index = findNextEmptyIndex();

	// update tagid
	SensorTagIndex[index] = tagId;
	if (DEBUG)
	{
		printf("[createNode] Index = %d, TagId = %lu\r\n", index, tagId);
	}

	// update its status
	SensorTags[index].is_detected = 0;
	SensorTags[index].has_detection_in_cycle = 1;
	SensorTags[index].first_discovery = ticksToSec(curr_timestamp);
	SensorTags[index].total_discovery = 0;
	SensorTags[index].first_absent = 0;
	SensorTags[index].total_absent = 0;

	if (DEBUG)
	{
		printNode(index);
	}
}

inline unsigned long ticksToSec(unsigned long timeTicks)
{
	return (timeTicks / CLOCK_SECOND);
}

unsigned long timeDiff(unsigned long startTimeInSec, unsigned long rawCurrTime)
{
	return (ticksToSec(rawCurrTime) - startTimeInSec);
}

inline void removeSensorTag(int index)
{
	SensorTagIndex[index] = 0;
}

void printNode(int index)
{
	printf("----- node #%ld -----\r\n", SensorTagIndex[index]);
	printf("is_dectected = %d\r\n", SensorTags[index].is_detected);
	printf("has_detection_in_cycle = %d\r\n", SensorTags[index].has_detection_in_cycle);
	printf("first_discovery = %lu\r\n", SensorTags[index].first_discovery);
	printf("total_discovery = %lu\r\n", SensorTags[index].total_discovery);
	printf("first_absent = %lu\r\n", SensorTags[index].first_absent);
	printf("total_absent = %lu\r\n", SensorTags[index].total_absent);
	printf("---------------------\r\n");
}

void resetNode(int index)
{
	removeSensorTag(index);
	SensorTags[index].is_detected = 0;
	SensorTags[index].has_detection_in_cycle = 0;
	SensorTags[index].first_discovery = 0;
	SensorTags[index].total_discovery = 0;
	SensorTags[index].first_absent = 0;
	SensorTags[index].total_absent = 0;
}

void updateDetectNode(int index)
{
	unsigned long tmpTimeDiff = timeDiff(SensorTags[index].first_discovery, curr_timestamp);

	SensorTags[index].has_detection_in_cycle = 1;

	if (tmpTimeDiff >= 15 && !SensorTags[index].is_detected)
	{
		SensorTags[index].is_detected = 1;
		printf("%lu DETECT %ld\r\n", SensorTags[index].first_discovery, SensorTagIndex[index]);
	}

	// update total discovery time
	SensorTags[index].total_discovery = tmpTimeDiff;
	if (DEBUG)
	{
		printNode(index);
	}
}

void updateAbsentNode(int index)
{
	if (DEBUG)
	{
		printf("[sender_scheduler reset] Current Node #%lu\r\n", SensorTagIndex[index]);
	}

	if (SensorTags[index].has_detection_in_cycle)
	{
		// if SensorTags is detected again within 30s
		SensorTags[index].has_detection_in_cycle = 0;
		SensorTags[index].first_absent = 0;
		SensorTags[index].total_absent = 0;
	}
	else
	{
		// before detect state, if you enter zone for < 15s,
		// and you enter again after a few seconds after,
		// sensortag should restart the count for new detect state
		if (!SensorTags[index].is_detected)
		{
			removeSensorTag(index);
			resetNode(index);
		}

		if (SensorTags[index].first_absent == 0)
		{
			SensorTags[index].first_absent = ticksToSec(curr_timestamp);
		}

		unsigned long tmpTimeDiff = timeDiff(SensorTags[index].first_absent, curr_timestamp);
		if (tmpTimeDiff >= 30 && SensorTags[index].is_detected)
		{
			SensorTags[index].is_detected = 0;
			printf("%lu ABSENT %ld\r\n", SensorTags[index].first_absent, SensorTagIndex[index]);

			// remove tagID from SensorTagIndex and SensorTags
			resetNode(index);
			if (DEBUG)
			{
				printNode(index);
			}
			return;
		}

		// update total absent time
		SensorTags[index].total_absent = tmpTimeDiff;
	}

	if (DEBUG)
	{
		printNode(index);
	}
}

int hasDiscovered(unsigned long int tagId)
{
	int i;
	for (i = 0; i < MAX_NODES; i++)
	{
		if (tagId == SensorTagIndex[i])
		{
			return i;
		}
	}
	return -1;
}

int findNextEmptyIndex()
{
	int i;
	for (i = 0; i < MAX_NODES; i++)
	{
		if (SensorTagIndex[i] == 0)
		{
			return i;
		}
	}

	return -1;
}

// function to swap elements
void swap(int *a, int *b)
{
	int t = *a;
	*a = *b;
	*b = t;
}

// function to find the partition position
int partition(int array[], int low, int high)
{

	// select the rightmost element as pivot
	int pivot = array[high];

	// pointer for greater element
	int i = (low - 1);

	// traverse each element of the array
	// compare them with the pivot
	int j;
	for (j = low; j < high; j++)
	{
		if (array[j] <= pivot)
		{
			// if element smaller than pivot is found
			// swap it with the greater element pointed by i
			i++;

			// swap element at i with element at j
			swap(&array[i], &array[j]);
		}
	}

	// swap the pivot element with the greater element at i
	swap(&array[i + 1], &array[high]);

	// return the partition point
	return (i + 1);
}

void quickSort(int array[], int low, int high)
{
	if (low < high)
	{
		// find the pivot element such that
		// elements smaller than pivot are on left of pivot
		// elements greater than pivot are on right of pivot
		int pi = partition(array, low, high);

		// recursive call on the left of pivot
		quickSort(array, low, pi - 1);

		// recursive call on the right of pivot
		quickSort(array, pi + 1, high);
	}
}

void generateArr()
{
	ROW = (random_rand() % NUM) * NUM;
	COL = random_rand() % NUM;
	int count = 0;

	int i = 0;

	for (i = 0; i < NUM; i++)
	{
		int tmp = ROW + i; // row adds 1
		AWAKE_SLOTS[count] = tmp;
		count++;
	}

	int intersection = ROW + COL;

	for (i = 0; i < NUM; i++)
	{
		int tmp = COL + i * NUM;
		if (tmp == intersection)
		{
			continue;
		}
		AWAKE_SLOTS[count] = tmp;
		count++;
	}

	quickSort(AWAKE_SLOTS, 0, SIZE - 1);

	if (DEBUG)
	{
		printf("Printing Matrix...\r\n");
		for (i = 0; i < SIZE; i++)
		{
			printf("%d ", AWAKE_SLOTS[i]);
		}
		printf("\r\n");
	}
}

static int findIndex(int n)
{
	int i = 0;
	for (i = 0; i < 19; i++)
	{
		if (AWAKE_SLOTS[i] == n)
		{
			return i;
		}
	}
	return -1;
}

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	leds_on(LEDS_GREEN);
	memcpy(&received_packet, packetbuf_dataptr(), sizeof(data_packet_struct));

	signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
	int nodeIdx = hasDiscovered(received_packet.src_id);
	int threshold = AB_THRESHOLD;

	if (SensorTags[nodeIdx].is_detected)
	{
		threshold = DT_THRESHOLD;
	}

	if (rssi > threshold)
	{
		if (DEBUG)
		{
			printf("[RX #%d]: Node = %lu. Sequence number = %lu. Timestamp = %3lu.%03lu. RSSI = %d\r\n", counter, received_packet.src_id, received_packet.seq, received_packet.timestamp / CLOCK_SECOND, ((received_packet.timestamp % CLOCK_SECOND) * 1000) / CLOCK_SECOND, rssi);
		}

		if (nodeIdx >= 0)
		{
			// if the tagId has been discovered prior, we check if the time diff > 15
			updateDetectNode(nodeIdx);
		}
		else
		{
			// else we create a new node
			createNode(received_packet.src_id);
		}
	}

	leds_off(LEDS_GREEN);
}

/*---------------------------------------------------------------------------*/
char sender_scheduler(struct rtimer *t, void *ptr)
{
	PT_BEGIN(&pt);
	generateArr();

	curr_timestamp = clock_time();
	if (DEBUG)
	{
		int i;
		printf("Printing SensorTagIndex:\r\n");
		for (i = 0; i < MAX_NODES; i++)
		{
			printf("%lu ", SensorTagIndex[i]);
			if (i == MAX_NODES - 1)
			{
				printf("\r\n");
			}
		}
		printf("[sender_scheduler] Start clock %lu ticks. Timestamp = %3lu.%03lu\r\n", curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND) * 1000) / CLOCK_SECOND);
	}

	while (1)
	{
		counter += 1;

		// reset all occurences here
		if (counter == NUM2)
		{
			int i;
			for (i = 0; i < MAX_NODES; i++)
			{
				if (SensorTagIndex[i] == 0)
				{
					break; // last discovered device
				}

				updateAbsentNode(i);
			}
			counter = 0;
		}

		int index = findIndex(counter);

		if (index >= 0)
		{
			leds_on(LEDS_RED);
			// radio on
			NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, TX_POWER);
			NETSTACK_RADIO.on();

			data_packet.seq++;
			curr_timestamp = clock_time();
			data_packet.timestamp = curr_timestamp;

			// if (DEBUG)
			// {
			// 	printf("[TX #%d]: Send seq #%lu  @ %8lu ticks   %3lu.%03lu\n", counter, data_packet.seq, curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND) * 1000) / CLOCK_SECOND);
			// }

			packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet_struct));
			broadcast_send(&broadcast);
			leds_off(LEDS_RED);

			rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
			PT_YIELD(&pt);
		}
		else
		{
			leds_on(LEDS_BLUE);
			// radio off
			NETSTACK_RADIO.off();

			// SLEEP_SLOT cannot be too large as value will overflow,
			// to have a large sleep interval, sleep many times instead
			rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1, (rtimer_callback_t)sender_scheduler, ptr);
			PT_YIELD(&pt);

			leds_off(LEDS_BLUE);
		}
	}

	PT_END(&pt);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2650_nbr_discovery_process, ev, data)
{
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

	PROCESS_BEGIN();

	// random_init(0x1234);

#ifdef TMOTE_SKY
	powertrace_start(CLOCK_SECOND * 5);
#endif

	broadcast_open(&broadcast, 129, &broadcast_call);

// for serial port
#if !WITH_UIP && !WITH_UIP6
	uart1_set_input(serial_line_input_byte);
	serial_line_init();
#endif

	if (DEBUG)
	{
		printf("[PROCESS_THREAD] CC2650 neighbour discovery\r\n");
		printf("[PROCESS_THREAD] Node %d will be sending packet of size %d Bytes\r\n", node_id, (int)sizeof(data_packet_struct));
	}

	// radio off
	NETSTACK_RADIO.off();

	// initialize data packet
	data_packet.src_id = node_id;
	data_packet.seq = 0;

	// Start sender in one millisecond.
	rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
