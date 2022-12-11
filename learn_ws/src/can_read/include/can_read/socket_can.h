#include <pthread.h>

#include <stdint.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <time.h>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <linux/net_tstamp.h>

#define SET_CAN0_BAUDRATE "sudo ip link set can0 type can bitrate 1000000"
#define CAN_FD_ON " dbitrate 2000000 berr-reporting on fd on"
#define CAN0_OPEN "sudo ip link set up can0"
#define CAN0_CLOSE "sudo ip link set down can0"

static void* ReceiveFunc(void* param);
static void* TransmitFunc(void* param);

pthread_t thread_0_;
static int run_flag_;
uint32_t receive_msgs_count;

uint8_t kbaudrate_port0 = 1000000; // 250000 1000000
uint8_t raw_socket_;
struct ifreq ifr_;
struct sockaddr_can addr_;
const char bus_name0[5] = "can0";

