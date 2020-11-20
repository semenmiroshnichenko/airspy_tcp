/*
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Airspy port :
 * Copyright (C) 2018 by Thierry Leconte http://www.github.com/TLeconte
 *
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <pthread.h>
#include <float.h>

#include <libairspyhf/airspyhf.h>
#include <samplerate.h>

// SRC_SINC_BEST_QUALITY 97dB 97%
// SRC_SINC_MEDIUM_QUALITY 97dB 90%
// SRC_SINC_FASTEST 97dB 80%
// SRC_LINEAR
#define DEFAULT_CONVERTER SRC_SINC_FASTEST
SRC_STATE *src_state_ptr;

#define AIRSPY_SAMPLE_TYPE AIRSPY_SAMPLE_FLOAT32_IQ // AIRSPY_SAMPLE_INT16_IQ

#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1

static SOCKET s;

static pthread_t tcp_worker_thread;
static pthread_t command_thread;
static pthread_cond_t exit_cond;
static pthread_mutex_t exit_cond_lock;

static pthread_mutex_t ll_mutex;
static pthread_cond_t cond;

struct llist {
  uint8_t *data;
  size_t len;
  struct llist *next;
};

typedef struct { /* structure size must be multiple of 2 bytes */
  char magic[4];
  uint32_t tuner_type;
  uint32_t tuner_gain_count;
} dongle_info_t;

static struct airspyhf_device* dev = NULL;
static uint32_t fscount,*supported_samplerates;
static int verbose=0;
static int lock=0;
static int ppm_error=0;
static int dshift=1;
static int output_sample_rate = 0;
static int rf_sample_rate = 0;

static int enable_biastee = 0;
static int global_numq = 0;

static struct llist *ls_buffer = NULL;
static struct llist *le_buffer = NULL;
static int llbuf_num = 64;

static volatile int do_exit = 0;

void usage(void)
{
  printf("airspy_tcp, a rtl-tcp compatible, I/Q server for airspy SDR\n\n"
    "Usage:\t[-a listen address]\n"
    "\t[-d device serial]\n"
    "\t[-p listen port (default: 1234)]\n"
    "\t[-f frequency to tune to [Hz]]\n"
    "\t[-g gain (default: 0 for auto)]\n"
    "\t[-s output samplerate in Hz ]\n"
    "\t[-S hardware samplerate in Hz ]\n"
    "\t[-n max number of linked list buffer to keep ]\n"
    "\t[-T enable bias-T ]\n"
    "\t[-P ppm_error (default: 0) ]\n"
    "\t[-D g digital shift (default : 1) ]\n"
    "\t[-L Lock settings ]\n"
    "\t[-v Verbose ]\n");
  exit(1);
}

static void sighandler(int signum)
{
  (void)signum;
  do_exit = 1;
  pthread_cond_signal(&cond);
}

static int rx_callback(airspyhf_transfer_t* transfer)
{
  int r;
  float *airspy_buffer;
  float *resampled_buffer;
  struct llist *rpt;
  uint32_t i;
  int16_t v;
  int16_t o;

  if(do_exit)
  {
    return 0;
  }

  airspy_buffer = (float *)transfer->samples;

  resampled_buffer = (float *)malloc(2 * transfer->sample_count * sizeof(float));

  SRC_DATA src_data;
  src_data.data_in = airspy_buffer;
  src_data.input_frames = (transfer->sample_count);

  src_data.data_out = resampled_buffer;
  src_data.output_frames = (transfer->sample_count);

  src_data.src_ratio = (double)output_sample_rate/(double)rf_sample_rate;
  src_data.end_of_input = 0;

  r = src_process(src_state_ptr, &src_data);
  if(r == 0)
  {
    //printf("Processed OK, %d samples in, %ld used, %ld samples out.\n"
    //  , transfer->sample_count
    //  , src_data.input_frames_used
    //  , src_data.output_frames_gen
    //);
  }
  else
  {
    printf("Processing error: %s\n", src_strerror(r));
    printf(" - using src ratio: %.03f\n", src_data.src_ratio);
    free(resampled_buffer);
    return 0;
  }

  /* Allocate new linked-list element, with appropriate data size */
  rpt = (struct llist*)malloc(sizeof(struct llist));
  rpt->len = (2*transfer->sample_count);
  rpt->data = malloc(rpt->len * sizeof(uint8_t));
  rpt->next = NULL;

  for(i = 0; i < rpt->len; i++)
  {
    v = (((int16_t)(resampled_buffer[i]*INT16_MAX) >> dshift));
    /* stupid added offset (-154), because osmosdr client code */
    /* try to compensate rtl dongle offset */

    /* round to 8bits half up to even */
    o = v >> 8;
    if(v&0x80)
    {
     if(v&0x7f) {o++;} else { if(v&0x100) o++;}
    }

    rpt->data[i] = (uint8_t)((o&0xff) + 128);
    //printf("%d,", rpt->data[i]);
  }

  free(resampled_buffer);

  /* Move data onto linked-list buffer for TCP output */
  pthread_mutex_lock(&ll_mutex);

  /* Copy pointer onto buffer */
  if (ls_buffer == NULL)
  {
    ls_buffer = le_buffer = rpt;
  }
  else
  {
    le_buffer->next=rpt;
    le_buffer=rpt;
  }
  global_numq++;

  /* Trim list */
  if(global_numq > llbuf_num)
  {
    struct llist *curelem;
    curelem=ls_buffer;
    ls_buffer=ls_buffer->next;
    if(ls_buffer==NULL)
      le_buffer=NULL;
    global_numq--;
    free(curelem->data);
    free(curelem);
  }

  pthread_cond_signal(&cond);
  pthread_mutex_unlock(&ll_mutex);

  return 0;
}

static void *tcp_worker(void *arg)
{
  (void)arg;
  struct llist *curelem;
  int bytessent, index;

  while(1)
  {

    pthread_mutex_lock(&ll_mutex);

    while(ls_buffer==NULL && do_exit==0)
      pthread_cond_wait(&cond, &ll_mutex);

    if(do_exit)
    {
      pthread_mutex_unlock(&ll_mutex);
      pthread_exit(0);
    }

    /* Capture pointer to top (oldest) element */
    curelem = ls_buffer;
    /* Move next element up to the top (can be NULL) */
    ls_buffer = ls_buffer->next;
    /* Decrement list counter */
    global_numq--;

    pthread_mutex_unlock(&ll_mutex);

    index = 0;
    while(curelem->len > 0)
    {
      bytessent = send(s,  &curelem->data[index], curelem->len, 0);
      curelem->len -= bytessent;
      index += bytessent;
      if(bytessent == SOCKET_ERROR || do_exit)
      {
        printf("worker socket bye\n");
        sighandler(0);
        pthread_exit(NULL);
      }
    }
    free(curelem->data);
    free(curelem);
  }
}

struct command{
  unsigned char cmd;
  unsigned int param;
}__attribute__((packed));


static int set_agc(uint8_t value)
{
  int r;

  r=airspyhf_set_hf_agc(dev, value);
        if( r != AIRSPYHF_SUCCESS ) return r;


  //r=airspy_set_mixer_agc(dev, value);
        return r;
}

static int set_hw_samplerate(uint32_t fs)
{
  uint32_t i;
  int r;

  for(i=0;i<fscount;i++)
  {
    if(supported_samplerates[i]==fs)
    {
      break;
    }
  }

  if(i < fscount)
  {
    r=airspyhf_set_samplerate(dev, i);
    return r;
  }

  printf("hw sample rate %d not supported\n",fs);
  return AIRSPYHF_ERROR;
}

static int set_freq(uint32_t f)
{
  int r;
  r=airspyhf_set_freq(dev, (uint32_t)((float)f*(1.0+(float)ppm_error/1e6)));
  return r;
}

static void *command_worker(void *arg)
{
  (void)arg;
  int left, received = 0;
  fd_set readfds;
  struct command cmd={0, 0};
  struct timeval tv= {1, 0};
  int r = 0;

  while(1) {
    left=sizeof(cmd);
    while(left >0) {
      FD_ZERO(&readfds);
      FD_SET(s, &readfds);
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      r = select(s+1, &readfds, NULL, NULL, &tv);
      if(r) {
        received = recv(s, (char*)&cmd+(sizeof(cmd)-left), left, 0);
        left -= received;
      }
      if(received == SOCKET_ERROR || do_exit) {
        printf("comm recv bye\n");
        sighandler(0);
        pthread_exit(NULL);
      }
    }

    if(lock)
    {
      printf("Control code 0x%02x discarded - settings locked!\n", cmd.cmd);
      cmd.cmd = 0xff;
      continue;
    }

    switch(cmd.cmd)
    {
      case 0x01:
        if(verbose) printf("set freq %d\n", ntohl(cmd.param));
        set_freq(ntohl(cmd.param));
        break;
      case 0x02:
        if(verbose) printf("set sample rate : %d\n", ntohl(cmd.param));
        set_hw_samplerate(ntohl(cmd.param));
        break;
      case 0x03:
        if(verbose) printf("set gain mode %d : not implemented \n", ntohl(cmd.param));
        break;
      case 0x04:
        if(verbose) printf("set gain (not implemented!!) : %d\n", ntohl(cmd.param));
        //airspy_set_linearity_gain(dev,(ntohl(cmd.param)+250)/37);
        break;
      case 0x05:
        if(verbose) printf("set freq correction %d\n",ntohl(cmd.param));
        ppm_error=ntohl(cmd.param);
        break;
      case 0x06:
        if(verbose) printf("set if stage gain %d : not implemented\n",ntohl(cmd.param));
        break;
      case 0x07:
        if(verbose) printf("set test mode %d: not impmemented\n",ntohl(cmd.param));
        break;
      case 0x08:
        set_agc(ntohl(cmd.param));
        break;
      case 0x09:
        if(verbose) printf("set direct sampling %d: not implemented\n",ntohl(cmd.param));
        break;
      case 0x0a:
        if (verbose) printf("set offset tuning %d : not impemented\n",ntohl(cmd.param));
        break;
      case 0x0b:
        if(verbose) printf("set rtl xtal %d : not implemented\n",ntohl(cmd.param));
        break;
      case 0x0c:
        if(verbose) printf("set tuner xtal %d : not implemented\n",ntohl(cmd.param));
        break;
      case 0x0d:
        if(verbose) printf("set tuner gain by index %d (not supported) \n", ntohl(cmd.param));
        //airspy_set_linearity_gain(dev,ntohl(cmd.param));
        break;
      case 0x0e:
        if(verbose) printf("set bias tee (not implemented) %d\n", ntohl(cmd.param));
        //airspy_set_rf_bias(dev, (int)ntohl(cmd.param));
        break;
      default:
        break;
    }
    cmd.cmd = 0xff;
  }
}

int main(int argc, char **argv)
{
  int r, opt;
  bool device_serial_specified = false;
  uint64_t device_serial;
  char* addr = "127.0.0.1";
  int port = 1234;
  uint32_t frequency = 100000000;
  //uint8_t clock_status = 0xFF;
  struct sockaddr_in local, remote;
  int gain = 0;
  struct llist *curelem,*prev;
  pthread_attr_t attr;
  void *status;
  struct timeval tv = {1,0};
  struct linger ling = {1,0};
  SOCKET listensocket;
  socklen_t rlen;
  fd_set readfds;
  dongle_info_t dongle_info;
  struct sigaction sigact, sigign;

  while ((opt = getopt(argc, argv, "d:a:p:f:g:s:S:b:n:d:P:TD:Lv")) != -1) {
    switch (opt) {
    case 'd':
      device_serial_specified = true;
      device_serial = (uint64_t)strtol(optarg, NULL, 16);
      break;
    case 'f':
      frequency = (uint32_t)atoi(optarg);
      break;
    case 'g':
      gain = (int)(atof(optarg) * 10); /* tenths of a dB */
      break;
    case 's':
      output_sample_rate = (uint32_t)atoi(optarg);
      break;
    case 'S':
      rf_sample_rate = (uint32_t)atoi(optarg);
      break;
    case 'a':
      addr = optarg;
      break;
    case 'p':
      port = atoi(optarg);
      break;
    case 'n':
      llbuf_num = atoi(optarg);
      break;
    case 'T':
      enable_biastee = 1;
      break;
    case 'P':
      ppm_error = atoi(optarg);
      break;
    case 'D':
      dshift = atoi(optarg);
      break;
    case 'L':
      lock = 1;
      break;
    case 'v':
      verbose = 1;
      break;
    case 'b':
      break;

    default:
      usage();
      break;
    }
  }

  if (argc < optind)
    usage();

  if(device_serial_specified)
  {
    r = airspyhf_open_sn(&dev, device_serial);
    if(verbose) printf("Opening device: 0x%016lx\n", device_serial);
  }
  else
  {
    r = airspyhf_open(&dev);
  }

  if( r != AIRSPYHF_SUCCESS ) {
    fprintf(stderr,"airspy_open() failed: (%d)\n", r);
    ////airspy_exit();
    return -1;
  }

  //r = airspy_set_sample_type(dev, AIRSPY_SAMPLE_TYPE);
  //if( r != AIRSPYHF_SUCCESS ) {
  //  fprintf(stderr,"airspy_set_sample_type() failed: (%d)\n", r);
  //  airspyhf_close(dev);
  //  //airspy_exit();
  //  return -1;
  //}

  //airspy_set_packing(dev, 1);

  r=airspyhf_get_samplerates(dev, &fscount, 0);
  if( r != AIRSPYHF_SUCCESS ) {
    fprintf(stderr,"airspy_get_sample_rate() failed: (%d)\n", r);
    airspyhf_close(dev);
    ////airspy_exit();
    return -1;
  }
  supported_samplerates = (uint32_t *) malloc(fscount * sizeof(uint32_t));
  r=airspyhf_get_samplerates(dev, supported_samplerates, fscount);
  if( r != AIRSPYHF_SUCCESS ) {
    fprintf(stderr,"airspy_get_sample_rate() failed: (%d)\n", r);
    airspyhf_close(dev);
    ////airspy_exit();
    return -1;
  }

  if(rf_sample_rate) {
    fprintf(stderr,"set_samplerate() rf_sample_rate not supported\n");
    //r = set_hw_samplerate(rf_sample_rate);
    //if( r != AIRSPYHF_SUCCESS ) {
    //  fprintf(stderr,"set_samplerate() failed: (%d)\n", r);
    //  airspyhf_close(dev);
    //  //airspy_exit();
    //  return -1;
    //}
    //if(verbose) printf("Hardware samplerate set to: %.06f MS/s\n", rf_sample_rate/1000000.0);
  } 
  //else {
  
    r=airspyhf_set_samplerate(dev, fscount-1);
    if( r != AIRSPYHF_SUCCESS ) {
      fprintf(stderr,"airspy_set_samplerate() failed: (%d)\n", r);
      airspyhf_close(dev);
      //airspy_exit();
      return -1;
    }
  //}

  /* Set the frequency */
  r = airspyhf_set_freq(dev, frequency);
  if( r != AIRSPYHF_SUCCESS ) {
    fprintf(stderr,"airspy_set_freq() failed: (%d)\n", r);
    airspyhf_close(dev);
    //airspy_exit();
    return -1;
  }
  if(verbose) printf("Frequency Set: %.06f MHz\n", frequency/(1000.0*1000.0));

  if (0 == gain) {
    /* Enable automatic gain */
    r=airspyhf_set_hf_agc(dev, 1);
    if( r != AIRSPYHF_SUCCESS ) {
      fprintf(stderr,"airspy_set agc failed: (%d)\n", r);
    }
    if(verbose) printf("Tuner gain set to auto.\n");
  } else {
    r = airspyhf_set_hf_agc(dev,(gain+250)/37);
    if( r != AIRSPYHF_SUCCESS ) {
      fprintf(stderr,"set gains failed: (%d)\n", r);
      airspyhf_close(dev);
      //airspy_exit();
      return -1;
    }
    if(verbose) printf("Tuner gain set to %f dB.\n", gain/10.0);
  }

  //r = airspy_set_rf_bias(dev, enable_biastee);
  //if( r != AIRSPYHF_SUCCESS ) {
  //  fprintf(stderr,"airspy_set_rf_bias() failed: (%d)\n", r);
  //  airspyhf_close(dev);
  //  //airspy_exit();
  //  return -1;
  //}
  if(verbose)
  {
    if(enable_biastee == 1)
    {
      printf("Bias Tee: On\n");
    }
    else
    {
      printf("Bias Tee: Off\n");
    }
  }

  //r = airspy_si5351c_read(dev, 0x00, &clock_status);
  //if( r != AIRSPYHF_SUCCESS ) {
  //  fprintf(stderr,"airspy_si5351c_read() failed: (%d)\n", r);
  //  airspyhf_close(dev);
  //  //airspy_exit();
  //  return -1;
  //} else {
  //  if((clock_status & 0x10) == 0x10) {
  //    if(verbose) printf("Frequency reference: Internal TCXO\n");
  //  } else {
  //   if(verbose) printf("Frequency reference: External Clock\n");
  //  }
  //}

  if(output_sample_rate == 0)
  {
    output_sample_rate = rf_sample_rate;
  }
  int src_error;
  src_state_ptr = src_new(DEFAULT_CONVERTER, 2, &src_error);
  if(src_state_ptr == NULL)
  {
    fprintf(stderr,"error setting up samplerate converter: (%d)\n", src_error);
    airspyhf_close(dev);
    //airspy_exit();
    return -1;
  }
  if(verbose) printf("Output samplerate set to: %.06f MS/s\n", output_sample_rate / (1000.0 * 1000.0));

  if(lock)
  {
    if(verbose) printf("Settings locked.\n");
  }

  sigact.sa_handler = sighandler;
  sigemptyset(&sigact.sa_mask);
  sigact.sa_flags = 0;
  sigign.sa_handler = SIG_IGN;
  sigaction(SIGINT, &sigact, NULL);
  sigaction(SIGTERM, &sigact, NULL);
  sigaction(SIGQUIT, &sigact, NULL);
  sigaction(SIGPIPE, &sigign, NULL);

  pthread_mutex_init(&exit_cond_lock, NULL);
  pthread_mutex_init(&ll_mutex, NULL);
  pthread_mutex_init(&exit_cond_lock, NULL);
  pthread_cond_init(&cond, NULL);
  pthread_cond_init(&exit_cond, NULL);

  memset(&local,0,sizeof(local));
  local.sin_family = AF_INET;
  local.sin_port = htons(port);
  local.sin_addr.s_addr = inet_addr(addr);

  listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  r = 1;
  setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
  setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
  bind(listensocket,(struct sockaddr *)&local,sizeof(local));

  r = fcntl(listensocket, F_GETFL, 0);
  r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);

  while(1) {
    printf("listening...\n");
    printf("Use the device argument 'rtl_tcp=%s:%d' in OsmoSDR "
           "(gr-osmosdr) source\n"
           "to receive samples in GRC and control "
           "parameters (frequency, gain, ...).\n",
           addr, port);
    listen(listensocket,1);

    while(1) {
      FD_ZERO(&readfds);
      FD_SET(listensocket, &readfds);
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      r = select(listensocket+1, &readfds, NULL, NULL, &tv);
      if(do_exit) {
        goto out;
      } else if(r) {
        rlen = sizeof(remote);
        s = accept(listensocket,(struct sockaddr *)&remote, &rlen);
        break;
      }
    }

    setsockopt(s, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
    r=5;setsockopt(s, SOL_SOCKET, SO_PRIORITY, (char *)&r, sizeof(int));

    printf("client accepted!\n");

    memset(&dongle_info, 0, sizeof(dongle_info));
    memcpy(&dongle_info.magic, "RTL0", 4);

    dongle_info.tuner_type = htonl(5);
    dongle_info.tuner_gain_count = htonl(22);

    r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
    if (sizeof(dongle_info) != r) {
      printf("failed to send dongle information\n");
      break;
    }

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
    r = pthread_create(&command_thread, &attr, command_worker, NULL);
    pthread_attr_destroy(&attr);

    fprintf(stderr,"start rx\n");
    r = airspyhf_start(dev, rx_callback, NULL);
    if( r != AIRSPYHF_SUCCESS ) {
            fprintf(stderr,"airspy_start_rx() failed: (%d)\n", r);
      break;
    }

    pthread_join(tcp_worker_thread, &status);
    pthread_join(command_thread, &status);

    close(s);

    fprintf(stderr,"stop rx\n");

    r = airspyhf_stop(dev);
    if( r != AIRSPYHF_SUCCESS ) {
      fprintf(stderr,"airspy_stop_rx() failed: (%d)\n", r);
      break;
    }

    curelem = ls_buffer;
    while(curelem != 0) {
      prev = curelem;
      curelem = curelem->next;
      free(prev->data);
      free(prev);
    }
    ls_buffer=le_buffer=NULL;
    global_numq = 0;

    do_exit = 0;
  }

out:
  airspyhf_close(dev);
  //airspy_exit();
  close(listensocket);
  close(s);
  src_delete(src_state_ptr);
  printf("bye!\n");
  return r >= 0 ? r : -r;
}
