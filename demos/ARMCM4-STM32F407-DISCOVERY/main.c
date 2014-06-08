/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
//#include "test.h"
#include <string.h>
#include "chprintf.h"
#include "usbcfg.h"

#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/netbuf.h"
#include "lwip/netifapi.h"
#include "lwip/mem.h"
#include "lwip/sio.h"

#include <lwip/tcpip.h>
#include <netif/slipif.h>

static const unsigned char BitReverseTable64[] =
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC
};

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

static void pwmpcb(PWMDriver *pwmp);
static void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void spicb(SPIDriver *spip);

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS   4

/* Depth of the conversion buffer, channels are sampled four times each.*/
#define ADC_GRP1_BUF_DEPTH      64

/*
 * ADC samples buffer.
 */
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 2 channels, SW triggered.
 * Channels:    IN11   (48 cycles sample time)
 *              Sensor (192 cycles sample time)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  adccb,
  NULL,
  /* HW dependent part.*/
  0,
  ADC_CR2_SWSTART,
  ADC_SMPR2_SMP_AN4(ADC_SAMPLE_28) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_28) | \
  ADC_SMPR2_SMP_AN6(ADC_SAMPLE_28) | ADC_SMPR2_SMP_AN7(ADC_SAMPLE_28),
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN5) | \
  ADC_SQR3_SQ3_N(ADC_CHANNEL_IN6) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN7)
};

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 1 and 4 enabled without callbacks,
 * the active state is a logic one.
 */
static PWMConfig pwmcfg = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  10000,                                    /* PWM period 1S (in ticks).    */
  pwmpcb,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  /* HW dependent part.*/
  0,
  0
};

/*
 * SPI2 configuration structure.
 * Speed 5.25MHz, CPHA=0, CPOL=0, 16bits frames, LSb transmitted first.
 * The slave select line is the pin 12 on the port GPIOB.
 */
static const SPIConfig spi2cfg =
  { NULL,
  /* HW dependent part.*/
  GPIOB, 12,
  SPI_CR1_DFF | SPI_CR1_LSBFIRST |SPI_CR1_BR_1 };

struct netconn *xUdpConn;

/*
 * PWM cyclic callback.
 * A new ADC conversion is started.
 */
static void pwmpcb(PWMDriver *pwmp) {

  (void)pwmp;

  /* Starts an asynchronous ADC conversion operation, the conversion
     will be executed in parallel to the current PWM cycle and will
     terminate before the next PWM cycle.*/
  chSysLockFromIsr();
  adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
  chSysUnlockFromIsr();
}

/*
 * ADC end conversion callback.
 * The PWM channels are reprogrammed using the latest ADC samples.
 * The latest samples are transmitted into a single SPI transaction.
 */
void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void) buffer; (void) n;
  /* Note, only in the ADC_COMPLETE state because the ADC driver fires an
     intermediate callback when the buffer is half full.*/
  if (adcp->state == ADC_COMPLETE) {
	  struct netbuf  *xNetBuf;
	  struct ip_addr ipaddr;

	  IP4_ADDR(&ipaddr, 192,168,222,1);

	  xNetBuf = netbuf_new ();
	  netbuf_ref ( xNetBuf, buffer, sizeof(samples));

	  netconn_sendto(xUdpConn, xNetBuf, &ipaddr, 1234);
	  netbuf_delete(xNetBuf);

  }
}

/*
 * SPI end transfer callback.
 */
static void spicb(SPIDriver *spip) {

  /* On transfer end just releases the slave select line.*/
  chSysLockFromIsr();
  spiUnselectI(spip);
  chSysUnlockFromIsr();
}

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");

  /* Network interface variables */
  struct ip_addr ipaddr, netmask, gw;
  struct netif netif;
  //struct netbuf  *xNetBuf;
  struct netbuf  *RecvBuf;
  //void *xBuf;
  int i;
  struct netcommand{
	  uint8_t type;
	  uint8_t addr;
	  uint32_t command;
  } netcommand_buf;

  enum netcommand_type{
	  set_syn = 0,
	  set_RxAtt = 1,
	  set_TxAtt = 2,
	  set_LOAtt = 3,
	  set_InSel = 4,
	  set_OOK = 5,
	  get_vsence = 6,
	  set_reportOpt = 7,
	  set_TxPwrOn = 8
  };

#define PWR_MAGIC 0xA0A0BEEF

  chThdSleepMilliseconds(500);

  IP4_ADDR(&gw, 192,168,222,1);
  IP4_ADDR(&ipaddr, 192,168,222,2);
  IP4_ADDR(&netmask, 255,255,255,0);

  tcpip_init(NULL, NULL);

  netifapi_netif_add(&netif, &ipaddr, &netmask, &gw, NULL, slipif_init, tcpip_input);
  netif_set_default(&netif);
  netif_set_up(&netif);


  xUdpConn = netconn_new( NETCONN_UDP );
  netconn_bind(xUdpConn, &ipaddr, 1234);

  //xBuf = (void*)mem_malloc(10);
  //memset(xBuf,0xAA,10);
  //xNetBuf = netbuf_new ();
  //netbuf_ref ( xNetBuf, xBuf, 10 );

  //IP4_ADDR(&ipaddr, 192,168,222,1);

  while (TRUE) {
    //chThdSleepMilliseconds(1500);
    //palSetPad(GPIOD, GPIOD_LED3);       /* Orange.  */
    netconn_recv (xUdpConn, &RecvBuf);

    if(netbuf_len(RecvBuf) == sizeof(struct netcommand)){
    	netbuf_copy(RecvBuf, &netcommand_buf, sizeof(netcommand_buf));
    	netcommand_buf.command = ntohl(netcommand_buf.command);
    }
    else{
    	netbuf_delete(RecvBuf);
    	continue;
    }

    switch((enum netcommand_type)netcommand_buf.type){
    	case(set_syn):
			spiSelect(&SPID2);
    		spiSend(&SPID2, 2, (uint16_t *)&(netcommand_buf.command));
    		spiUnselect(&SPID2);
			break;
    	case(set_TxAtt):
			netcommand_buf.command = BitReverseTable64[netcommand_buf.command&(63)]<<8;
			spiSend(&SPID2, 1, &(netcommand_buf.command));
			palSetPad(GPIOD, 8);
			while(!palReadPad(GPIOD, 8));
			palClearPad(GPIOD, 8);
			break;
    	case(set_RxAtt):
			palClearPort(GPIOD, (31)<<1);
			palSetPort(GPIOD, (netcommand_buf.command&(31))<<1);
			if(netcommand_buf.addr == 1){
				palSetPad(GPIOD, 0);
				while(!palReadPad(GPIOD, 0));
				palClearPad(GPIOD, 0);
			}
			if(netcommand_buf.addr<=4 && netcommand_buf.addr>=2){
				palSetPad(GPIOC, 14-netcommand_buf.addr);
				while(!palReadPad(GPIOC, 14-netcommand_buf.addr));
				palClearPad(GPIOC, 14-netcommand_buf.addr);
			}
    		break;
    	case(set_LOAtt):
    		palClearPort(GPIOD, (31)<<11);
    		palSetPort(GPIOD, (netcommand_buf.command&(31))<<11);
    		palSetPad(GPIOD, 9);
    		while(!palReadPad(GPIOD, 9));
    		palClearPad(GPIOD, 9);
    		break;
    	case(set_InSel):
    		if(netcommand_buf.command&0x2){
    			palSetPad(GPIOB, 8);
    			palClearPad(GPIOB, 9);
    		}
    		else{
    			palSetPad(GPIOB, 9);
    			palClearPad(GPIOB, 8);
    		}
    		if(netcommand_buf.command&0x1){
    	    	palSetPad(GPIOE, 0);
    	    	palClearPad(GPIOE, 1);
    	    }
    	    else{
    	    	palSetPad(GPIOE, 1);
    	    	palClearPad(GPIOE, 0);
    	    }

			break;
    	case(set_TxPwrOn):
    		if(netcommand_buf.command == PWR_MAGIC){
    			palSetPad(GPIOE, 11);
    		}
    		else{
    			palClearPad(GPIOE, 11);
    		}
    		break;
    	case(set_OOK):
    	    if(netcommand_buf.command == PWR_MAGIC){
    	    	palClearPad(GPIOE, 12);
    	    	palSetPad(GPIOE, 13);
    	    }
    	    else{
    	    	palSetPad(GPIOE, 12);
    	    	palClearPad(GPIOE, 13);
    	    }
    	    break;
    	case(set_reportOpt):
			adcStartConversion(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
    		break;
    	default:
    		break;


    }

    netbuf_delete(RecvBuf);
    //palClearPad(GPIOD, GPIOD_LED3);    /* Orange.  */
  }

}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */

  halInit();
  chSysInit();

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /*
   * If the user button is pressed after the reset then the test suite is
   * executed immediately before activating the various device drivers in
   * order to not alter the benchmark scores.
   */
  //if (palReadPad(GPIOA, GPIOA_BUTTON))
    //TestThread(&SD2);

  /*
   * Initializes the SPI driver 2. The SPI2 signals are routed as follow:
   * PB12 - NSS.
   * PB13 - SCK.
   * PB14 - MISO.
   * PB15 - MOSI.
   */
  spiStart(&SPID2, &spi2cfg);
  palSetPad(GPIOB, 12);
  palClearPad(GPIOD, 8);
  //palClearPad(GPIOD, 9);
  palClearPad(GPIOE, 11);
  palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);           /* NSS.     */
  palSetPadMode(GPIOD, 8, PAL_MODE_OUTPUT_PUSHPULL |
		                   PAL_STM32_OSPEED_HIGHEST);           /* AttLE    */
  palSetPadMode(GPIOD, 9, PAL_MODE_OUTPUT_PUSHPULL |
  		                   PAL_STM32_OSPEED_HIGHEST);           /* Rx LO AttLE */
  palSetGroupMode(GPIOD, 0xF800, 0, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);           /*MTx[C8:C0p5] */
  palSetGroupMode(GPIOD, 0x003F, 0, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);           /*MRx[C8:C0p5] + Ant1LEin*/
  palSetGroupMode(GPIOC, 0x1C00, 0, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);           /*Ant(2-4)LEin*/
  palSetGroupMode(GPIOE, 0x0003, 0, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);           /*InSel[1:0]*/
  palSetGroupMode(GPIOB, 0x0300, 0, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);           /*InSel[3:2]*/
  palSetGroupMode(GPIOE, 0x3800, 0, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);           /*PAEN,OOKONOFF*/
  palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5) |
                           PAL_STM32_OSPEED_HIGHEST);           /* SCK.     */
  palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(5));              /* MISO.    */
  palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(5) |
                           PAL_STM32_OSPEED_HIGHEST);           /* MOSI.    */

  /*
   * Initializes the ADC driver 1 and enable the thermal sensor.
   * The pin PC1 on the port GPIOC is programmed as analog input.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();
  palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
  palSetGroupMode(GPIOA, 0x00F0, 0, PAL_MODE_INPUT_ANALOG);           /*A[7:4] */

  /*
   * Initializes the PWM driver 4, routes the TIM4 outputs to the board LEDs.
   */
  //pwmStart(&PWMD4, &pwmcfg);
  //palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));  /* Green.   */
  //palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_ALTERNATE(2));  /* Blue.    */

  /*
   * Creates the example thread.
   */

  chThdSleepMilliseconds(2000);
  if (SDU1.config->usbp->state == USB_SELECTED || SDU1.config->usbp->state == USB_ACTIVE) {
      sio_set_serial_driver(&SDU1);
  }
  else{
	  sio_set_serial_driver(NETCOM);
  }

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched with output on the serial
   * driver 2.
   */

  while (TRUE) {
    chThdSleepMilliseconds(500);
  }
}

