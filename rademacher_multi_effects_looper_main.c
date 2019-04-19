/*
 * Copyright © 2019 Patrick Rademacher
 * [This program is licensed under the "MIT License"]
 * Please see the file LICENSE in the source
 * distribution of this software for license terms.
 */

//Patrick Rademacher (c) 2019
//Professor Bryant York
//Portland State University
//March 12, 2019

/ CC-by-www.Electrosmash.com open-source project.

#include "delaylines.h" 
#include <stdio.h>
#include <bcm2835.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

// Define Input Pins
#define PUSH1                   RPI_GPIO_P1_08          //GPIO14
#define PUSH2                   RPI_V2_GPIO_P1_38       //GPIO20 
#define TOGGLE_SWITCH   RPI_V2_GPIO_P1_32       //GPIO12
#define FOOT_SWITCH     RPI_GPIO_P1_10          //GPIO15
#define LED                     RPI_V2_GPIO_P1_36       //GPIO16
#define MAX_LOOP 8000000
#define sample_delay 10
#define PI 3.14159265

uint32_t LFO_sine_wave[] = {
0x800,0x80c,0x819,0x826,0x833,0x840,0x84d,0x85a,0x866,0x873,0x880,0x88d,0x89a,0x8a7,0x8b3,0x8c0,
0x8cd,0x8da,0x8e7,0x8f3,0x900,0x90d,0x91a,0x926,0x933,0x940,0x94c,0x959,0x966,0x973,0x97f,0x98c,
0x998,0x9a5,0x9b2,0x9be,0x9cb,0x9d7,0x9e4,0x9f0,0x9fd,0xa09,0xa16,0xa22,0xa2e,0xa3b,0xa47,0xa53,
0xa60,0xa6c,0xa78,0xa84,0xa91,0xa9d,0xaa9,0xab5,0xac1,0xacd,0xad9,0xae5,0xaf1,0xafd,0xb09,0xb15,
0xb21,0xb2d,0xb38,0xb44,0xb50,0xb5c,0xb67,0xb73,0xb7e,0xb8a,0xb96,0xba1,0xbac,0xbb8,0xbc3,0xbcf,
0xbda,0xbe5,0xbf0,0xbfc,0xc07,0xc12,0xc1d,0xc28,0xc33,0xc3e,0xc49,0xc53,0xc5e,0xc69,0xc74,0xc7e,
0xc89,0xc94,0xc9e,0xca9,0xcb3,0xcbd,0xcc8,0xcd2,0xcdc,0xce6,0xcf1,0xcfb,0xd05,0xd0f,0xd19,0xd23,
0xd2c,0xd36,0xd40,0xd4a,0xd53,0xd5d,0xd66,0xd70,0xd79,0xd82,0xd8c,0xd95,0xd9e,0xda7,0xdb0,0xdb9,
0xdc2,0xdcb,0xdd4,0xddd,0xde6,0xdee,0xdf7,0xdff,0xe08,0xe10,0xe19,0xe21,0xe29,0xe31,0xe39,0xe41,
0xe49,0xe51,0xe59,0xe61,0xe69,0xe70,0xe78,0xe7f,0xe87,0xe8e,0xe96,0xe9d,0xea4,0xeab,0xeb2,0xeb9,
0xec0,0xec7,0xece,0xed5,0xedb,0xee2,0xee8,0xeef,0xef5,0xefc,0xf02,0xf08,0xf0e,0xf14,0xf1a,0xf20,
0xf26,0xf2b,0xf31,0xf37,0xf3c,0xf42,0xf47,0xf4c,0xf51,0xf57,0xf5c,0xf61,0xf66,0xf6a,0xf6f,0xf74,
0xf79,0xf7d,0xf82,0xf86,0xf8a,0xf8f,0xf93,0xf97,0xf9b,0xf9f,0xfa3,0xfa6,0xfaa,0xfae,0xfb1,0xfb5,
0xfb8,0xfbb,0xfbf,0xfc2,0xfc5,0xfc8,0xfcb,0xfce,0xfd0,0xfd3,0xfd6,0xfd8,0xfdb,0xfdd,0xfdf,0xfe2,
0xfe4,0xfe6,0xfe8,0xfea,0xfeb,0xfed,0xfef,0xff0,0xff2,0xff3,0xff5,0xff6,0xff7,0xff8,0xff9,0xffa,
0xffb,0xffc,0xffc,0xffd,0xffe,0xffe,0xffe,0xfff,0xfff,0xfff,0xfff,0xfff,0xfff,0xfff,0xffe,0xffe,
0xffe,0xffd,0xffc,0xffc,0xffb,0xffa,0xff9,0xff8,0xff7,0xff6,0xff5,0xff3,0xff2,0xff0,0xfef,0xfed,
0xfeb,0xfea,0xfe8,0xfe6,0xfe4,0xfe2,0xfdf,0xfdd,0xfdb,0xfd8,0xfd6,0xfd3,0xfd0,0xfce,0xfcb,0xfc8,
0xfc5,0xfc2,0xfbf,0xfbb,0xfb8,0xfb5,0xfb1,0xfae,0xfaa,0xfa6,0xfa3,0xf9f,0xf9b,0xf97,0xf93,0xf8f,
0xf8a,0xf86,0xf82,0xf7d,0xf79,0xf74,0xf6f,0xf6a,0xf66,0xf61,0xf5c,0xf57,0xf51,0xf4c,0xf47,0xf42,
0xf3c,0xf37,0xf31,0xf2b,0xf26,0xf20,0xf1a,0xf14,0xf0e,0xf08,0xf02,0xefc,0xef5,0xeef,0xee8,0xee2,
0xedb,0xed5,0xece,0xec7,0xec0,0xeb9,0xeb2,0xeab,0xea4,0xe9d,0xe96,0xe8e,0xe87,0xe7f,0xe78,0xe70,
0xe69,0xe61,0xe59,0xe51,0xe49,0xe41,0xe39,0xe31,0xe29,0xe21,0xe19,0xe10,0xe08,0xdff,0xdf7,0xdee,
0xde6,0xddd,0xdd4,0xdcb,0xdc2,0xdb9,0xdb0,0xda7,0xd9e,0xd95,0xd8c,0xd82,0xd79,0xd70,0xd66,0xd5d,
0xd53,0xd4a,0xd40,0xd36,0xd2c,0xd23,0xd19,0xd0f,0xd05,0xcfb,0xcf1,0xce6,0xcdc,0xcd2,0xcc8,0xcbd,
0xcb3,0xca9,0xc9e,0xc94,0xc89,0xc7e,0xc74,0xc69,0xc5e,0xc53,0xc49,0xc3e,0xc33,0xc28,0xc1d,0xc12,
0xc07,0xbfc,0xbf0,0xbe5,0xbda,0xbcf,0xbc3,0xbb8,0xbac,0xba1,0xb96,0xb8a,0xb7e,0xb73,0xb67,0xb5c,
0xb50,0xb44,0xb38,0xb2d,0xb21,0xb15,0xb09,0xafd,0xaf1,0xae5,0xad9,0xacd,0xac1,0xab5,0xaa9,0xa9d,
0xa91,0xa84,0xa78,0xa6c,0xa60,0xa53,0xa47,0xa3b,0xa2e,0xa22,0xa16,0xa09,0x9fd,0x9f0,0x9e4,0x9d7,
0x9cb,0x9be,0x9b2,0x9a5,0x998,0x98c,0x97f,0x973,0x966,0x959,0x94c,0x940,0x933,0x926,0x91a,0x90d,
0x900,0x8f3,0x8e7,0x8da,0x8cd,0x8c0,0x8b3,0x8a7,0x89a,0x88d,0x880,0x873,0x866,0x85a,0x84d,0x840,
0x833,0x826,0x819,0x80c,0x800,0x7f3,0x7e6,0x7d9,0x7cc,0x7bf,0x7b2,0x7a5,0x799,0x78c,0x77f,0x772,
0x765,0x758,0x74c,0x73f,0x732,0x725,0x718,0x70c,0x6ff,0x6f2,0x6e5,0x6d9,0x6cc,0x6bf,0x6b3,0x6a6,
0x699,0x68c,0x680,0x673,0x667,0x65a,0x64d,0x641,0x634,0x628,0x61b,0x60f,0x602,0x5f6,0x5e9,0x5dd,
0x5d1,0x5c4,0x5b8,0x5ac,0x59f,0x593,0x587,0x57b,0x56e,0x562,0x556,0x54a,0x53e,0x532,0x526,0x51a,
0x50e,0x502,0x4f6,0x4ea,0x4de,0x4d2,0x4c7,0x4bb,0x4af,0x4a3,0x498,0x48c,0x481,0x475,0x469,0x45e,
0x453,0x447,0x43c,0x430,0x425,0x41a,0x40f,0x403,0x3f8,0x3ed,0x3e2,0x3d7,0x3cc,0x3c1,0x3b6,0x3ac,
0x3a1,0x396,0x38b,0x381,0x376,0x36b,0x361,0x356,0x34c,0x342,0x337,0x32d,0x323,0x319,0x30e,0x304,
0x2fa,0x2f0,0x2e6,0x2dc,0x2d3,0x2c9,0x2bf,0x2b5,0x2ac,0x2a2,0x299,0x28f,0x286,0x27d,0x273,0x26a,
0x261,0x258,0x24f,0x246,0x23d,0x234,0x22b,0x222,0x219,0x211,0x208,0x200,0x1f7,0x1ef,0x1e6,0x1de,
0x1d6,0x1ce,0x1c6,0x1be,0x1b6,0x1ae,0x1a6,0x19e,0x196,0x18f,0x187,0x180,0x178,0x171,0x169,0x162,
0x15b,0x154,0x14d,0x146,0x13f,0x138,0x131,0x12a,0x124,0x11d,0x117,0x110,0x10a,0x103,0xfd,0xf7,
0xf1,0xeb,0xe5,0xdf,0xd9,0xd4,0xce,0xc8,0xc3,0xbd,0xb8,0xb3,0xae,0xa8,0xa3,0x9e,0x99,0x95,0x90,
0x8b,0x86,0x82,0x7d,0x79,0x75,0x70,0x6c,0x68,0x64,0x60,0x5c,0x59,0x55,0x51,0x4e,0x4a,0x47,0x44,
0x40,0x3d,0x3a,0x37,0x34,0x31,0x2f,0x2c,0x29,0x27,0x24,0x22,0x20,0x1d,0x1b,0x19,0x17,0x15,0x14,
0x12,0x10,0xf,0xd,0xc,0xa,0x9,0x8,0x7,0x6,0x5,0x4,0x3,0x3,0x2,0x1,0x1,0x1,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x1,0x1,0x1,0x2,0x3,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xa,0xc,0xd,0xf,0x10,0x12,0x14,0x15,0x17,
0x19,0x1b,0x1d,0x20,0x22,0x24,0x27,0x29,0x2c,0x2f,0x31,0x34,0x37,0x3a,0x3d,0x40,0x44,0x47,0x4a,
0x4e,0x51,0x55,0x59,0x5c,0x60,0x64,0x68,0x6c,0x70,0x75,0x79,0x7d,0x82,0x86,0x8b,0x90,0x95,0x99,
0x9e,0xa3,0xa8,0xae,0xb3,0xb8,0xbd,0xc3,0xc8,0xce,0xd4,0xd9,0xdf,0xe5,0xeb,0xf1,0xf7,0xfd,0x103,
0x10a,0x110,0x117,0x11d,0x124,0x12a,0x131,0x138,0x13f,0x146,0x14d,0x154,0x15b,0x162,0x169,0x171,
0x178,0x180,0x187,0x18f,0x196,0x19e,0x1a6,0x1ae,0x1b6,0x1be,0x1c6,0x1ce,0x1d6,0x1de,0x1e6,0x1ef,
0x1f7,0x200,0x208,0x211,0x219,0x222,0x22b,0x234,0x23d,0x246,0x24f,0x258,0x261,0x26a,0x273,0x27d,
0x286,0x28f,0x299,0x2a2,0x2ac,0x2b5,0x2bf,0x2c9,0x2d3,0x2dc,0x2e6,0x2f0,0x2fa,0x304,0x30e,0x319,
0x323,0x32d,0x337,0x342,0x34c,0x356,0x361,0x36b,0x376,0x381,0x38b,0x396,0x3a1,0x3ac,0x3b6,0x3c1,
0x3cc,0x3d7,0x3e2,0x3ed,0x3f8,0x403,0x40f,0x41a,0x425,0x430,0x43c,0x447,0x453,0x45e,0x469,0x475,
0x481,0x48c,0x498,0x4a3,0x4af,0x4bb,0x4c7,0x4d2,0x4de,0x4ea,0x4f6,0x502,0x50e,0x51a,0x526,0x532,
0x53e,0x54a,0x556,0x562,0x56e,0x57b,0x587,0x593,0x59f,0x5ac,0x5b8,0x5c4,0x5d1,0x5dd,0x5e9,0x5f6,
0x602,0x60f,0x61b,0x628,0x634,0x641,0x64d,0x65a,0x667,0x673,0x680,0x68c,0x699,0x6a6,0x6b3,0x6bf,
0x6cc,0x6d9,0x6e5,0x6f2,0x6ff,0x70c,0x718,0x725,0x732,0x73f,0x74c,0x758,0x765,0x772,0x77f,0x78c,
0x799,0x7a5,0x7b2,0x7bf,0x7cc,0x7d9,0x7e6,0x7f3,0x800,}; 


uint8_t FOOT_SWITCH_val;
uint8_t TOGGLE_SWITCH_val;
uint8_t PUSH1_val;
uint8_t PUSH2_val;
uint32_t loop_buffer[MAX_LOOP];
uint32_t phase_loop_buffer[MAX_LOOP]; 
uint32_t flang_loop_buffer[MAX_LOOP];
uint32_t pog_loop_buffer[MAX_LOOP];
uint32_t rad_loop_buffer[MAX_LOOP];
int flang_delay_1 = 10;
int flang_delay_2 = 20;
int flang_delay_3 = 30;
int flang_delay_4 = 40;
int flang_delay_5 = 50;
int flang_delay_6 = 60;
int flang_delay_7 = 70;
int flang_delay_8 = 80;
int flang_delay_9 = 90;
int flang_delay_10 = 100;

int main(int argc, char** argv) {
       
    uint32_t loop_cue = 0;
    uint32_t loop_read = 0;
    uint32_t no_loop_output = 0;
    uint32_t with_loop_output = 0;
    uint32_t read_timer = 0;
    uint32_t final_loop_read = 0;
    uint32_t dry_signal = 0;
    uint32_t phased_signal = 0;
    int number_of_filters = 16;
    double * wet_signal = (double*)malloc((number_of_filters + 1)*sizeof(double));
    double ** IST = (double**)malloc(number_of_filters * sizeof(double*));
    double ** OST = (double**)malloc(number_of_filters * sizeof(double*));
    int * ISC = (int*)malloc(number_of_filters*sizeof(int));
    int * OSC = (int*)malloc(number_of_filters*sizeof(int));
    double gain = 0;
	uint32_t flang_signal = 0;
    double * flang_table_1 = (double*)malloc(10 * sizeof(double));
    double * flang_table_2 = (double*)malloc(20 * sizeof(double));
    double * flang_table_3 = (double*)malloc(30 * sizeof(double));
    double * flang_table_4 = (double*)malloc(40 * sizeof(double));
    double * flang_table_5 = (double*)malloc(50 * sizeof(double));
    double * flang_table_6 = (double*)malloc(60 * sizeof(double));
    double * flang_table_7 = (double*)malloc(70 * sizeof(double));
    double * flang_table_8 = (double*)malloc(80 * sizeof(double));
    double * flang_table_9 = (double*)malloc(90 * sizeof(double));
    double * flang_table_10 = (double*)malloc(100 * sizeof(double));
    double * flang_wet_signal = (double*)malloc(10 * sizeof(double));
    int * ISC_2 = (int*)malloc(10 * sizeof(int));
    int delay_switch_counter = 0;
    double flang_gain = .5;
    int index = 0;
    double LFO_rate = 200;
    int LFO_sample = 0;
    double LFO_counter = 1;
    uint32_t delay_buffer[3000];
    uint32_t pog_signal = 0;
    uint32_t pog_value = 1;
    uint32_t delay_depth = 5000; //default starting delay is 100000 is 0.25 sec approx.
    int  delay_read[6] = {0, 0, 0, 0, 0, 0};
    double delay_read_double[6] = {0, 0, 0, 0, 0, 0};
    int delay_write = 0;
    double val = PI/180;
    double rad_signal = 0;
    int i = 0;
    int j = 0;
    int loop_effect = 0;
    int signal_effect = 0;
    for (i = 0; i < number_of_filters; i++)
    {
        IST[i] = (double *)malloc(sample_delay * sizeof(double));
        OST[i] = (double *)malloc(sample_delay * sizeof(double));
        wet_signal[i] = 0;
        ISC[i] = 0;
        OSC[i] = 0;
            for (j = 0; j < sample_delay; j++)
            {
                IST[i][j] = 0;
                OST[i][j] = 0;

            }
    }

    for (i = 0; i < 10; i++)
    {
        flang_table_1[i] = 0;
    }

    for (i = 0; i < 20; i++)
    {
    	flang_table_2[i] = 0;
    }

    for (i = 0; i < 30; i++)
    {
    	flang_table_3[i] = 0;
    }

    for (i = 0; i < 40; i++)
    { 
        flang_table_4[i] = 0;
    }

    for (i = 0; i < 50; i++)
    { 
    	flang_table_4[i] = 0;
    }

    for (i = 0; i < 60; i++)
    { 
    	flang_table_5[i] = 0;
    }

    for (i = 0; i < 70; i++)
    { 
        flang_table_5[i] = 0;
    }

    for (i = 0; i < 80; i++)
    { 
        flang_table_6[i] = 0;
    }

    for (i = 0; i < 90; i++)
    {    
        flang_table_9[i] = 0;
    }

    for (i = 0; i < 100; i++)
    { 
        flang_table_10[i] = 0;
    }

    for (i = 0; i < 10; i++)
    { 
        ISC_2[i] = 0;
        flang_wet_signal[i] = 0;
    }




    // Start the BCM2835 Library to access GPIO.
    if (!bcm2835_init())
    {
		printf("bcm2835_init failed. Are you running as root??\n");
       return 1;
	}
        // Start the SPI BUS.
    if (!bcm2835_spi_begin())
    {
		printf("bcm2835_spi_begin failed. Are you running as root??\n");
        return 1;
	}
 
//define PWM    
    bcm2835_gpio_fsel(18,BCM2835_GPIO_FSEL_ALT5 ); //PWM0 signal on GPIO18    
    bcm2835_gpio_fsel(13,BCM2835_GPIO_FSEL_ALT0 ); //PWM1 signal on GPIO13    
    bcm2835_pwm_set_clock(2); // Max clk frequency (19.2MHz/2 = 9.6MHz)
    bcm2835_pwm_set_mode(0,1 , 1); //channel 0, markspace mode, PWM enabled. 
    bcm2835_pwm_set_range(0,64);   //channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
    bcm2835_pwm_set_mode(1, 1, 1); //channel 1, markspace mode, PWM enabled.
    bcm2835_pwm_set_range(1,64);   //channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
 
        //define SPI bus configuration
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);    // 4MHz clock with _64 
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
 
    uint8_t mosi[10] = { 0x01, 0x00, 0x00 }; //12 bit ADC read 0x08 ch0, - 0c for ch1 
    uint8_t miso[10] = { 0 };
 
    //Define GPIO pins configuration
    bcm2835_gpio_fsel(PUSH1, BCM2835_GPIO_FSEL_INPT);                   //PUSH1 button as input
    bcm2835_gpio_fsel(PUSH2, BCM2835_GPIO_FSEL_INPT);                       //PUSH2 button as input
    bcm2835_gpio_fsel(TOGGLE_SWITCH, BCM2835_GPIO_FSEL_INPT);       //TOGGLE_SWITCH as input
    bcm2835_gpio_fsel(FOOT_SWITCH, BCM2835_GPIO_FSEL_INPT);         //FOOT_SWITCH as input
    bcm2835_gpio_fsel(LED, BCM2835_GPIO_FSEL_OUTP);                         //LED as output
 
    bcm2835_gpio_set_pud(PUSH1, BCM2835_GPIO_PUD_UP);           //PUSH1 pull-up enabled   
    bcm2835_gpio_set_pud(PUSH2, BCM2835_GPIO_PUD_UP);           //PUSH2 pull-up enabled 
    bcm2835_gpio_set_pud(TOGGLE_SWITCH, BCM2835_GPIO_PUD_UP);   //TOGGLE_SWITCH pull-up enabled 
    bcm2835_gpio_set_pud(FOOT_SWITCH, BCM2835_GPIO_PUD_UP);     //FOOT_SWITCH pull-up enabled 
 
 	while(1)
 	{
        //read 12 bits ADC
    	bcm2835_spi_transfernb(mosi, miso, 3);
    	dry_signal = miso[2] + ((miso[1] & 0x0F) << 8); 
 
                //Read the PUSH buttons every 50000 times (0.25s) to save resources.
        read_timer++;
        if (read_timer==3000)
        {
        	read_timer=0;
        	PUSH1_val = bcm2835_gpio_lev(PUSH1);
        	PUSH2_val = bcm2835_gpio_lev(PUSH2);
    		TOGGLE_SWITCH_val = bcm2835_gpio_lev(TOGGLE_SWITCH);
        	FOOT_SWITCH_val = bcm2835_gpio_lev(FOOT_SWITCH);
        	//light the effect when the footswitch is activated.
        	bcm2835_gpio_write(LED,!FOOT_SWITCH_val); 
                                

                                
			if (PUSH2_val == 0 && loop_cue == 0)
        	{   
        		bcm2835_delay(100); //100ms delay for buttons debouncing. 
            	printf("\nstart of loop\n");
            	loop_read = 0;
            	loop_cue = 1;
        	}

        	else if (PUSH2_val == 0 && loop_cue == 1)
        	{
        		bcm2835_delay(100); //100ms delay for buttons debouncing.
            	printf("\nend of loop\n");
            	final_loop_read = loop_read;
            	loop_read = 0;
            	loop_cue = 2;
        	}

        	else if (PUSH2_val == 0 && loop_cue == 2)
        	{
        		bcm2835_delay(100); //100ms delay for buttons debouncing.
            	loop_read = 0;
            	loop_cue = 0;
         	}

        	if (PUSH1_val==0) 
        	{               
        		bcm2835_delay(100); //100ms delay for buttons debouncing.
            	if (LFO_rate == 200)
            	{
            		LFO_rate = 10;
            	}   
            
				else if (LFO_rate >= 10 && LFO_rate <20)
            	{
            		LFO_rate += 1;
            	}
                    
				else if (LFO_rate == 20)
            	{
            		LFO_rate = 22;
            	}
            
				else if (LFO_rate == 22)
            	{
            		LFO_rate = 25;
            	}
                    
				else if (LFO_rate == 25)
            	{
            		LFO_rate = 28;
            	}
                    
				else if (LFO_rate == 28)
            	{
            		LFO_rate = 33;
            	}
                    
				else if (LFO_rate == 33)
            	{
            		LFO_rate = 40;
            	}

            	else if (LFO_rate == 40)
            	{
            		LFO_rate = 50;
            	}
                    
				else if (LFO_rate == 50)
            	{
             		LFO_rate = 66;
            	}
                    
				else if (LFO_rate == 66)
            	{
            		LFO_rate = 100;
            	}	
                    
				else if (LFO_rate == 100)
            	{
            		LFO_rate = 200;
            	}
           
				if (pog_value<12)
            	{       
            		pog_value++;
            	}
                                        
				else if (pog_value == 12)
            	{
            		pog_value = 1;
            	}                                
        	}	
		}

		LFO_counter++;
    	if (LFO_counter >= LFO_rate)
    	{
        	LFO_counter = 1;
        	LFO_sample ++;
    	}

    	if (LFO_sample == 999)
   		{
        	LFO_sample = 1;
    	}
    
		index = 0;
    	wet_signal[index] = (double) dry_signal;
    	gain = .2;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index); 
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);
    	index += 1;
    	gain += .05;
    	all_pass_filter(&wet_signal[index], &wet_signal[index + 1], gain, IST, OST, ISC, OSC, index);

    	wet_signal[index + 1] = wet_signal[index + 1] * (LFO_sine_wave[LFO_sample]/4095.0);
    	phased_signal = (uint32_t) ((wet_signal[index + 1] * 1.1) + (dry_signal * .7));
    	phased_signal = (phased_signal >> 1) * 1.4;
 
        
    	index = 0;
    	flang_gain = .95;
    	flang_wet_signal[0] = (double) dry_signal;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[1], flang_gain, flang_table_1, ISC_2, flang_delay_1, index);
    	flang_wet_signal[1] = flang_wet_signal[1] * (LFO_sine_wave[LFO_sample]/4095.0);
    	index += 1;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[2], flang_gain, flang_table_2, ISC_2, flang_delay_2, index);
    	flang_wet_signal[2] = flang_wet_signal[2] * (LFO_sine_wave[LFO_sample]/4095.0);
    	index += 1;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[3], flang_gain, flang_table_3, ISC_2, flang_delay_3, index);
    	flang_wet_signal[3] = flang_wet_signal[3] * (LFO_sine_wave[LFO_sample]/4095.0);
    	index += 1;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[4], flang_gain, flang_table_4, ISC_2, flang_delay_4, index);
    	flang_wet_signal[4] = flang_wet_signal[4] * (LFO_sine_wave[LFO_sample]/4095.0);
    	index += 1;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[5], flang_gain, flang_table_5, ISC_2, flang_delay_5, index);
    	flang_wet_signal[5] = flang_wet_signal[5] * (LFO_sine_wave[LFO_sample]/4095.0);
    	index += 1;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[6], flang_gain, flang_table_6, ISC_2, flang_delay_6, index);
    	flang_wet_signal[6] = flang_wet_signal[6] * (LFO_sine_wave[LFO_sample]/4095.0);
    	index += 1;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[7], flang_gain, flang_table_7, ISC_2, flang_delay_7, index);
    	flang_wet_signal[7] = flang_wet_signal[7] * (LFO_sine_wave[LFO_sample]/4095.0);
    	index += 1;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[8], flang_gain, flang_table_8, ISC_2, flang_delay_8, index);
    	flang_wet_signal[8] = flang_wet_signal[8] * (LFO_sine_wave[LFO_sample]/4095.0);
    	index += 1;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[9], flang_gain, flang_table_9, ISC_2, flang_delay_9, index);
    	flang_wet_signal[9] = flang_wet_signal[9] * (LFO_sine_wave[LFO_sample]/4095.0);
    	index += 1;
    	flanger(&flang_wet_signal[0], &flang_wet_signal[10], flang_gain, flang_table_10, ISC_2, flang_delay_10, index);
    	flang_wet_signal[10] = flang_wet_signal[10] * (LFO_sine_wave[LFO_sample]/4095.0);


		if (delay_switch_counter >= 1000000)
    	{
    		delay_switch_counter = 0;
    	}
    
		delay_switch_counter++;
        
		if (delay_switch_counter >= 0 && delay_switch_counter < 100000)
    	{
    		flang_signal = (uint32_t)(flang_wet_signal[1] + (.5 * dry_signal));
    	}

    	else if (delay_switch_counter >= 100000 && delay_switch_counter < 200000)
    	{
    		flang_signal = (uint32_t)(flang_wet_signal[2] + (.5 * dry_signal));
   	 	}	

    	else if (delay_switch_counter >= 200000 && delay_switch_counter < 300000)
   	 	{
    		flang_signal = (uint32_t)(flang_wet_signal[3] + (.5 * dry_signal));
    	}

		else if (delay_switch_counter >= 300000 && delay_switch_counter < 400000)
    	{
    		flang_signal = (uint32_t)(flang_wet_signal[4] + (.5 * dry_signal));
    	}

		else if (delay_switch_counter >= 400000 && delay_switch_counter < 500000)
    	{
    		flang_signal = (uint32_t)(flang_wet_signal[5] + (.5 * dry_signal));
    	}

    	else if (delay_switch_counter >= 500000 && delay_switch_counter < 600000)
    	{
    		flang_signal = (uint32_t)(flang_wet_signal[6] + (.5 * dry_signal));
    	}

    	else if (delay_switch_counter >= 600000 && delay_switch_counter < 700000)
    	{
    		flang_signal = (uint32_t)(flang_wet_signal[7] + (.5 * dry_signal));
    	}

		else if (delay_switch_counter >= 700000 && delay_switch_counter < 800000)
    	{
    		flang_signal = (uint32_t)(flang_wet_signal[8] + (.5 * dry_signal));
    	}

    	else if (delay_switch_counter >= 800000 && delay_switch_counter < 900000)
    	{
			flang_signal = (uint32_t)(flang_wet_signal[9] + (.5 * dry_signal));
    	}

    	else if (delay_switch_counter >= 900000 && delay_switch_counter < 1000000)
    	{
    		flang_signal = (uint32_t)(flang_wet_signal[10] + (.5 * dry_signal));
    	}

        

 		delay_buffer[delay_write] = dry_signal;
		delay_write++;
    
		if(delay_write >= delay_depth)
		{
			delay_write = 0;
		}


    	else if (pog_value == 1)
    	{
    		delay_read_double[0] = delay_read_double[0] + 1;
        	pog_signal = delay_buffer[delay_read[0]];
    	} 
                
		else if (pog_value == 2)
    	{
    		delay_read_double[1] = delay_read_double[1] + 1.25; //goes to the fourth
        	pog_signal = delay_buffer[delay_read[1]];
    	}
                
		else if (pog_value == 3)
    	{
    		delay_read_double[2] = delay_read_double[2] + 1.3333; //goes to the fifth
        	pog_signal = delay_buffer[delay_read[2]];
    	}
                
		else if (pog_value == 4)
    	{
    		delay_read_double[3] = delay_read_double[3] + 1.5; // goes to the seventh
        	pog_signal = delay_buffer[delay_read[3]];
    	}
                
		else if (pog_value == 5)
    	{
    	delay_read_double[4] = delay_read_double[4] + 1.6666; // goes to the ninth
        pog_signal = delay_buffer[delay_read[4]];
    	}
                
		else if (pog_value == 6)
    	{
    		delay_read_double[5] = delay_read_double[5] + 2; // goes to octave
        	pog_signal = delay_buffer[delay_read[5]];
    	}
                
		if (pog_value == 7)
    	{
    		delay_read_double[0] = delay_read_double[0] + 1;
        	delay_read_double[1] = delay_read_double[1] + 1.25; // goes to octave
        	pog_signal = (delay_buffer[delay_read[0]] + delay_buffer[delay_read[1]]) >> 1;
    	}
                
		if (pog_value == 8)
    	{
    		delay_read_double[0] = delay_read_double[0] + 1;
        	delay_read_double[2] = delay_read_double[2] + 1.3333; // goes to octave
        	pog_signal = (delay_buffer[delay_read[0]] + delay_buffer[delay_read[2]]) >> 1;
    	}
                 
		if (pog_value == 9)
    	{
    		delay_read_double[0] = delay_read_double[0] + 1;
        	delay_read_double[3] = delay_read_double[3] + 1.5; // goes to octave
        	pog_signal = (delay_buffer[delay_read[0]] + delay_buffer[delay_read[3]]) >> 1;
    	}
                 
		if (pog_value == 10)
    	{
    		delay_read_double[0] = delay_read_double[0] + 1;
        	delay_read_double[4] = delay_read_double[4] + 1.6666; // goes to octave
        	pog_signal = (delay_buffer[delay_read[0]] + delay_buffer[delay_read[4]]) >> 1;
    	}
                  
		if (pog_value == 11)
    	{
    		delay_read_double[0] = delay_read_double[0] + 1;
        	delay_read_double[5] = delay_read_double[5] + 2; // goes to octave
        	pog_signal = (delay_buffer[delay_read[0]] + delay_buffer[delay_read[5]]) >> 1;
    	}
                  
		if (pog_value == 12)
    	{
    		delay_read_double[0] = delay_read_double[0] + 1;
        	delay_read_double[1] = delay_read_double[1] + 1.25;
        	delay_read_double[2] = delay_read_double[2] + 1.333;
        	delay_read_double[3] = delay_read_double[3] + 1.5;
        	delay_read_double[4] = delay_read_double[4] + 1.6666; // goes to octave
        	pog_signal = (delay_buffer[delay_read[3]] + delay_buffer[delay_read[4]]) >> 1;
        	pog_signal = (pog_signal + delay_buffer[delay_read[0]]) >> 1;
        	pog_signal = (pog_signal + delay_buffer[delay_read[2]]) >> 1;
    	}
                        
		for (int i = 0; i < 6; i++)
    	{
    		if(delay_read_double[i] >= delay_depth) 
        	{
        		delay_read_double[i] = 0;
        	}
      	
			delay_read[i] = (int) delay_read_double[i];
    	}	
        

    	rad_signal = dry_signal * sin(dry_signal*val) * (dry_signal * cos(dry_signal*val));
    	rad_signal += dry_signal;
    	rad_signal = (uint32_t) rad_signal;
    	rad_signal = (int) rad_signal;

    	loop_read++;                
           
    	if (loop_cue == 1)
    	{
    		if (loop_read >= MAX_LOOP)
        	{
        		loop_read = 0;
        	}
     	
			loop_buffer[loop_read] = dry_signal;
     		phase_loop_buffer[loop_read] = phased_signal;
     		flang_loop_buffer[loop_read] = flang_signal;
     		pog_loop_buffer[loop_read] = pog_signal;
     		rad_loop_buffer[loop_read] = rad_signal;
    	}
        
    	else if (loop_cue == 2)
        {
        	if (loop_read == final_loop_read)
            {
            	loop_read = 0;
            }                
        }

        while (bcm2835_gpio_lev(TOGGLE_SWITCH))
        {
        	PUSH1_val= bcm2835_gpio_lev(PUSH1);
            PUSH2_val= bcm2835_gpio_lev(PUSH2);
            if (PUSH2_val == 0)
            {
            	if (signal_effect < 4)
                {
                	signal_effect++;
                }
                
				else if (signal_effect >= 4)
                {
                	signal_effect = 0;
                }
                printf("\n\nsignal_effect = %d\n\n", signal_effect);
             }
                
			if (PUSH1_val == 0)
            {
            	if (loop_effect < 4)
                {
                	loop_effect++;
                }
                
				else if (loop_effect >= 4)
                {
                	loop_effect = 0;
                }
                printf("\n\nloop_effect = %d\n\n", loop_effect);
            }
                
			switch(signal_effect)
    		{
    			case 0: //one blink
    			bcm2835_delay(400);
    			bcm2835_gpio_write(LED,0);
    			bcm2835_delay(200);
    			bcm2835_gpio_write(LED,1);
				break;
 
    			case 1: //two blinks
        		bcm2835_delay(400);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1); 
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
    			break;
 
    			case 2:
        		bcm2835_delay(400);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);     
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
    			break;
 
    			case 3:
    			bcm2835_delay(400);
    			bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
    			break;
         
				case 4:
    			bcm2835_delay(400);
    			bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(200);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		break;
            }

          	switch(loop_effect)
    		{
    			case 0: //one blink
    			bcm2835_delay(300);
    			bcm2835_gpio_write(LED,0);
    			bcm2835_delay(150);
    			bcm2835_gpio_write(LED,1);
    			break;
 
    			case 1: //two blinks
        		bcm2835_delay(300);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1); 
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
    			break;
 
    			case 2:
        		bcm2835_delay(300);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);     
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
    			break;
 
    			case 3:
    			bcm2835_delay(300);
    			bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
       			bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
    			break;
         
				case 4:
    			bcm2835_delay(300);
    			bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,0);
        		bcm2835_delay(150);
        		bcm2835_gpio_write(LED,1);
        		bcm2835_delay(200);
        		break;
    		}
             
		}
        
		if (signal_effect == 0)
        {
        	no_loop_output = dry_signal;
        }

        else if (signal_effect == 1)
        {
        	no_loop_output = phased_signal;
        }

        else if (signal_effect == 2)
        {
        	no_loop_output = flang_signal;
        }

        else if (signal_effect == 3)
        {
        	no_loop_output = pog_signal;
        }

        else if (signal_effect == 4)
        {
        	no_loop_output = rad_signal;
        }

        if (loop_effect == 0)
        {
        	with_loop_output = (loop_buffer[loop_read] + no_loop_output) >> 1;
        }

        else if (loop_effect == 1)
        {
        	with_loop_output = (phase_loop_buffer[loop_read] + no_loop_output) >> 1;
        }

        else if (loop_effect == 2)
        {
        	with_loop_output = flang_loop_buffer[loop_read] + no_loop_output;
        }

        else if (loop_effect == 3)
        {
        	with_loop_output = pog_loop_buffer[loop_read] + no_loop_output;
        }

        else if (loop_effect == 4)
        {
        	with_loop_output = (rad_loop_buffer[loop_read] + no_loop_output) >> 1;
        }
        
        if (loop_cue == 0 || loop_cue == 1)
        {
        	bcm2835_pwm_set_data(1,no_loop_output & 0x3F);
            bcm2835_pwm_set_data(0,no_loop_output >> 6);
        }

        else
        {
        	bcm2835_pwm_set_data(1, with_loop_output & 0x3F);
             bcm2835_pwm_set_data(0, with_loop_output >> 6);
        }
    //generate output PWM signal 6 bits
 
 }
        //close all and exit
        bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
