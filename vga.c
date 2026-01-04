#include <unistd.h>
#include <stdio.h>

// if using CPUlator, you should copy+paste contents of the file below instead of using #include
#ifndef __SYSTEM_INFO__
#define __SYSTEM_INFO__

#define DE10LITE 1 // change to 0 for CPUlator or DE1-SoC, 1 for DE10-Lite

/* do not change anything after this line */

#if DE10LITE
#define BOARD				"DE10-Lite"
#define MAX_X		160
#define MAX_Y		120
#define YSHIFT		  8
#else
#define MAX_X		320
#define MAX_Y		240
#define YSHIFT		  9
#endif


/* Memory */
#define SDRAM_BASE			0x00000000
#define SDRAM_END			0x03FFFFFF
#define FPGA_PIXEL_BUF_BASE		0x08000000
#define FPGA_PIXEL_BUF_END		0x0800FFFF
#define FPGA_CHAR_BASE			0x09000000
#define FPGA_CHAR_END			0x09001FFF

/* Devices */
#define LED_BASE			0xFF200000
#define LEDR_BASE			0xFF200000
#define HEX3_HEX0_BASE			0xFF200020
#define HEX5_HEX4_BASE			0xFF200030
#define SW_BASE				0xFF200040
#define KEY_BASE			0xFF200050
#define JP1_BASE			0xFF200060
#define ARDUINO_GPIO			0xFF200100
#define ARDUINO_RESET_N			0xFF200110
#define JTAG_UART_BASE			0xFF201000
#define TIMER_BASE			0xFF202000
#define TIMER_2_BASE			0xFF202020
#define MTIMER_BASE			0xFF202100
#define RGB_RESAMPLER_BASE    		0xFF203010
#define PIXEL_BUF_CTRL_BASE		0xFF203020
#define CHAR_BUF_CTRL_BASE		0xFF203030
#define ADC_BASE			0xFF204000
#define ACCELEROMETER_BASE		0xFF204020

/* Nios V memory-mapped registers */
#define MTIME_BASE             		0xFF202100

#endif

typedef uint16_t pixel_t;

volatile pixel_t* pVGA = (pixel_t*)FPGA_PIXEL_BUF_BASE;

volatile int* kptr = (int*)KEY_BASE;
volatile int* swptr = (int*)SW_BASE;
volatile int* ledptr = (int*)LEDR_BASE;


const pixel_t blk = 0x0000;
const pixel_t wht = 0xffff;
const pixel_t red = 0xf819;
const pixel_t grn = 0x07e0;
const pixel_t blu = 0x001f;

volatile uint32_t* mtime_ptr = (uint32_t*)MTIMER_BASE;
volatile uint64_t PERIOD = (uint64_t)100000000;
volatile int pending = 0;
const int half_y = MAX_Y / 2;
const int half_x = MAX_X / 2;
volatile int y1 = 1, x1 = 1;

volatile int player_score = 0, opp_score = 0;
volatile int y = MAX_Y / 2, x = MAX_X / 3, y2 = MAX_Y / 2, x2 = 2 * MAX_X / 3;
volatile int i = 0, i2 = 0, pcrash = 0, ocrash = 0;
volatile int dx[4] = { 1,0,-1,0 }, dy[4] = { 0,-1,0,1 };
volatile int dirchange = 0;
volatile int dirchange1 = 0;


//function prototypes
void set_mtimer(volatile uint32_t* time_ptr, uint64_t new_time64);
uint64_t get_mtimer(volatile uint32_t* time_ptr);
void setup_mtimecmp();
void keytimer_ISR(void);
pixel_t getcolor(int y, int x);
pixel_t makePixel(uint8_t r8, uint8_t g8, uint8_t b8);
void drawPixel(int y, int x, pixel_t colour);
void rect(int y1, int y2, int x1, int x2, pixel_t c);
void mtimer_ISR(void);
void setup_cpu_irqs(uint32_t new_mie_value);
void delay(int N);
void hex_display(int o, int p);
void gamestart();
void speed();
void setup_keyinterrupts();



void set_mtimer(volatile uint32_t* time_ptr, uint64_t new_time64) {
	*(time_ptr + 0) = (uint32_t)0;
	*(time_ptr + 1) = (uint32_t)(new_time64 >> 32);
	*(time_ptr + 0) = (uint32_t)new_time64;
}

void setup_keyinterrupts(){

	*(kptr+2)=0x3;
	*(kptr+3)=0xF;
	
}

uint64_t get_mtimer(volatile uint32_t* time_ptr) {
	uint32_t mtime_h, mtime_1;

	do {
		mtime_h = *(time_ptr + 1);
		mtime_1 = *(time_ptr + 0);
	} while (mtime_h != *(time_ptr + 1));

	return ((uint64_t)mtime_h << 32) | mtime_1;
}
void setup_mtimecmp() {

	uint64_t mtime64 = get_mtimer(mtime_ptr);
	mtime64 = (mtime64 / PERIOD + 1) * PERIOD;
	set_mtimer(mtime_ptr + 2, mtime64);
}

void keytimer_ISR(void) {

	int key=*(kptr+3);
	*(kptr+3)=key;
	if (key  & 0x1) {
		if(pending==1){
		   pending=0;	
		   *ledptr=0b00;
		}
		else{
		  pending=1;
		  *ledptr=0b01;
		}
		/*pending=1;
		*ledptr=0b01;
		return;*/
	}
     if (key & 0x2) {
		 
		if(pending==-1){
		   pending=0;	
		   *ledptr=0b00;
		}
		else{
		  pending=-1;
		  *ledptr=0b10;
		}
		 
       /*pending=-1;
		 *ledptr=0b10;
		 return;*/
	}
	
	return;

}
pixel_t getcolor(int y, int x) {
	return *(pVGA + (y << YSHIFT) + x);
}

pixel_t makePixel(uint8_t r8, uint8_t g8, uint8_t b8)
{
	// inputs: 8b of each: red, green, blue
	const uint16_t r5 = (r8 & 0xf8) >> 3; // keep 5b red
	const uint16_t g6 = (g8 & 0xfc) >> 2; // keep 6b green
	const uint16_t b5 = (b8 & 0xf8) >> 3; // keep 5b blue
	return (pixel_t)((r5 << 11) | (g6 << 5) | b5);
}

void drawPixel(int y, int x, pixel_t colour) {
	*(pVGA + (y << YSHIFT) + x) = colour;
}

void rect(int y1, int y2, int x1, int x2, pixel_t c)
{
	for (int y = y1; y < y2; y++)
		for (int x = x1; x < x2; x++)
			drawPixel(y, x, c);
}

volatile int prev1=0, prev2=0;
void mtimer_ISR(void) {
    
	speed();
	uint64_t mtimecmp64 = get_mtimer(mtime_ptr + 2);
	mtimecmp64 += PERIOD;
	set_mtimer(mtime_ptr + 2, mtimecmp64);

	drawPixel(y, x, red);
	y += dy[i];
	x += dx[i];
	
	if(!(*kptr&0x1)){
       prev1=0;
	}
	if(!(*kptr&0x2)){
       prev2=0;
	}

    if (!prev1&&pending == 1) {
		i = (i + 1) % 4;
		pending=0;
		prev1=1;
	}
	else if (!prev2&&pending == -1) {
		i = (i + 3) % 4;
		pending = 0;
		prev2=1;
	}
	
	if(pending==1){
		i=(i + 3) % 4;
	}
	if(pending==-1){
		i=(i + 1) % 4;
	}

	*ledptr = 0;
	if (getcolor(y, x) != blk) {
		pcrash = 1;
	}


	//entire robot player is handled here
	int left = (i2 + 1) % 4;
	int right = (i2 + 3) % 4;
	int straight = i2;
	int safe_left = (getcolor(y2 + dy[left], x2 + dx[left]) == blk);
	int safe_right = (getcolor(y2 + dy[right], x2 + dx[right]) == blk);
	int safe_straight = (getcolor(y2 + dy[straight], x2 + dx[straight]) == blk);

	if (safe_straight) {
		i2 = straight;
	}
	else if (safe_left) {
		i2 = left;
	}
	else if (safe_right) {
		i2 = right;
	}
	else {
		ocrash = 1;
	}

	drawPixel(y2, x2, grn);
	y2 += dy[i2];
	x2 += dx[i2];


}

void handler(void) __attribute__((interrupt("machine")));

void handler(void) {
	int mcause_value;

	__asm__ volatile("csrr %0, mcause" : "=r"(mcause_value));

	if (mcause_value == 0x80000007) {
		mtimer_ISR();
	}
	
	if (mcause_value == 0x80000012) keytimer_ISR();

}

uint32_t mstatus_value=0b1000, mtvec_value, old_mie_value;
void setup_cpu_irqs(uint32_t new_mie_value) {

	mtvec_value = (uint32_t)&handler;

	__asm__ volatile("csrc mstatus, %0" :: "r"(mstatus_value));

	__asm__ volatile("csrw mtvec, %0" :: "r"(mtvec_value));

	__asm__ volatile("csrr %0, mie"   :  "=r"(old_mie_value));
	__asm__ volatile("csrc mie, %0"   :: "r"(old_mie_value));
	__asm__ volatile("csrs mie, %0"   :: "r"(new_mie_value));

	__asm__ volatile("csrs mstatus, %0" :: "r"(mstatus_value));

}

void delay(int N)
{
	uint64_t start = get_mtimer(mtime_ptr);
	while (get_mtimer(mtime_ptr) < start + N) {
		*pVGA; // read volatile memory location to waste time
	}
}

void hex_display(int o, int p) {
	char segment[10] = { 0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,
					  0x7F,0x6F };
	int* hex = (int*)HEX3_HEX0_BASE;
	unsigned int val = 0;
	val += segment[p];
	val += (segment[o] << 16);
	*hex = val;
}
void gamestart() {
	uint32_t mstatus_value=0b1000;
	__asm__ volatile("csrc mstatus, %0" :: "r"(mstatus_value));
	rect(0, MAX_Y, 0, MAX_X, wht);
	rect(y1, MAX_Y - y1, x1, MAX_X - x1, blk);
	rect(y1, MAX_Y / 4, x1, MAX_X / 6, wht);
	rect(y1, MAX_Y / 4, 5 * MAX_X / 6, MAX_X - x1, wht);
	rect(3 * MAX_Y / 4, MAX_Y, x1, MAX_X / 6, wht);
	rect(3 * MAX_Y / 4, MAX_Y, 5 * MAX_X / 6, MAX_X - x1, wht);
	rect(3 * MAX_Y / 4, MAX_Y, 5 * MAX_X / 6, MAX_X - x1, wht);
	y = MAX_Y / 2;
	x = MAX_X / 3;
	y2 = MAX_Y / 2;
	x2 = 2 * MAX_X / 3;
	i = 0;
	i2 = 2;
	pcrash = 0;
	ocrash = 0;
	pending = 0;
	*ledptr = 0;
	PERIOD = (uint64_t)10000000;
	
	__asm__ volatile("csrs mstatus, %0" :: "r"(mstatus_value));
}

void speed() {
	
	double coeff[5]={2,3,5,7,9};
	PERIOD = (uint64_t)10000000;
	
	if ((*swptr) & 0x1) {
       PERIOD = (uint64_t)PERIOD*coeff[0];
	}
	else if ((*swptr) & 0x2) {
	   PERIOD = (uint64_t)PERIOD*coeff[1];
	}
	else if ((*swptr) & 0x4) {
	   PERIOD = (uint64_t)PERIOD*coeff[2];
	}
	else if ((*swptr) & 0x8) {
	   PERIOD = (uint64_t)PERIOD*coeff[3];
	}
	else if ((*swptr) & 0x16) {
	   PERIOD = (uint64_t)PERIOD*coeff[4];
	}
	else if ((*swptr) & 0x32) {
	  PERIOD = (uint64_t)PERIOD/coeff[0];
	}
	else if ((*swptr) & 0x64) {
	  PERIOD = (uint64_t)PERIOD/coeff[1];	
	}
	else if ((*swptr) & 0x128) {
	 PERIOD = (uint64_t)PERIOD/coeff[2];
	}
	else if ((*swptr) & 0x256) {
	 PERIOD = (uint64_t)PERIOD/coeff[3];
	}
	else if ((*swptr) & 0x512) {
	 PERIOD = (uint64_t)PERIOD/coeff[4];
	}
	
}

int main()
{
	setup_mtimecmp();
	setup_keyinterrupts();
	setup_cpu_irqs(0x50080);

	gamestart();
	__asm__ volatile("csrs mstatus, %0" :: "r"(mstatus_value));

	while (1) { 

		hex_display(opp_score, player_score);

		if (ocrash || pcrash) {
			if (!ocrash && pcrash) {
				opp_score++;
				gamestart();
				__asm__ volatile("csrs mstatus, %0" :: "r"(mstatus_value));
			}

			if (ocrash && !pcrash) {
				player_score++;
				gamestart();
				__asm__ volatile("csrs mstatus, %0" :: "r"(mstatus_value));
			}
			if (ocrash && pcrash) {
				gamestart();
				__asm__ volatile("csrs mstatus, %0" :: "r"(mstatus_value));
			}
			
			if(player_score == 9||opp_score == 9){

			if (player_score == 9) {
				rect(0, MAX_Y, 0, MAX_X, red);
				__asm__ volatile("csrc mstatus, %0" :: "r"(mstatus_value));
			}
			if (opp_score == 9) {
				rect(0, MAX_Y, 0, MAX_X, grn);
				__asm__ volatile("csrc mstatus, %0" :: "r"(mstatus_value));
			}	
				
			}
		}

	}


}


