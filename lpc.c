#include "uart.h"
#include "spi.h"
#include "gpio.h"
#include "extintr.h"
#include "timer.h"

#define SBIT_WordLenght    0x00u
#define SBIT_DLAB          0x07u
#define SBIT_FIFO          0x00u
#define SBIT_RxFIFO        0x01u
#define SBIT_TxFIFO        0x02u

#define SBIT_RDR           0x00u
#define SBIT_THRE          0x05u

#define LED1      P2_0
#define LED2      P2_1

#define clkpin P2_12
#define datapin P0_3

void decodedata(void);
void clockISR(void);
unsigned char digitalRead(char dataPin);
//Milliseconds to wait until starting a new value
//This can be a different value depending on which flavor caliper you are using.
const int cycleTime = 32; 

unsigned volatile int clockFlag = 0; 

long now = 0;
long lastInterrupt = 0;
long value = 0;

float finalValue = 0;
float previousValue = 0;

int newValue = 0;
int sign = 1;
int currentBit = 1;


void myExtIntrIsr_0(void)
{
    GPIO_PinToggle(LED1); /* Toggle the LED1 (P2_0) */ 
}

void myExtIntrIsr_1(void)
{
    GPIO_PinToggle(LED2); /* Toggle the LED2 (P2_1) */ 
}





/* Function to initialize the UART0 at specifief baud rate */
void uart_init(uint32_t baudrate)
{
    uint32_t var_UartPclk_u32,var_Pclk_u32,var_RegValue_u32;

    LPC_PINCON->PINSEL0 &= ~0x000000F0;
    LPC_PINCON->PINSEL0 |= 0x00000050;            // Enable TxD0 P0.2 and p0.3 

    LPC_UART0->FCR = (1<<SBIT_FIFO) | (1<<SBIT_RxFIFO) | (1<<SBIT_TxFIFO); // Enable FIFO and reset Rx/Tx FIFO buffers    
    LPC_UART0->LCR = (0x03<<SBIT_WordLenght) | (1<<SBIT_DLAB);             // 8bit data, 1Stop bit, No parity


    /** Baud Rate Calculation :
       PCLKSELx registers contains the PCLK info for all the clock dependent peripherals.
       Bit6,Bit7 contains the Uart Clock(ie.UART_PCLK) information.
       The UART_PCLK and the actual Peripheral Clock(PCLK) is calculated as below.
       (Refer data sheet for more info)

       UART_PCLK    PCLK
         0x00       SystemFreq/4        
         0x01       SystemFreq
         0x02       SystemFreq/2
         0x03       SystemFreq/8   
     **/

    var_UartPclk_u32 = (LPC_SC->PCLKSEL0 >> 6) & 0x03;

    switch( var_UartPclk_u32 )
    {
    case 0x00:
        var_Pclk_u32 = SystemCoreClock/4;
        break;
    case 0x01:
        var_Pclk_u32 = SystemCoreClock;
        break; 
    case 0x02:
        var_Pclk_u32 = SystemCoreClock/2;
        break; 
    case 0x03:
        var_Pclk_u32 = SystemCoreClock/8;
        break;
    }


    var_RegValue_u32 = ( var_Pclk_u32 / (16 * baudrate )); 
    LPC_UART0->DLL =  var_RegValue_u32 & 0xFF;
    LPC_UART0->DLM = (var_RegValue_u32 >> 0x08) & 0xFF;

    util_BitClear(LPC_UART0->LCR,(SBIT_DLAB));  // Clear DLAB after setting DLL,DLM
}


/* Function to transmit a char */
void uart_TxChar(float ch)
{
    while(util_IsBitCleared(LPC_UART0->LSR,SBIT_THRE)); // Wait for Previous transmission
    LPC_UART0->THR=ch;                                  // Load the data to be transmitted
}


/* Function to Receive a char */
char uart_RxChar()
{
    char ch; 
    while(util_IsBitCleared(LPC_UART0->LSR,SBIT_RDR));  // Wait till the data is received
    ch = LPC_UART0->RBR;                                // Read received data    
    return ch;
}



int main()
{
    char ch,a[]="\n\rExploreEmbedded";
    int i;

    SystemInit();


		uart_init(9600);  // Initialize the UART0 for 9600 baud rate
			
    GPIO_PinDirection(LED1,OUTPUT);        /* Configure the pins as Output to blink the Leds*/
    GPIO_PinDirection(LED2,OUTPUT);
		GPIO_PinDirection(clkpin,INPUT);        /* Configure the pins as input to clock*/
    


    EINT_AttachInterrupt(EINT2,clockISR,RISING);   /* myExtIntrIsr_0 will be called by EINT2_IRQHandler */
		//  EINT_AttachInterrupt(EINT1,myExtIntrIsr_1,FALLING);   /* myExtIntrIsr_1 will be called by EINT1_IRQHandler */
	
	
	
		uart_TxChar('h'); //Transmit "hello" char by char
    uart_TxChar('e');
    uart_TxChar('l');
    uart_TxChar('l');
    uart_TxChar('o');


    for(i=0;a[i];i++)  //transmit a predefined string
        uart_TxChar(a[i]);


    while(1)
    {
			
		if(newValue) 
		{
			if(finalValue != previousValue) {
					previousValue = finalValue;
					uart_TxChar(finalValue);     
			 }
				newValue = 0;
		}
			
		 //The ISR Can't handle the arduino command millis()
		 //because it uses interrupts to count. The ISR will 
		 //set the clockFlag and the clockFlag will trigger 
		 //a call the decode routine outside of an ISR.
		 if(clockFlag == 1)
		 {
				clockFlag = 0;
				decodedata();
		 }
        //Finally receive a char and transmit it infinitely
        ch = uart_RxChar(); 
        uart_TxChar(ch);
    }       
}


void decodedata(void){
   unsigned char dataIn;
   dataIn = digitalRead(datapin); 
   TIMER_Start(0);                         /* Start the Timers */
   now = millis();
   
   if((now - lastInterrupt) > cycleTime)
   {
     finalValue = (value * sign) / 100.00;
     currentBit = 0;
     value = 0;
     sign = 1;
     newValue = 1;      
   } 
   else if (currentBit < 16 )
   {
      
     if (dataIn == 0)
     {
       if (currentBit < 16) {
          value |= 1 << currentBit;
       }  
       else if (currentBit == 20) {
          sign = -1;
       }
               
     }
     
     currentBit++;
     
   }
   
   lastInterrupt = now;
   
}

void clockISR(){
 clockFlag = 1; 
}

unsigned char digitalRead(char dataPin){
	GPIO_PinDirection(datapin,INPUT);
	if(!LPC_GPIO0->FIOPIN & (1<<3)){
			return 1;
	}
	else{
		return 0;
	}

}
