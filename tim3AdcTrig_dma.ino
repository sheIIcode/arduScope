/*
 Timer Interrupts Example

 Demonstrates usage of the HardwareTimer classes by blinking the LED
 
 Created 22 April 2010, last updated 8 June 2010
 By Bryan Newbold for LeafLabs
 This code is released with no strings attached.
 
 */
#define LED_PIN PC13
#define BUFFER_SIZE 32
uint16_t adc_buffer[BUFFER_SIZE];

void handler_led(void);

int toggle = 0;


void setup_dma(void) {
    //connect dma1 to clk
    RCC_BASE->AHBENR |= RCC_AHBENR_DMA1EN; 
  
    // Configure DMA1 Channel1 for ADC
    DMA1CH1_BASE->CPAR = (uint32_t)&ADC1->regs->DR;  // Peripheral address
    DMA1CH1_BASE->CMAR = (uint32_t)adc_buffer; // Memory address
    DMA1CH1_BASE->CNDTR = BUFFER_SIZE;         // Number of data items to transfer
    DMA1CH1_BASE->CCR = DMA_CCR_MINC |        // Memory increment mode
                         DMA_CCR_PSIZE_16BITS |     // Peripheral size 16 bits
                         DMA_CCR_MSIZE_16BITS |     // Memory size 16 bits
                         DMA_CCR_CIRC |        // Circular mode
                         DMA_CCR_TCIE |        // Transfer complete interrupt enable
                         DMA_CCR_EN;           // Enable DMA

    // Enable DMA for ADC1
    ADC1->regs->CR2 |= ADC_CR2_DMA;
}

void setup()  {
	Serial.begin(1000000); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor
    // Set up the LED to blink 
    pinMode(LED_PIN, OUTPUT);

    // Set up BUT for input
//    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Setup LED Timer
//    Timer2.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
//    Timer2.setPeriod(LED_RATE); // in microseconds
//    Timer2.setCompare(TIMER_CH1, 1);      // overflow might be small
//    Timer3.attachInterrupt(TIMER_CH1, handler_led);

    TIMER3->regs.gen->PSC = 3600 - 1;   // Prescaler value (72 MHz / 720 = 100 kHz)
    TIMER3->regs.gen->ARR = 10 - 1;  // Auto-reload value (100 kHz / 1000 = 100 Hz)
    TIMER3->regs.gen->CR2 |= TIMER_CR2_MMS_UPDATE;

    // Configure ADC1
    ADC1->regs->CR2 |= ADC_CR2_EXTTRIG;  // Enable external trigger for injected group
//    ADC1->regs->CR2 |= ADC_CR2_EXTSEL_TIM3_TRGO;
    ADC1->regs->CR2 = 0x180001; // fix ADC_CR2_EXTSEL_TIM3_TRGO setup
//    ADC1->regs->CR2 |= ADC_CR2_DMA;       // Enable DMA mode
    ADC1->regs->SQR1 = 0;           // 1 conversion in the regular sequence
    ADC1->regs->SQR3 = 0;           // Channel 0
    ADC1->regs->CR2 |= ADC_CR2_EXTTRIG;
    ADC1->regs->CR2 |= ADC_CR2_ADON;      // Enable ADC
    
//    ADC1->regs->CR2 |= ADC_CR2_RSTCAL;    // Reset calibration
//    while (ADC1->regs->CR2 & ADC_CR2_RSTCAL);  // Wait for reset to complete
//    ADC1->regs->CR2 |= ADC_CR2_CAL;       // Start calibration
//    while (ADC1->regs->CR2 & ADC_CR2_CAL);  // Wait for calibration to complete


  setup_dma();
      
    delay(1000);
}

void loop() {

//  if (ADC1->regs->SR & ADC_SR_EOC) {
//    Serial.println(ADC1->regs->CR2);
    for(int i = 0; i< BUFFER_SIZE; i++){
      Serial.print("d:");
      Serial.println(adc_buffer[i]);
    }
    
    
    delay(40);

}

void handler_led(void) {
    toggle ^= 1;
    digitalWrite(LED_PIN, toggle);
    Serial.print("*");
    Serial.println(ADC1->regs->CR2);
} 
