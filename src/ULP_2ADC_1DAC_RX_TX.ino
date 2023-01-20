/* ESP32 ULP ADC DAC RX TX by RZ6AT 
Gets 12 bit 41000 samples per secund by ULP soprocessor ADC Audio_I and Audio_Q 
at RX mode and microphone and SWR meter at TX mode and ULP soprocessor sends 
to DAC1 and DAC2 12 bit 48400 per secund voltage levels to Speaker at RX mode 
or to mod at TX mode. Connect Audio_I to gpio34  and Audio_Q to gpio35
 Connect microphone to gpio39  and swr meter to gpio36
 Connect speaker and modulator to lowByte gpio25 via 160 kOhm + highByte to gpio26 via 10 kOhm  1:16
*/

#include "esp32/ulp.h"
#include "soc/rtc.h"
#include <driver/adc.h>
#include <driver/dac.h>
#include <driver/rtc_io.h>
#include "driver/rtc_cntl.h"

#define RX 33                       // RX LED PIN 1=light
#define TX 32                       // TX LED PIN 1=light
#define PTT 27                      // PEDAL for PTT 0=PTT
#define SAMPLES 256                 // 32/64/128/256/512
#define SAMPLING_FREQUENCY  40000   //

const gpio_num_t interruptPin = GPIO_NUM_5; // interrupt input by fast core gpio16
const gpio_num_t intr_out = GPIO_NUM_4;     // rtc_gpio10 ulp interrupt output by slow core gpio4
const gpio_num_t adcI_input = GPIO_NUM_34;  // ulp adcI input adc1_6 gpio34
const gpio_num_t adcQ_input = GPIO_NUM_35;  // ulp adcQ input adc1_7 gpio35
const gpio_num_t mic_input = GPIO_NUM_39;   // ulp mic input adc1_3 gpio39
const gpio_num_t swr_input = GPIO_NUM_36;   // ulp swr input adc1_0 gpio36
const gpio_num_t dac_L = GPIO_NUM_25;       // ulp dac_L output gpio25
const gpio_num_t dac_H = GPIO_NUM_26;       // ulp dac_H output gpio26

// RTC_SLOW_MEM: code=0-255 dac=256 adcI=512 adcQ=768 low 16bit in 32bit address in RTC_SLOW_MEM
const uint32_t ULP_CODE_OFFSET = 0;         // codes start address in RTC_SLOW_MEM lenght=256*4=1kbyte  
const uint32_t dac_slow = 256;              // DAC table start address in RTC_SLOW_MEM lenght=256*4=1kbyte
const uint16_t adcI_slow = 512;              // adcI table start address in RTC_SLOW_MEM lenght=(256+256)*4=2kbyte
const uint16_t adcQ_slow = 768;              // adcQ table start address in RTC_SLOW_MEM lenght=(256+256)*4=2kbyte
const uint32_t dacTable1 = 1024;            // dac1 table start 512 32bit wodrs lenght=(2*256)*4=2kbyte
const uint32_t dacTable2 = 1536;            // dac2 table start 512 32bit words lenght=(2*256)*4=2kbyte
const uint32_t retAddress1 = 14;            // ulp code number = return address from DAC table1
const uint32_t retAddress2 = 15;            // ulp code number = return address from DAC table2
const uint8_t clockDiv = 1;                 // adc clk up by 2
volatile bool tx_mode = 0;                  // 0=RX mode, 1=TX mode
volatile bool intr = false;                 // interrupts on rise and falling edges on gpio16 312 Hz
volatile bool half = false;                 // half of adc & dac tables (half=0 at 0--127)
int8_t adc_fastI[SAMPLES];                  // current adcI 256 samples table
int8_t adc_fastQ[SAMPLES];                  // current adcQ 256 samples table
int8_t dac_fast[SAMPLES];                   // current dac 256 samples table
volatile bool adcQ_to_dac = true;           // adcQ move to dac for test

// Create two commands to make dac tables /////////////////////////////////////
// based on https://github.com/bitluni/ULPSoundESP32/tree/master/Mono
uint32_t create_I_WR_REG(uint32_t reg, uint32_t low_bit, uint32_t high_bit, uint32_t val){
    typedef union {ulp_insn_t ulp_ins; uint32_t ulp_bin;} ulp_union;
    const ulp_insn_t singleinstruction[] = {I_WR_REG(reg, low_bit, high_bit, val)};
    ulp_union recover_ins;
    recover_ins.ulp_ins=singleinstruction[0];
    return (uint32_t)(recover_ins.ulp_bin);
}

uint32_t create_I_BXI(uint32_t imm_pc){
    typedef union {ulp_insn_t ulp_ins; uint32_t ulp_bin;} ulp_union;
    const ulp_insn_t singleinstruction[] = {I_BXI(imm_pc)};
    ulp_union recover_ins;
    recover_ins.ulp_ins=singleinstruction[0];
    return (uint32_t)(recover_ins.ulp_bin);
}
// Fix ESP32 ULP Core developer mistake = no command write_to_dac(byte N);
void make_dac_tables() {
  for(int i = 0; i < 256; i++)  //Create and fill values to DAC1 & DAC2 tables + returns
      {
        RTC_SLOW_MEM[dacTable1 + i * 2] = create_I_WR_REG(RTC_IO_PAD_DAC1_REG,19,26,i); //dac1: 0x1D4C0121 | (i << 10)
        RTC_SLOW_MEM[dacTable1 + 1 + i * 2] = create_I_BXI(retAddress1); // 0x80000000 + retAddress1 * 4
        RTC_SLOW_MEM[dacTable2 + i * 2] = create_I_WR_REG(RTC_IO_PAD_DAC2_REG,19,26,i); //dac2: 0x1D4C0122 | (i << 10)
        RTC_SLOW_MEM[dacTable2 + 1 + i * 2] = create_I_BXI(retAddress2); // 0x80000000 + retAddress2 * 4
      }
}
// ULP INIT ///////////////////////////////////////////////////////////////////
void ulp_init(){
  rtc_clk_fast_freq_set(RTC_FAST_FREQ_XTALD4);  // 40 mHz crystal divided by 4 = RTC_CLK
//  analogSetClockDiv(clockDiv);                  // Up to 2 faster ADC DAC samples  
  rtc_gpio_init(intr_out);                      // IRQ ULP Output Pin gpio4
  rtc_gpio_set_direction(intr_out, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_init(adcI_input);                    // ADC1 I gpio34
  rtc_gpio_set_direction(adcI_input, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_init(adcQ_input);                    // ADC1 Q gpio35
  rtc_gpio_set_direction(adcQ_input, RTC_GPIO_MODE_INPUT_ONLY);   
  rtc_gpio_init(dac_L);                         // dac low byte gpio25
  rtc_gpio_set_direction(dac_L, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_init(dac_H);                         // dac high byte gpio26
  rtc_gpio_set_direction(dac_H, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_init(mic_input);                     // ADC1 MIC gpio39
  rtc_gpio_set_direction(mic_input, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_init(swr_input);                     // ADC1 SWR input gpio36
  rtc_gpio_set_direction(swr_input, RTC_GPIO_MODE_INPUT_ONLY);

  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
  adc1_config_width(ADC_WIDTH_BIT_12);  
  adc1_ulp_enable();
  Serial.println("Start ADC");
}
// SET RX MODE ////////////////////////////////////////////////////////////////
void set_rx_mode(){
  tx_mode = 0;
  digitalWrite(RX, HIGH);
  digitalWrite(TX, LOW);
  RTC_SLOW_MEM[ULP_CODE_OFFSET + 2] = 0x5000001D; // ADC1 GPIO6 MUX=7 R1
//  Serial.println(RTC_SLOW_MEM[ULP_CODE_OFFSET + 2], HEX); 
  RTC_SLOW_MEM[ULP_CODE_OFFSET + 3] = 0x50000022; // ADC1 GPIO7 MUX=8 R2
//  Serial.println(RTC_SLOW_MEM[ULP_CODE_OFFSET + 3], HEX); 
}
// SET TX MODE ////////////////////////////////////////////////////////////////
void set_tx_mode() {
  tx_mode = 1;
  digitalWrite(RX, LOW);
  digitalWrite(TX, HIGH);
  RTC_SLOW_MEM[ULP_CODE_OFFSET + 2] = 0x50000005; // ADC1 GPIO0 MUX=1 R1
//  Serial.println(RTC_SLOW_MEM[ULP_CODE_OFFSET + 2], HEX); 
  RTC_SLOW_MEM[ULP_CODE_OFFSET + 3] = 0x50000012; // ADC1 GPIO3 MUX=4 R2
//  Serial.println(RTC_SLOW_MEM[ULP_CODE_OFFSET + 3], HEX); 
}
// BOUNCE PTT BUTTON ////////////////////////////////////////////////////////
volatile uint8_t cnt = 0;
volatile bool pres = 0;
void bounce(){
  if(!digitalRead(PTT)) { 
    if(!pres) { cnt += 1;
      if(cnt == 20) {             // 312/20=15,6 Hz = 64 milisecund bounce supress
        digitalWrite(RX, LOW);      
        digitalWrite(TX, HIGH);
        pres = 1; 
        set_tx_mode();   }  }  }
  else {
    if(pres) { cnt -= 1;
      if(cnt == 0) { 
        digitalWrite(RX, HIGH);            
        digitalWrite(TX, LOW);
        pres = 0;
        set_rx_mode();   }  }  }
}
// ISR /////////////////////////////////////////////////////////////////////
void IRAM_ATTR isr(){
    intr = true; 
}
// ADC ///////////////////////////////////////////////////////////////////
void IRAM_ATTR adc_(){
  int16_t sample = 0;
  for(uint8_t i = 0; i < 128; i++) {
      sample = RTC_SLOW_MEM[i + adcI_slow + (!half)*128];
      sample = 0x0fff & sample;
      adc_fastI[i + (!half)*128] = sample;          // move adcI & adcQ samples from RTC_SLOW_MEM to RAM

      sample = RTC_SLOW_MEM[i + adcQ_slow + (!half)*128];
      sample = 0x0fff & sample;
      adc_fastQ[i + (!half)*128] = sample;          // move adcI & adcQ samples from RTC_SLOW_MEM to RAM
         
      // Move adcQ samples to dac for test 
      if(adcQ_to_dac)  dac_fast[i + (!half)*128] = sample;    
      //      Serial.println(sample, HEX); 
  }
}
// DAC ///////////////////////////////////////////////////////////////////////
void IRAM_ATTR dac_(){
    int16_t sample = 0;
    for(uint8_t i = 0; i < 128; i++) {
    sample = dac_fast[i + (!half)*128];                  // move dac 128 samples from RAM
    sample = 0x0fff & sample;                            // 12 bit only
    RTC_SLOW_MEM[dac_slow + i + (!half)*128] = sample;   // to RTC_SLOW_MEM
  }      
}
// ADC & DAC ULP /////////////////////////////////////////////////////////////
  void adc_ulp() {
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2); 
    dac_output_voltage(DAC_CHANNEL_1, 128); 
    dac_output_voltage(DAC_CHANNEL_2, 128); 
  // ULP ADC & DAC -----------------------------------------------------------      
  const ulp_insn_t program[] = {  //№ clocks   
    I_MOVI(R0, 0),                //1 6  R0=0 dac samples counter 0-256
    M_LABEL(1),                   // label 1
  // ADC 
    I_ADC(R1, 0, ADC1_CHANNEL_6), //2 45 GPIO34 AUDIO_I ADC6  SLOW RAM FREE
    I_ADC(R2, 0, ADC1_CHANNEL_7), //3 45 GPIO35 AUDIO_Q ADC7  SLOW RAM FREE                         
    I_ST(R1, R0, adcI_slow),      //4 8  store to ADC_I samples table
    I_ST(R2, R0, adcQ_slow),      //5 8  store to ADC_Q samples table
  // DAC    
    I_LD(R1, R0, dac_slow),       //6 8  load from to_dac_tbl 12 BIT in 16 bit
    I_LSHI(R1, R1, 1),            //7 8 shift left 1 bit because dacTable is 512 
    I_MOVR(R2, R1),               //8 6  save DAC to R2
    I_ANDI(R1, R1, 0x01fe),       //9 6  get low byte only
    I_ADDI(R1, R1, dacTable1),    //10 6  set index DAC1 table 
    I_RSHI(R2, R2, 4),            //11 8  load DAC high byte shifted right 4 bit sample
    I_ANDI(R2, R2, 0x01e0),       //12 6  get byte only
    I_ADDI(R2, R2, dacTable2),    //13 6  set index DAC1 table
    I_BXR(R1),                    //14 4+12+4 jump to DAC1 table and return here
    I_BXR(R2),                    //15 4+12+4 jump to DAC2 table and return here               
    I_BL(11, 126),                //   4 8 jump 11 OPS forward if R0 little then  126
    I_BGE(4, 128),                //   4 8 jump 4 OPS forward if R0 grater or equal to 126
    I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + 10, 1), // 12 interrupt -> 1 
    M_LABEL(2),                   //   label 2    
    I_MOVI(R1, 0),                //   6 delay 6 clocks
    M_BX(3),                      //   8 jump to label 3
    I_BL(-2, 254),                //   4 8 jump to 2 ops back
    I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + 10, 1), // 12 interrupt -> 0                                
    I_MOVI(R0, 0),                //   6 256 samples
    M_LABEL(3),                   //   label 3
    I_ADDI(R0, R0, 1),            //   6 next sample       
    I_DELAY(1),                   //   156 HZ * 256 = 40000 samples per secund
    M_BX(1),                      //   Go to begin
    I_BGE(0, 300),                //   delay 4 clocks   
    M_BX(2)                       //   8 jump to label 2
  };

  size_t size = sizeof(program) / sizeof(ulp_insn_t);
  if (ulp_process_macros_and_load(ULP_CODE_OFFSET, program, &size) != ESP_OK) {
    Serial.println(F("Error loading ULP code!"));
    return;  }
    
  //this is how to get the opcodes
//  for(int i = 0; i < +size; i++) {
//    Serial.println(RTC_SLOW_MEM[ULP_CODE_OFFSET + i], HEX); // 32bit opcodes
//    Serial.println(i);  // number of code
//  } 
  if (ulp_run(ULP_CODE_OFFSET) != ESP_OK) {
    Serial.println(F("Error running ULP code!"));
    return;  }
}
// SETUP //////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  pinMode(interruptPin, INPUT);       // adc interrupt 40 kHz
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, CHANGE);
  pinMode(PTT, INPUT_PULLUP);
  pinMode(RX, OUTPUT);
  digitalWrite(RX, LOW);
  pinMode(TX, OUTPUT);
  digitalWrite(TX, LOW);
  make_dac_tables();
  set_rx_mode();
  ulp_init();
  adc_ulp();
}
// LOOP ///////////////////////////////////////////////////////////////////////
void loop() { 
  if(intr){
    intr = false;
    half = !half;
    bounce();
    dac_();
    adc_();
  }
}
