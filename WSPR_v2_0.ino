/*
Arduino UNO DDS-60/AD9850 WSPR/QRSS Controller

 Generates WSPR coordinated frequency hopping transmissions 
 on 6 thru 160 meters synchronized by GPS time data.
 
 This sketch is configured for DDS60 AD9851 operation.
 
 Acknowlegements    
 The on-chip generation of the WSPR message algorithm is the work of 
 Andy Talbot, G4JNT. The EEPROM read/write routine is courtesy of of
 G0MGX. Portions of the GPS receive code were influenced by 
 Igor Gonzalez Martin's Arduino tutorial.
 
 Copyright (C) 2011,  Gene Marcus W3PM GM4YRE
 
 16 May 2013  Modified to compile with Arduino IDE 1.04 
              Corrected satellite display data bug
 
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.

 Modifications to the original sketch made by Rob PA0RWE december 2015  (v2_0)
 -  Start with DCF module but because DCF was not stable enough I switched to GPS
    Adafruit GPS library is used, including SoftwareSerial.
 -  Removed QRSS and WWVB stuff
 -  Improved DDS control software (Arduino_DDS60) by Peter Marks http://marxy.org is used.
 _________________________________________________________________________
 
 UNO Digital Pin Allocation
 D0
 D1
 D2  Cal enable / TX inhibit
 D3
 D4  load
 D5  clock
 D6  data
 D7  
 D8  PB2 Cal+
 D9  
 D10 PB3 Cal-
 D11 Serial Rx - GPS-Tx
 D12 Serial Tx - GPS-Rx
 D13 TX (LED)
 A0/D14 LCD D7
 A1/D15 LCD D6
 A2/D16 LCD D5
 A3/D17 LCD D4
 A4/D18 LCD enable
 A5/D19 LCD RS 
 ------------------------------------------------------------
 */
// include the library code:
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <MsTimer2.h>
#include <Adafruit_GPS.h>


struct CalFact {
  union {
    long value;
    struct {
      unsigned char b1;
      unsigned char b2;
      unsigned char b3;
      unsigned char b4;
    }
    __attribute__((packed));
  }
  __attribute__((packed));
}
__attribute__((packed));
struct CalFact CalFactor;


//__________________________________________________________________________________________________
// ENTER WSPR DATA:
char call[7] = "XXXXXX";    
char locator[5] = "XXXX"; // Use 4 character locator e.g. "EM64"
byte power = XX; // Min = 0 dBm, Max = 43 dBm, steps 0,3,7,10,13,17,20,23,27,30,33,37,40,43
//__________________________________________________________________________________________________

//__________________________________________________________________________________________________
// LOAD BAND FREQUENCY DATA:  Equal to WSPR bandhopping
unsigned long band[10] ={
  1838100,  // timeslot 0  00,20,40 minutes after hour
  3594100,  // timeslot 1  02,22,42 minutes after hour
//  5288700,  // timeslot 2  04,24.44 minutes after hour
  50294500, // timeslot 2  04,24.44 minutes after hour
  7040100,  // timeslot 3  06,26,46 minutes after hour
  10140200, // timeslot 4  08,28,48 minutes after hour
  14097100, // timeslot 5  10,30,50 minutes after hour
  18106100, // timeslot 6  12,32,52 minutes after hour
  21096100, // timeslot 7  14,34,54 minutes after hour
  24926100, // timeslot 8  16,36,56 minutes after hour
  28126100, // timeslot 9  18,38,58 minutes after hour
   
};

//__________________________________________________________________________________________________
// LOAD TRANSMIT TIME SLOT DATA: ( 0=idle, 1=transmit WSPR )

const byte TransmitFlag[10] ={
  1, // timeslot 0
  1, // timeslot 1
  1, // timeslot 2 
  1, // timeslot 3
  1, // timeslot 4
  1, // timeslot 5 
  1, // timeslot 6
  1, // timeslot 7
  1, // timeslot 8
  1  // timeslot 9
};

unsigned long fCLK=180000000;    // Enter clock frequency 

//__________________________________________________________________________________________________

const char SyncVec[162] = {
  1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,
  1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,
  0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,
  0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0
};


#define InhibitButton  2
#define CalSetButton   2
#define CalUpButton    8
#define CalDwnButton   10

// initialize the LCD library with the numbers of the interface pins
//                rs en d4 d5 d6 d7
LiquidCrystal lcd(19,18,17,16,15,14);
SoftwareSerial GPSSerial(11, 12);   // Rx, Tx (not used)
Adafruit_GPS GPS(&GPSSerial);

// configure variables
int GPSpin = 0;                     // RX PIN
int txPin  = 13;                    // TX (HIGH on transmit)
char sz[3];                         // Number of satellites
int seconds=0, minute=0, hour=0;    // Internal clock

int mSecTimer2;
unsigned int OffsetFreq[4];
byte data  = 6;                     //DATA 
byte clock = 5;                     //CLOCK 
byte load  = 4;                     //LOAD 
byte count = 0;

byte temp = 1;
unsigned long FreqWord, TempWord, TempFreq;
unsigned long TenMHz = 10000000;

char buf[10];
volatile byte bb, i, j, ii, timeslot;
byte symbol[162];
byte c[11];                         // encoded message
byte sym[170];                      // symbol table 162
byte symt[170];                     // symbol table temp
byte RXflag=1;                      // GPS/DCF receiver control 0 = disable, 1 = enable
byte RMCflag=0;                     // RMC actual data flag
int  MsgLength;
volatile byte InhibitFlag = 0;      // 1 will inhibit transmitter
unsigned long n1;                   // encoded callsign
unsigned long m1;                   // encodes locator


/******************************************************************
 * T I M E R 1  I R Q   S E C O N D  C O U N T E R
 *
 *  Clock - interrupt routine used as master timekeeper
 *  Timer1 Overflow Interrupt Vector, called every second
 ******************************************************************/
ISR(TIMER1_COMPA_vect) {
  seconds++ ;
  if (seconds == 60) {
    minute++ ;
    seconds=0 ;
  }
  if (minute == 60) {
    hour++;
    minute=0 ;
  }
  if (hour == 24) {
    hour=0 ;
  }

  displaytime();
  
  if(InhibitFlag == 0)
  {
    if(bitRead(minute,0) == 0 & seconds == 0) 
    {
      if (minute < 20) {
        timeslot = minute/2;
      }
      else{
        if (minute < 40) {
          timeslot = (minute-20)/2;
        }
        else {
          timeslot = (minute -40)/2;
        }
      }
      setfreq();
      transmit();
    }  
  }
  else{
  }
}


/******************************************************************
 * T I M E R 2   I R Q   m S E C O N D   C O U N T E R
 *
 *  Timer2 Overflow Interrupt Vector, called every mSec to increment
 *  the mSecTimer2 used for WSPR transmit timing.
 ******************************************************************/
ISR(TIMER2_COMPA_vect) {
  mSecTimer2++;
  if(mSecTimer2 > 681){
    mSecTimer2 = 0;
    if(bb < 3) {                    // Begin 2 second delay - actually 0.682mSec * 3
      TempWord = FreqWord;
      TransmitSymbol();
      bb++;
    }
    else
    {
    if (count < 162)                // Begin 162 WSPR symbol transmission
      {
        TempWord = FreqWord + OffsetFreq[sym[count]];
        TransmitSymbol();
        count++;                    // Increments the interrupt counter
      }
      else
      {
        TIMSK2 = 0;                 // Disable WSPR timer
        digitalWrite(txPin,LOW);    // External transmit control OFF 
        TempWord=0;                 // Turn off transmitter
        TransmitSymbol(); 
        RXflag = 1;                 // Turn GPS/DCF receiver back on 
        lcd.setCursor(9,1);      
        lcd.print("IDLE   ");
      }
    }
  }
}


/******************************************************************
 * S E T U P 
 ******************************************************************/
void setup()
{
  //Set up Timer2 to fire every mSec (WSPR timer)
  TIMSK2 = 0;        //Disable timer during setup
  TCCR2A = 2;        //CTC mode
  TCCR2B = 4;        //Timer Prescaler set to 64
  OCR2A = 247;       // Timer set for 1 mSec with correction factor

  //Set up Timer1A to fire every second (master clock)
  TCCR1B = 0;        //Disable timer during setup
  TIMSK1 = 2;        //Timer1 Interrupt enable
  TCCR1A = 0;        //Normal port operation, Wave Gen Mode normal
  TCCR1B = 12;       //Timer prescaler to 256 - CTC mode
  OCR1A = 62377;     //Timer set for 1000 mSec using correction factor
  // 62500 is nominal for 1000 mSec. Decrease variable to increase clock speed

  //Load offset values
  OffsetFreq[0] = 0;
  OffsetFreq[1] = 1.43 * pow(2,32) / fCLK;
  OffsetFreq[2] = 2.93 * pow(2,32) / fCLK;
  OffsetFreq[3] = 4.39 * pow(2,32) / fCLK;
  
  // Set up TX pin to output
  pinMode(txPin, OUTPUT);

  // Set Serial to 115200 baud
  Serial.begin(115200);
  
  // Set GPS Serial to 9600 baud
  GPS.begin(9600);

  // Set up transmit inhibit interrupt
  pinMode(InhibitButton, INPUT);
  digitalWrite(InhibitButton, HIGH);  // internal pull-up enabled

  // Set up calibration pins 
  pinMode(CalSetButton, INPUT);       // declare pushbutton as input 
  digitalWrite(CalSetButton, HIGH);   // internal pull-up enabled
  pinMode(CalUpButton, INPUT);        // declare pushbutton as input 
  digitalWrite(CalUpButton, HIGH);    // internal pull-up enabled
  pinMode(CalDwnButton, INPUT);       // declare pushbutton as input 
  digitalWrite(CalDwnButton, HIGH);   // internal pull-up enabled

  // Set up DDS-60
  pinMode (data, OUTPUT);   // sets pin as OUPUT
  pinMode (clock, OUTPUT);  // sets pin as OUTPUT
  pinMode (load, OUTPUT);   // sets pin as OUTPUT

  // set up the LCD for 16 columns and 2 rows 
  lcd.begin(16, 2); 

  // Turn on LCD 
  lcd.display();

  // turn off transmitter
  TempWord = 0;
  TransmitSymbol();

  // Display "IDLE" on LCD
  lcd.setCursor(9,1);
  lcd.print("IDLE   ");

  // Load Cal Factor
  CalFactor.b1 = EEPROM.read(50);
  CalFactor.b2 = EEPROM.read(51);
  CalFactor.b3 = EEPROM.read(52);
  CalFactor.b4 = EEPROM.read(53);

  setfreq(); 

  // Begin WSPR message calculation
  encode_call();
  encode_locator();
  encode_conv();
  interleave_sync();


  // Calibration process follows:
  if(digitalRead(CalSetButton) == LOW)
  {
    FreqWord = TenMHz*pow(2,32)/fCLK;
    TempWord = FreqWord;
    TransmitSymbol();
    detachInterrupt(0);     // Disable transmit inhibit interrupt
    TIMSK1 = 0;             // Disable timer1 interrupt (second clock)
    TIMSK2 = 0;             // Disable timer2 interrupt (WSPR)

    lcd.setCursor(0,0);
    lcd.print("Adjust to 10MHz");
    lcd.setCursor(0,1);
    lcd.print("Cal Factor= ");
    lcd.setCursor(12,1);
    lcd.print(CalFactor.value);
    calibrate();
  }
  attachInterrupt(0, TXinhibit, LOW);  // TX inhibit pin on interrupt 0 - pin 2
}

/******************************************************************
 *    L O O P 
 ******************************************************************/
void loop()
{
  if (RXflag == 1) {
    char c = GPS.read();
//    if (c) Serial.print(c);
    
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) return;
      seconds = GPS.seconds; minute = GPS.minute; hour = GPS.hour;
    }  
  }
}

/******************************************************************
 *   C O D E R I N G 
 ******************************************************************/
void encode() 
{
  encode_call();
  encode_locator();
  encode_conv();
  interleave_sync();
};

//******************************************************************
// normalize characters 0..9 A..Z Space in order 0..36
char chr_normf(char bc ) 
{
  char cc=36;
  if (bc >= '0' && bc <= '9') cc=bc-'0';
  if (bc >= 'A' && bc <= 'Z') cc=bc-'A'+10;
  if (bc >= 'a' && bc <= 'z') cc=bc-'a'+10;  
  if (bc == ' ' ) cc=36;

  return(cc);
}

//******************************************************************
void encode_call()
{
  unsigned long t1;

  // coding of callsign
  if (chr_normf(call[2]) > 9) 
  {
    call[5] = call[4];
    call[4] = call[3]; 
    call[3] = call[2];
    call[2] = call[1];
    call[1] = call[0];
    call[0] = ' ';
  }

  n1=chr_normf(call[0]);
  n1=n1*36+chr_normf(call[1]);
  n1=n1*10+chr_normf(call[2]);
  n1=n1*27+chr_normf(call[3])-10;
  n1=n1*27+chr_normf(call[4])-10;
  n1=n1*27+chr_normf(call[5])-10;

  // merge coded callsign into message array c[]
  t1=n1;
  c[0]= t1 >> 20;
  t1=n1;
  c[1]= t1 >> 12;
  t1=n1;
  c[2]= t1 >> 4;
  t1=n1;
  c[3]= t1 << 4;
}

//******************************************************************
void encode_locator()
{
  unsigned long t1;
  // coding of locator
  m1=179-10*(chr_normf(locator[0])-10)-chr_normf(locator[2]);
  m1=m1*180+10*(chr_normf(locator[1])-10)+chr_normf(locator[3]);
  m1=m1*128+power+64;

  // merge coded locator and power into message array c[]
  t1=m1;
  c[3]= c[3] + ( 0x0f & t1 >> 18);
  t1=m1;
  c[4]= t1 >> 10;
  t1=m1;
  c[5]= t1 >> 2;
  t1=m1;
  c[6]= t1 << 6;
}

//******************************************************************
// convolutional encoding of message array c[] into a 162 bit stream
void encode_conv()
{
  int bc=0;
  int cnt=0;
  int cc;
  unsigned long sh1=0;

  cc=c[0];

  for (int i=0; i < 81;i++) {
    if (i % 8 == 0 ) {
      cc=c[bc];
      bc++;
    }
    if (cc & 0x80) sh1=sh1 | 1;

    symt[cnt++]=parity(sh1 & 0xF2D05351);
    symt[cnt++]=parity(sh1 & 0xE4613C47);

    cc=cc << 1;
    sh1=sh1 << 1;
  }
}

//******************************************************************
byte parity(unsigned long li)
{
  byte po = 0;
  while(li != 0)
  {
    po++;
    li&= (li-1);
  }
  return (po & 1);
}

//******************************************************************
// interleave reorder the 162 data bits and and merge table with the sync vector
void interleave_sync()
{
  int ii,ij,b2,bis,ip;
  ip=0;

  for (ii=0;ii<=255;ii++) {
    bis=1;
    ij=0;
    for (b2=0;b2 < 8 ;b2++) {
      if (ii & bis) ij= ij | (0x80 >> b2);
      bis=bis << 1;
    }
    if (ij < 162 ) {
      sym[ij]= SyncVec[ij] +2*symt[ip];
      ip++;
    }
  }
}

/******************************************************************
 *  S E T F R E Q 
 *  Determine time slot and load band frequency data. Display 
 *  frequency and calculate frequency word for DDS
 ******************************************************************/
void setfreq()
{
  // Print frequency to the LCD
  lcd.setCursor(0,0);

  ltoa(band[timeslot],buf,10);

  if (buf[7]==0) {
    lcd.print(buf[0]);
    lcd.print(',');
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print('.');
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(" KHz  ");
  }
  else {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(',');
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print('.');
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(" KHz  ");
  }
  FreqWord = (band[timeslot]+(CalFactor.value*(band[timeslot]/pow(10,7))))*pow(2,32)/fCLK;
}


/******************************************************************
 *  T R A N S M I T
 *  Determine if it is time to transmit. If so, determine if it is
 *  time to transmit the WSPR message. If not turn to IDLE
 ******************************************************************/
void transmit()
{
  if(InhibitFlag ==1) return;
  else  
    if (TransmitFlag[timeslot] == 1)
    { // Start WSPR transmit process
      RXflag = 0;                 // Disable GPS/DCF receiver
      lcd.setCursor(9,1); 
      lcd.print("WSPR TX");
      digitalWrite(txPin,HIGH);   // External transmit control ON
      bb =0;
      count = 0;                  // Start WSPR symbol transmit process
      TIMSK2 = 2;                 // Enable timer2 interrupt 
    }
    else
    { // Turn off transmitter and idle
      TIMSK2 = 0;                 // Turn off WSPR timer
      lcd.setCursor(9,1);
      lcd.print("IDLE   ");
      digitalWrite(txPin,LOW);    // External transmit control OFF
      RXflag = 1;                 // Enable GPS/DCF receiver
      TempWord=0;
      TransmitSymbol();
    }
}  

/******************************************************************
 *   T U N E    D D S
 ******************************************************************/
void TransmitSymbol()
{
  digitalWrite (load, LOW);       // take load pin low

  for(int i = 0; i < 32; i++)
  {
    if ((TempWord & 1) == 1)
      outOne();
    else
      outZero();
    TempWord = TempWord >> 1;
  }
  byte_out(0x09);

  digitalWrite (load, HIGH);      // Take load pin high again
}

void byte_out(unsigned char byte)
{
  int i;

  for (i = 0; i < 8; i++)
  {
    if ((byte & 1) == 1)
      outOne();
    else
      outZero();
    byte = byte >> 1;
  }
}

void outOne()
{
  digitalWrite(clock, LOW);
  digitalWrite(data, HIGH);
  digitalWrite(clock, HIGH);
  digitalWrite(data, LOW);
}

void outZero()
{
  digitalWrite(clock, LOW);
  digitalWrite(data, LOW);
  digitalWrite(clock, HIGH);
}


/******************************************************************
 *  C A L I B R A T E  
 ******************************************************************/
void calibrate()
{ // Process to determine frequency calibration factor
  while(digitalRead(CalSetButton) == LOW)
  {
    if (digitalRead(CalUpButton) == LOW)
    {
      for(TempWord=0; TempWord < 350000; TempWord++) {
      }; //crude debounce delay
      CalFactor.value++;
    };
    
    if (digitalRead(CalDwnButton) == LOW)
    {
      for(TempWord=0; TempWord < 350000; TempWord++) {
      }; //crude debounce delay
      CalFactor.value--;
    };
    
    TempWord = (TenMHz+CalFactor.value)*pow(2,32)/fCLK;
    TransmitSymbol();
    lcd.setCursor(12,1);
    lcd.print(CalFactor.value);
    lcd.print("   ");
  }

  // Writes CalFactor to address 50 + 3 bytes of EEprom
  EEPROM.write(50,CalFactor.b1);
  EEPROM.write(51,CalFactor.b2);
  EEPROM.write(52,CalFactor.b3);
  EEPROM.write(53,CalFactor.b4);
  
  lcd.setCursor(0,1);
  lcd.print("         IDLE   ");
  TempWord = 0;                             // turn off 10 MHz calibrate signal
  TransmitSymbol();
  setfreq(); 
  attachInterrupt(0, TXinhibit, LOW);       // TX inhibit pin on interrupt 0 - pin 2 
  TIMSK1 = 2;                               // Enable Timer1 Interrupt 
}   


/******************************************************************
 *   D I S P L A Y   T I M E 
 *   
 *   Displays Time and number of active satellites during Tx IDLE
 ******************************************************************/
void displaytime()
{
  lcd.setCursor(0,1);
  lcd.display();
  if (hour < 10) lcd.print ("0");
  lcd.print (hour);
  lcd.print (":");
  if (minute < 10) lcd.print ("0");
  lcd.print (minute);
  lcd.print (":");
  if (seconds < 10) lcd.print ("0");
  lcd.print (seconds);
  lcd.print (" "); 

  if (RXflag == 0) return;
  
  if (GPS.satellites >= 0 && GPS.satellites < 20) {
    lcd.setCursor(9, 1);
    lcd.print (" SAT:");
    lcd.setCursor(14, 1);
    lcd.print (GPS.satellites);        
  }
}

/******************************************************************
 *   D I S P L A Y   F R E Q 
 ******************************************************************/
void DisplayFreq()
{
  // Print frequency to the LCD
  lcd.setCursor(0,0);

  ltoa(TempFreq,buf,10);

  if (buf[7]==0) {
    lcd.print(buf[0]);
    lcd.print(',');
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print('.');
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(" KHz ");
  }
  else {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(',');
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print('.');
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(" KHz ");
  }
}


/******************************************************************
 *   T X   I N H I B I T 
 ******************************************************************/
void TXinhibit()
{
  for(TempWord=0; TempWord < 350000; TempWord++) {
  }; //crude debounce delay
  InhibitFlag = !InhibitFlag;
  if(InhibitFlag == 1)            // 1 turns OFF transmitter
  {
    TIMSK2 = 0;                   // Turn off WSPR timer
    lcd.setCursor(15,0);
    lcd.print("*");
    digitalWrite(txPin,LOW);      // External transmit control OFF
    RXflag = 1;                   // Enable GPS/DCF receiver
    TempWord=0;                   // Set DDS60 to 0Hz
    TransmitSymbol();
  }
  else 
  {
    lcd.setCursor(15,0);
    lcd.print(" ");
  }
}

// End of file







































































































