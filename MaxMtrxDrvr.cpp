#include "Arduino.h"
#include "MaxMtrxDrvr.h"

void MaxMtrxDrvr::putBytes(int addr, byte addrByte, byte dataByte)
{ /*Clocks 'data' byte into MAX's register; MSB first*/
  /*byte i = 8;
  byte mask;
  while(i > 0) {
    mask = 0x01 << (i - 1);      // get bitmask
    digitalWrite(clk, LOW);   // tick
    if (data & mask) {           // choose bit
      digitalWrite(dataPin, HIGH);// send 1
    } else {
      digitalWrite(dataPin, LOW); // send 0
    }
    digitalWrite(clk, HIGH);   // tock
    --i;                         // move to lesser bit
  }*/
  digitalWrite(ss, LOW);

  for(int i=0; i<nMax*2; i++)                    //Tell all chips down the line - do nothing; addr*2 because 2 no-op bytes are needed to send 1 no-op command
    shiftOut(dataPin, clk, MSBFIRST, 0x00);

  shiftOut(dataPin, clk, MSBFIRST, addrByte);  //Send mutating bytes down the line
  shiftOut(dataPin, clk, MSBFIRST, dataByte);

  for(int i=0; i<addr*2; i++)                       //Push the mutating bytes to the correct chip
    shiftOut(dataPin, clk, MSBFIRST, 0x00);

  digitalWrite(ss, LOW);
  digitalWrite(ss, HIGH);
}

void MaxMtrxDrvr::errLed(int errType)
{ /*Light Status LED to indicate err*/
    switch(errType)
    {
        case 0:
            for(int i=0;i<3;i++)
            {
                digitalWrite(statusPin, HIGH);
                delay(100);
                digitalWrite(statusPin, LOW);
                delay(100);
            }
            break;
        case 1:
            digitalWrite(statusPin, HIGH);
            delay(500);
            digitalWrite(statusPin, LOW);
            delay(500);
            break;
        default:
            break;
    }
}

void MaxMtrxDrvr::setLed(int col, int row, int value, int addr)
{ /*col=x row=y; Range: col=[0,nCol-1]  row=[0,nRow-1]*/
  byte ledSegs[] = {128, 64, 32, 16, 8, 4, 2, 0};                       //SEG_DP,A,B,C,D,E,F,G
  byte ledCath[] = {0x01, 0x02, 0x03 , 0x04, 0x05, 0x06, 0x07, 0x08};   //DIG_0,1,2,3,4,5,6,7
  if(value == 1)
  {
      if( !(row<0 || row>nRow-1) && !(col<0 || col>nCol-1) ) //If (x,y) not out of array dimension
      {
          putBytes(addr, ledCath[row], ledSegs[col]);
      }
      else errLed(0);
  }
  else if(value == 0)
  {
      putBytes(addr, ledCath[row], 0x00);
  }
}

void MaxMtrxDrvr::clear(int addr)
{ /*Clear LED MX*/
   for(int i=0; i<nRow; i++) //For each row
   {
      putBytes(addr, i+1, 0x00);    //Set ctrl register
   }
}


//----------------------------------------------------------------------------------
//                              PUBLIC INTERFACE
//----------------------------------------------------------------------------------
MaxMtrxDrvr::MaxMtrxDrvr(int SS, int clock, int dataP, int numChips)
{ /*Constructor*/
    ss = SS;
    clk = clock;
    dataPin = dataP;
    nMax = numChips;
}

void MaxMtrxDrvr::setMXDim(int MxCol, int MxRow)
{
   nRow = MxRow;
   nCol = MxCol;
}

void MaxMtrxDrvr::setStatusPin(int ledPinNum)
{
   statusPin = ledPinNum;
}

void MaxMtrxDrvr::setMsg(char* m)
{
    msg = m;
}

char* MaxMtrxDrvr::getMsg()
{
   return msg;
}


void MaxMtrxDrvr::init()
{
   //Initialize Pins
   pinMode(ss,  OUTPUT);
   pinMode(clk, OUTPUT);
   pinMode(dataPin,OUTPUT);
   //Initialize Control Registers for each chip
   for(int i=0; i<nMax; i++)
   {
        putBytes(i, max7221_reg_shutdown, 0x01);    //0=Shutdown, 1=Normal Op  (ACK)
        putBytes(i, max7221_reg_displayTest, 0x00); //0=Off, 1=Display Test    (ACK)
        putBytes(i, max7221_reg_decodeMode, 0x00);  //0=Off, 1=7-Segment Mode  (Untested)
        putBytes(i, max7221_reg_scanLimit, 0x07);   //(????)                   (Untested)
        putBytes(i, max7221_reg_intensity, 0x08 & 0x0f);   //(????)            (Untested)
   }

   for(int i=0;i<nMax;i++)
      clear(i);   //Clear all MX

   errLed(0); //Wink 3 times.
}

void MaxMtrxDrvr::writeCol(int col, byte binCol, int chipNum)
{/*Translates and display binary-byte encoding of a column to 2-byte instructions for Max chip*/
  //byte colA[] = {8,16,32,64,128};
   for(int i=0; i<nRow; i++)
   {
      byte mask = 0x01 << i;
      if(mask & binCol)
      {
         putBytes(0, (i+1) & 0x0f, col); //Designate row
      }
   }
}

void MaxMtrxDrvr::writeRow(int row, byte binRow, int addr)
{/*Translates and display binary-byte encoding of a row in to a 2-byte instructions for Max chip*/
    putBytes(addr, row+1, binRow); //0-6;
}

void MaxMtrxDrvr::writeLet(char asciiLetter, int addr)
{
   int offset = 3;
   //Mapping function depends on how chTbl is set up; map ascii value to starting byte of corresponding letter
   byte startByteIndex = (asciiLetter - 65)*10 + offset;  //3 is an offset number; it's a 5x7 array so we skip first 2 starting bytes
   for(int i=0; i<8; i++) //Write to each row; writeRow(index of row, byte encoding)
   {
      writeRow(i, pgm_read_byte_near(chTbl + startByteIndex + i), addr);
      //pgm_read_byte_near() reads the address of starting location of array; chTbl = 0; Note pointer arithmetic
   }
}

void MaxMtrxDrvr::writeAlphabet()
{
    for(int i=0; i<26; i++)
    {
        writeLet(65+i,0);
        delay(500);
    }
}


