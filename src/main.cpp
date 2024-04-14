#include <Arduino.h>
#define SDA_OUTPUT (DDRD |= (1 << DDB4))
#define SCL_OUTPUT (DDRD |= (1 << DDB3))
#define SDA_INPUT  (DDRD &= ~(1 << DDB4))
#define SCL_INPUT  (DDRD &= ~(1 << DDB3))

#define SDA_HIGH   (PORTD |=  (1 << PD4))
#define SCL_HIGH   (PORTD |=  (1 << PD3))
#define SDA_LOW    (PORTD &= ~(1 << PD4))
#define SCL_LOW    (PORTD &= ~(1 << PD3))

#define SDA_READ   ((PIND & (1 << PIND4)) >> PIND4)
#define SCL_READ   ((PIND & (1 << PIND3)) >> PIND3)

#define HALF 25
#define FULL (HALF * 2)

void I2C_Slave_WriteByte(byte data);
int8_t I2C_Slave_WriteData(char *data);
uint8_t I2C_Slave_ReadACK();
void I2C_Slave_SendACK(); 
uint8_t I2C_Slave_Begin(byte address);
uint8_t I2C_Slave_ReadByte();
void I2C_Slave_ReadData(byte *data,uint8_t count);

void setup() {
  delay(100);
  Serial.begin(9600);
}

void loop() {
   byte rev[10];
   int8_t s,c;
   s=I2C_Slave_Begin(0x55);
   if(s==0)
   { 
      I2C_Slave_ReadData(rev,8);
      Serial.println("Master want to write data");
      Serial.println(String((char*)rev));
   }
   else if(s==1)
   {
    Serial.println("Master want to read data"); 
    c = I2C_Slave_WriteData("abcd");
    if(c==1)  Serial.println("Write successfully");
    else if(c==-1)  Serial.println("fail to write");

   }
   else if(s==2)
   {
    Serial.println("Wrong address");
   }
   delay(100);
}


uint8_t I2C_Slave_Begin(byte address)
{
  uint8_t data=0,rw=0;
  while(true)
  {
    SDA_INPUT;
    SCL_INPUT;
    while((SDA_READ != 1) || (SCL_READ !=1));
    while(SDA_READ == 1);
    delayMicroseconds(HALF/2);
    if(SCL_READ ==1)  // Star condition
    {
      while(SCL_READ==1);
      //read address + r/w
      for(int i=0;i<8;i++)
      {
        while(SCL_READ==0);
        data=(data<<1) | SDA_READ;
        while(SCL_READ==1);
      }
      rw = data & 1;
      data = data >> 1;
      if(data == address)
      {
        //send ack
        SDA_OUTPUT;
        SDA_LOW;
        while(SCL_READ == 0 );
        while(SCL_READ == 1 );
        SDA_INPUT;
        return rw;
      }
      else
      {
        return 2;
      }
    }
    else{}
  }
} 

void I2C_Slave_WriteByte(byte data)
{
  SDA_OUTPUT;
  for(int i=0;i<8;i++)
  {
    if( (data & 0x80) == 0x80)
    {
      SDA_HIGH;
    }
    else
    {
      SDA_LOW;
    }
    data = data << 1;
    while(SCL_READ==0);
    while(SCL_READ==1);
  }  
}

int8_t I2C_Slave_WriteData(char *data)
{
  uint8_t ack;
  while(*data != '\0')
  {
    I2C_Slave_WriteByte(*data);
    ack = I2C_Slave_ReadACK();
    if(ack==1)
    {
      SDA_INPUT;
      return -1;
    }
    data++;
  } 
  SDA_INPUT;
  return 1;
}

uint8_t I2C_Slave_ReadACK()
{
  uint8_t ack;
  SDA_INPUT;
  while(SCL_READ==0);
  ack=SDA_READ;
  while(SCL_READ==0);
  return ack;
}

void I2C_Slave_SendACK()
{
  SDA_OUTPUT;
  SDA_LOW;
  while(SCL_READ==0);
  while(SCL_READ==1);
  SDA_INPUT;
}

uint8_t I2C_Slave_ReadByte()
{
  uint8_t data;
  SDA_INPUT;
  SCL_INPUT;
  //read address + r/w
  for(int i=0;i<8;i++)
  {
    while(SCL_READ==0);
    data=(data<<1) | SDA_READ;
    while(SCL_READ==1);
  }
  SDA_OUTPUT;
  SDA_LOW;
  while(SCL_READ == 0 );
  while(SCL_READ == 1 );
  SDA_INPUT;
  return data;
}

void I2C_Slave_ReadData(byte *data,uint8_t count)
{
  for(int i=0;i<count;i++)
  {
    *data=I2C_Slave_ReadByte();
    data++;
  }
}