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

uint8_t I2C_Slave_Begin(byte address);
uint8_t I2C_Slave_ReadByte();
void I2C_Slave_ReadData(byte *data,uint8_t count);

void setup() {
  delay(100);
  Serial.begin(9600);
}

void loop() {
  byte rev[10];
  int8_t begin;
  begin=I2C_Slave_Begin(0x55);
  if(begin==0)
  {
    I2C_Slave_ReadData(rev,4);
    Serial.println("Master write data- Slave read data");
    Serial.println(String((char*)rev));
  }
  else if(begin==1)
  {
    Serial.println("read");
  }
  else if(begin==2)
  {
    Serial.println("wrong");
  }
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
