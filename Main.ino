#include <Wire.h>
#include <stdio.h>
#include <math.h>

#define BMP180_ADDR 0x77 // 7-bit address
#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6
#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4

int16_t AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
uint16_t AC4,AC5,AC6; 
double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;
char _error;
	
void setup()
{
	Serial.begin(9600); 
	Serial.println("REBOOT");
	if (begin())
		Serial.println("BMP180 init success");
	else
	{
		Serial.println("BMP180 init fail\n\n");
		while(1);
	}
}

void loop(){
	char status;
  double T,P,p0,a;


  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(ALTITUDE,0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE*3.28084,0);
  Serial.println(" feet");

  status = startTemperature();
  if (status != 0)
  {
    delay(status);

    status = getTemperature(T);
    if (status != 0)
    {
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      

      status = startPressure(3);
      if (status != 0)
      {
        delay(status);


        status = getPressure(P,T);
        if (status != 0)
        {
         
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");


          p0 = sealevel(P,ALTITUDE);
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");

          a = altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);
          Serial.print(" meters, ");
          Serial.print(a*3.28084,0);
          Serial.println(" feet");
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  delay(5000);
	
}

char begin(){
	double c3,c4,b1;
	Wire.begin();
	if(readDataFromBMP(0xAA,2))
		AC1= signed16BitVar;
	else
		return(0);
	if(readDataFromBMP(0xAC,2)
		AC2= signed16BitVar;
	else
		return(0);	
	if(readDataFromBMP(0xAE,2)
		AC3= signed16BitVar;
	else
		return(0);	
	if(readDataFromBMP(0xB0,2)
		AC4= unSigned16BitVar;
	else
		return(0);
	if(readDataFromBMP(0xB2,2)
		AC5= unSigned16BitVar;
	else
		return(0);
	if(readDataFromBMP(0xB4,2)
		AC6= unSigned16BitVar;
	else
		return(0);
	if(readDataFromBMP(0xB6,2)
		VB1= signed16BitVar;
	else
		return(0);
	if(readDataFromBMP(0xB8,2)
		VB2= signed16BitVar;
	else
		return(0);
	if(readDataFromBMP(0xBA,2)
		MB=	signed16BitVar;
	else
		return(0);
	if(readDataFromBMP(0xBC,2)
		MC= signed16BitVar;
	else
		return(0);
	if(readDataFromBMP(0xBE,2)
		MD= signed16BitVar;
	else
		return(0);
		
	return 1;

}

int16_t signed16BitVar;
uint16_t unSigned16BitVar;
 
char readDataFromBMP(unsigned char address, char length)
{
	char x;
	unsigned char values[2];
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(address);
	_error = Wire.endTransmission();
	if (_error == 0)
	{
		Wire.requestFrom(BMP180_ADDR,length);
		while(Wire.available() != length) ;
		for(x=0;x<length;x++)
		{
			values[x] = Wire.read();
		}
		
		signed16BitVar = (int16_t)((data[0]<<8)|data[1]);
			
		unSigned16BitVar = (((uint16_t)data[0]<<8)|(uint16_t)data[1]);
		
		return 1;
	}
	return(0);
}


char writeBytes(unsigned char *values, char length){
	char x;
	
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(values,length);
	_error = Wire.endTransmission();
	if (_error == 0)
		return(1);
	else
		return(0);
}


char startTemperature(void){
	unsigned char data[2], result;
	
	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	result = writeBytes(data, 2);
	if (result) 
		return(5); 
	else
		return(0); 
}


char getTemperature(double &T){
	unsigned char data[2];
	char result;
	double tu, a;
	
	data[0] = BMP180_REG_RESULT;

	result = readBytes(data, 2);
	if (result)
	{
		tu = (data[0] * 256.0) + data[1];

		
		a = c5 * (tu - c6);
		T = a + (mc / (a + md));

	}
	return(result);
}


char startPressure(char oversampling)
{
	unsigned char data[2], result, delay;
	
	data[0] = BMP180_REG_CONTROL;

	switch (oversampling)
	{
		case 0:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
		case 1:
			data[1] = BMP180_COMMAND_PRESSURE1;
			delay = 8;
		break;
		case 2:
			data[1] = BMP180_COMMAND_PRESSURE2;
			delay = 14;
		break;
		case 3:
			data[1] = BMP180_COMMAND_PRESSURE3;
			delay = 26;
		break;
		default:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
	}
	result = writeBytes(data, 2);
	if (result) 
		return(delay);
	else
		return(0); 
}


char getPressure(double &P, double &T){
	unsigned char data[3];
	char result;
	double pu,s,x,y,z;
	
	data[0] = BMP180_REG_RESULT;

	result = readBytes(data, 3);
	if (result)
	{
		pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);

		s = T - 25.0;
		x = (x2 * pow(s,2)) + (x1 * s) + x0;
		y = (y2 * pow(s,2)) + (y1 * s) + y0;
		z = (pu - x) / y;
		P = (p2 * pow(z,2)) + (p1 * z) + p0;


	}
	return(result);
}


double sealevel(double P, double A){
	return(P/pow(1-(A/44330.0),5.255));
}


double altitude(double P, double P0)
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}


char getError(void)
{
	return(_error);
}
