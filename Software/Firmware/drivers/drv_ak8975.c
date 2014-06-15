#include "ak8975.h"

vs16 AK8975_Data[3];
u8 data_temp[6];
static u8 iic_data_to_write;

u8 AK8975_Run(void)
{
	iic_data_to_write = 1;
	return ANO_TC_I2C1_Write_Int(0x18,0x0a,1,&iic_data_to_write);
}

u8 AK8975_check = 0;
u8 AK8975_Check1(void)
{
	return ANO_TC_I2C1_Read_Int(0x18,0x02,1,&AK8975_check);
}
u8 AK8975_Check2(void)
{
	if(AK8975_check==1)
		return 1;
	else
		return 0;
}
u8 AK8975_Read(void)
{
	return ANO_TC_I2C1_Read_Int(0x18,0x03,6,data_temp);
}
void AK8975_Cal(void)
{
	AK8975_Data[0]=data_temp[1]<<8 | data_temp[0];
	AK8975_Data[1]=data_temp[3]<<8 | data_temp[2];
	AK8975_Data[2]=data_temp[5]<<8 | data_temp[4];
}
void AK8975_GetValue(T_int16_xyz *mag)
{
	mag->X = AK8975_Data[0];
	mag->Y = AK8975_Data[1];
	mag->Z = AK8975_Data[2];
}
