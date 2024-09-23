#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#define abs(x) (((x)>0) ? (x):(-(x)))
#ifndef PI
#define PI					3.14159265358979f
#endif

#define ANGLE_TO_RAD PI/180

#define TRUE 1
#define FALSE 0
#define LimitMax(input, max)   \
{                          		 \
  if (input > max)       			 \
  {                     		   \
    input = max;       			   \
  }                     		   \
  else if (input < -max) 		   \
  {                     			 \
    input = -max;      				 \
  }                      			 \
}

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef union
{
	uint8_t s[4];
	fp32 f;
}Un1;

//Î´²âÊÔ
typedef union
{
	uint8_t msg_u8[8];
	int8_t msg_8[8];
	uint16_t msg_u16[4];
	int16_t msg_16[4];
	fp32 msg_f32[2];
}Can_Commu_u;

#endif



