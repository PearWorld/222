#ifndef __TEST_H__
#define __TEST_H__

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
  #define     __I     volatile                /*!< defines 'read only' permissions      */
#else
  #define     __I     volatile const          /*!< defines 'read only' permissions      */
#endif
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

typedef int int32_t;
typedef short int16_t;
typedef unsigned int const ucint32_t;    /* Read Only */
typedef unsigned short const ucint16_t;  /* Read Only */
typedef unsigned char const ucint8_t;    /* Read Only */
typedef volatile unsigned int vuint32_t;
typedef volatile unsigned short vuint16_t;
typedef volatile unsigned char vuint8_t;
typedef unsigned int size_t;
typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
#define TRUE  1
#define FALSE 0

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))// ������ GPIO7 ��0 |= 10000000 = 10000000 ��
/*
0 �� 7 λ�ֱ��Ӧ GPIO0 �� GPIO7
8 �� 15 λ�ֱ��Ӧ GPIO16 �� GPIO23
16 �� 23 λ�ֱ��Ӧ GPIO32 �� GPIO39
24 �� 31 λ�ֱ��Ӧ GPIO48 �� GPIO55
��������Ļ���0~31������ʵ��ʹ��ʱ���������Ҫ�Լ���0��8��16��24
�����ǵ�ƽģʽ�� GPIO16 ��0 |= 100000000 = 100000000 ��
*/

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

// EXTI relational
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
// EXTI relational

#ifdef __cplusplus
} 
#endif

#endif

