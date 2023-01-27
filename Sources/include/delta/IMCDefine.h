/*extern int  IMCAssertFailedLine(const char* szFile, int nLine);*/
/*extern void IMCTrace(const char* szFmt, ...);*/

#ifndef _IMC_DEFINE_H_
#define _IMC_DEFINE_H_

#define PGE_FMT_NO                0x0 // No pulse output
#define PGE_FMT_PD                0x1 // Pulse/Dir
#define PGE_FMT_CW                0x2 // CW/CCW
#define PGE_FMT_AB                0x3 // A/B Phase
                                  
#define ENC_FMT_NO                0x0 // No pulse input
#define ENC_FMT_PD                0x1 // Pulse/Dir
#define ENC_FMT_CW                0x2 // CW/CCW
#define ENC_FMT_AB                0x3 // A/B Phase
                                  
#define ENC_RATE_X0               0x0
#define ENC_RATE_X1               0x1
#define ENC_RATE_X2               0x2
#define ENC_RATE_X4               0x3
                                  
#define ENC_TRIG_FIRST            0x0
#define ENC_TRIG_LAST             0x1

#define     NO_TRIG_ENC           0x0000
#define INDEX0_TRIG_ENC           0x0001
#define INDEX1_TRIG_ENC           0x0002
#define INDEX2_TRIG_ENC           0x0004
#define INDEX3_TRIG_ENC           0x0008
#define INDEX4_TRIG_ENC           0x0010
#define INDEX5_TRIG_ENC           0x0020
#define INDEX6_TRIG_ENC           0x0040
#define INDEX7_TRIG_ENC           0x0080
                                  
#define OTP0_TRIG_ENC             0x0001
#define OTP1_TRIG_ENC             0x0002
#define OTP2_TRIG_ENC             0x0004
#define OTP3_TRIG_ENC             0x0008
#define OTP4_TRIG_ENC             0x0010
#define OTP5_TRIG_ENC             0x0020
#define OTP6_TRIG_ENC             0x0040
#define OTP7_TRIG_ENC             0x0080
#define OTN0_TRIG_ENC             0x0100
#define OTN1_TRIG_ENC             0x0200
#define OTN2_TRIG_ENC             0x0400
#define OTN3_TRIG_ENC             0x0800
#define OTN4_TRIG_ENC             0x1000
#define OTN5_TRIG_ENC             0x2000
#define OTN6_TRIG_ENC             0x4000
#define OTN7_TRIG_ENC             0x8000

#define ENC0_TRIG_DAC             0x00000001
#define ENC1_TRIG_DAC             0x00000002
#define ENC2_TRIG_DAC             0x00000004
#define ENC3_TRIG_DAC             0x00000008
#define ENC4_TRIG_DAC             0x00000010
#define ENC5_TRIG_DAC             0x00000020
#define ENC6_TRIG_DAC             0x00000040
#define ENC7_TRIG_DAC             0x00000080

#define ENC0_TRIG_LED             0x00000001
#define ENC1_TRIG_LED             0x00000002
#define ENC2_TRIG_LED             0x00000004
#define ENC3_TRIG_LED             0x00000008
#define ENC4_TRIG_LED             0x00000010
#define ENC5_TRIG_LED             0x00000020
#define ENC6_TRIG_LED             0x00000040
#define ENC7_TRIG_LED             0x00000080

#define DAC_SOURCE_SOFT           0x0
#define DAC_SOURCE_PCL            0x1

#define ADC_COMPARE_NO            0x0
#define ADC_COMPARE_L2H           0x1
#define ADC_COMPARE_H2L           0x2
#define ADC_COMPARE_ANY           0x3


#define RIO_SET0                  0x0
#define RIO_SET1                  0x1
#define RIO_SLAVE0                0x0
#define RIO_SLAVE1                0x1
#define RIO_SLAVE2                0x2
#define RIO_PORT0                 0x0
#define RIO_PORT1                 0x1
#define RIO_PORT2                 0x2
#define RIO_PORT3                 0x3

#define RIO_INT_NO                0x0
#define RIO_INT_RISE              0x1
#define RIO_INT_FALL              0x2
#define RIO_INT_LEVEL             0x3

#define RESET_TMR                 0x01
#define RESET_DDA                 0x02
#define RESET_ENC                 0x04
#define RESET_PCL                 0x08
#define RESET_DAC                 0x10
#define RESET_ADC                 0x20
#define RESET_LIO                 0x40
#define RESET_RIO                 0x80
#define RESET_ALL                 0xFF

#define IMC_ALL_INT_UNMASK        0x1FF
#define IMC_PGE_INT_UNMASK        0x001
#define IMC_ENC_INT_UNMASK        0x002
#define IMC_LIO_INT_UNMASK        0x004
#define IMC_TMR_INT_UNMASK        0x008
#define IMC_ADC_INT_UNMASK        0x030
#define IMC_PCL_INT_UNMASK        0x040
#define IMC_GSB_INT_UNMASK        0x080
#define IMC_RIO_INT_UNMASK        0x100

#define IMC_ALL_INT_MASK          0x000
#define IMC_PGE_INT_MASK          0xFFE
#define IMC_ENC_INT_MASK          0xFFD
#define IMC_LIO_INT_MASK          0xFFB
#define IMC_TMR_INT_MASK          0xFF7
#define IMC_ADC_INT_MASK          0xFCF
#define IMC_PCL_INT_MASK          0xFBF
#define IMC_GSB_INT_MASK          0xF7F
#define IMC_RIO_INT_MASK          0xEFF

#define LIO_INT_NO                0x0
#define LIO_INT_RISE              0x1
#define LIO_INT_FALL              0x2
#define LIO_INT_LEVEL             0x3

#define LIO_OTP0                   0x0
#define LIO_OTP1                   0x1
#define LIO_OTP2                   0x2
#define LIO_OTP3                   0x3
#define LIO_OTP4                   0x4
#define LIO_OTP5                   0x5
#define LIO_OTP6                   0x6
#define LIO_OTP7                   0x7
#define LIO_OTN0                   0x0
#define LIO_OTN1                   0x1
#define LIO_OTN2                   0x2
#define LIO_OTN3                   0x3
#define LIO_OTN4                   0x4
#define LIO_OTN5                   0x5
#define LIO_OTN6                   0x6
#define LIO_OTN7                   0x7
#define LIO_HOME0                  0x0
#define LIO_HOME1                  0x1
#define LIO_HOME2                  0x2
#define LIO_HOME3                  0x3
#define LIO_HOME4                  0x4
#define LIO_HOME5                  0x5
#define LIO_HOME6                  0x6
#define LIO_HOME7                  0x7
#define LIO_SVO0                   0x0
#define LIO_SVO1                   0x1
#define LIO_SVO2                   0x2
#define LIO_SVO3                   0x3
#define LIO_SVO4                   0x4
#define LIO_SVO5                   0x5
#define LIO_SVO6                   0x6
#define LIO_SVO7                   0x7

#define LIO_SOURCE_COMP0          0x01
#define LIO_SOURCE_COMP1          0x02
#define LIO_SOURCE_COMP2          0x04
#define LIO_SOURCE_COMP3          0x08
#define LIO_SOURCE_COMP4          0x10
#define LIO_SOURCE_COMP5          0x20
#define LIO_SOURCE_COMP6          0x40
#define LIO_SOURCE_COMP7          0x80

#endif


