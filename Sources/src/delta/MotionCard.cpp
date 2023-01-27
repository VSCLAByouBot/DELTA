#include "delta/MotionCard.h"

//==================== 軸卡ADC注意事項(20221017) ====================
/* 若沒有要接外部感測器(如:扭矩感測器,吸嘴夾爪等),請將ADC排線從軸卡上移除,並且於此程式註解掉所有ADC相關之功能
 * 若要使用ADC,請務必確認外部電壓值範圍符合軸卡讀取電壓值之限制範圍(-5 ~ +5V or 0 ~ 10V),再於此程式開啟所有ADC相關之功能
 */

//==================== 參數設定 ====================
const double K = K_56;							  // 第5,6關節耦合的Pitch補償倍數
const double GearRatio[AXIS] = {GR_0, GR_1, GR_2, // 減速比
								GR_3, GR_4, GR_5};
const double MaxTorque[AXIS] = {MAX_TQ_0, MAX_TQ_1, MAX_TQ_2,  // 最大馬達轉矩 = n * 額定轉矩[Nm],
								MAX_TQ_3, MAX_TQ_4, MAX_TQ_5}; // 最大馬達轉矩設定,驅動器參數P1-41

const double MaxVoltage = MAX_VOLTAGE; // 最大電壓[V]
const double CircleRad = 2.0 * M_PI;
double JointABSRad[AXIS] = {0.0};

int MotionCard_OpenCard(bool read_abs_enc, const int rd_abs_enc_max)
{
	//  --- 軸卡初始化 ---
	/*  Bool IMC_OpenDevice(		// true：Initial IMP 模組成功 false：Initial IMP 模組失敗
	int nMode,			// 0：初始化 IMP 的 PCI 模式  1：初始化 IMP 的 Standalone 模式
	WORD wCardIndex)		*/
	printf("MotionCard Opening ...\n");
	int nRet = IMC_OpenDevice(0, 0);

	if (nRet == 0)
	{
		puts("IMC_OpenDevice(): Failed\n");
		return IMC_OPENDEV_ERR;
	}

	//  --- 重置 IMP 模組 ---
	/*  IMC_GLB_ResetModule(
	WORD ModuleID,		// RESET_ALL: All Modules
	WORD wCardIndex)	// 欲控制的運動控制卡之編號，編號範圍 0~5 */
	IMC_GLB_ResetModule(RESET_ALL, 0);
	// --- 遮閉 IMP 模組中斷功能 ---
	/*  IMC_GLB_SetInterruptMode(
	int mode,			// IMC_ALL_INT_UNMASK: All Modules
	WORD wCardIndex) */
	IMC_GLB_SetInterruptMask(IMC_ALL_INT_UNMASK, 0);

	//----------------- 編碼器 -----------------
	for (int i = 0; i < AXIS; i++)
	{
		// --- A腳位開啟或關閉輸入訊號反相 ---
		/* IMC_ENC_EnableInAInverse(
		WORD enc_ch_no,			// encoder counter channel number（0~7）
		WORD wInverse,			// 1 開啟或0 關閉輸入訊號反相
		WORD wCardIndex)*/
		IMC_ENC_EnableInAInverse(i, 0, 0); // 0->encoder counter channel in A腳位輸入訊號不反相
		IMC_ENC_EnableInBInverse(i, 0, 0); // 0->encoder counter channel in B腳位輸入訊號不反相
		IMC_ENC_EnableInCInverse(i, 0, 0); // 0->encoder counter channel in C腳位輸入訊號不反相

		// --- 開啟或關閉輸入訊號 A/B 對調 ---
		/* IMC_ENC_EnableInABSwap(
		WORD enc_ch_no,			// encoder counter channel number（0~7）
		WORD wSwap,				// 1 開啟或 0 關閉輸入訊號 A/B 對調
		WORD wCardIndex) */
		IMC_ENC_EnableInABSwap(i, 0, 0); // 0->A/B輸入訊號不對調

		// --- 設定編碼器輸入格式為 A/B 型 ---
		/* IMC_ENC_SetInputFormat(
		WORD enc_ch_no,			// encoder counter channel number（0~7）
		WORD wFormat,			// Input type is quadratic or A/B phase
		WORD wCardIndex)*/
		IMC_ENC_SetInputFormat(i, ENC_FMT_AB, 0); // 設定編碼器輸入格式為 A/B 型

		// ---  設定編碼器輸入解碼倍率 x4 ---
		/*  IMC_ENC_SetInputRate(
		WORD enc_ch_no,			// encoder counter channel number（0~7）
		WORD rate,				// Multiplier rate to be 4
		WORD wCardIndex) */
		IMC_ENC_SetInputRate(i, ENC_RATE_X4, 0); // 設定編碼器輸入解碼倍率 x4

		// --- 清除 Encoder counter value ---
		/* IMC_ENC_ClearCounter(
		WORD enc_ch_no,			// encoder counter channel number（0~7）
		WORD wClear,			// 1 開啟或 0 關閉清除功能
		WORD wCardIndex)*/
		IMC_ENC_ClearCounter(i, 1, 0); // 開啟清除編碼器
		IMC_ENC_ClearCounter(i, 0, 0); // 關閉清除編碼器

		// --- 啟動 ENC Counter 記錄功能 ---
		/* IMC_ENC_StartCounter(
		WORD enc_ch_no,			// encoder counter channel number（0~7）
		WORD wStart,			// 1 開啟或 0 關閉編碼器功能
		WORD wCardIndex) */
		IMC_ENC_StartCounter(i, 1, 0); // 開始編碼器輸入
	}

	// 讀取絕對型編碼器數值,若數值錯誤就重新讀取(清命令時不會執行,其餘則自動執行)
	if (read_abs_enc)
	{
		int counter = 1;
		// 若讀取次數超過定值,程式將回傳錯誤代碼
		while (!MotionCard_ABS(JointABSRad))
		{
			printf("MotionCard_ABS(): Failed to read ABS Encoder, (%d/%d), try again...\n", counter, rd_abs_enc_max);
			if (counter >= rd_abs_enc_max)
			{
				printf("MotionCard_ABS(): Read ABS Encoder Failed, attempted %d times.\n", rd_abs_enc_max);
				return IMC_RD_ABS_ENC_ERR;
			}
			++counter;
		}
		printf("MotionCard_ABS(): Read ABS Encoder Successfully!\n");
	}

	//---------------- DAC & ADC ----------------
	for (int i = 0; i < AXIS; i++)
	{
		// --- 設定 DAC 命令源為軟體規劃 ---
		/* IMC_DAC_SelectSource(
		WORD channel,		// DAC channel number 0 ~ 7
		WORD source,		// DAC_CMD_SOFT：Source from DAC output buffer
		WORD wCardIndex)*/
		IMC_DAC_SelectSource(i, DAC_SOURCE_SOFT, 0); // 設定 DAC 命令源為軟體規劃

		// --- 開啟 DAC 功能 ---
		/* IMC_DAC_EnableChannel(
		WORD channel ,		// DAC channel number 0 ~ 7
		WORD wEnable,		// 1 開啟或 0 關閉指定 DA Channel 轉換功能
		WORD wCardIndex)*/
		IMC_DAC_EnableChannel(i, 1, 0); // 開啟 DAC 功能
		// IMC_ADC_EnableChannel(i, 1, 0); // 開啟 ADC 功能
	}

	// IMC_ADC_SetConverterMode(2, 0);

	IMC_DAC_StartConverter(1, 0); // 開始 DAC 轉換
	// IMC_ADC_StartConverter(1, 0); // 開始 ADC 轉換

	for (int i = 0; i < AXIS; i++)
	{
		// --- DAC 電壓清空 ---
		/* IMC_DAC_SetOutputVoltage(
		WORD channel,		// DAC channel number 0 ~ 7
		float fVoltage,		// 類比輸出電壓 (-10V ~ 10 V)
		WORD wCardIndex) */
		IMC_DAC_SetOutputVoltage(i, 0.0, 0);
	}

	//---------------- 中斷功能 ----------------
	// --- 設定計時器計時時間 1000 us (1豪秒) ---
	/* IMC_TMR_SetTimer(
	float dfPeriod,			// 計時器時間(µs)，可設定範圍 (0 ~ 2^32 毫秒)
	WORD wCardIndex) */
	IMC_TMR_SetTimer(1000, 0);

	// --- 開啟計時器計時功能 ---
	/* IMC_TMR_SetTimerEnable(
	WORD wEnable,			// 1 開啟或 0 關閉計時器功能
	WORD wCardIndex) */
	IMC_TMR_SetTimerEnable(1, 0);

	// --- 開啟計時器中斷功能 ---
	/* IMC_TMR_SetTimerIntEnable(
	WORD wEnable,			// 1 開啟或 0 關閉計時器中斷功能
	WORD wCardIndex) */
	IMC_TMR_SetTimerIntEnable(1, 0);
	puts("MotionCard Open OK!");
	return 0;
}

void MotionCard_CloseCard()
{
	printf("MotionCard Closing ... ");
	IMC_TMR_SetTimerIntEnable(0, 0); // 關閉計時器中斷功能
	IMC_TMR_SetTimerEnable(0, 0);	 // 關閉計時器計時功能

	for (int i = 0; i < AXIS; i++)
		IMC_DAC_SetOutputVoltage(i, 0.0, 0); // DAC電壓清空

	IMC_DAC_StartConverter(0, 0); // 停止 DAC 轉換
	// IMC_ADC_StartConverter(0, 0); // 停止 ADC 轉換

	for (int i = 0; i < AXIS; i++)
	{
		IMC_ENC_StartCounter(i, 0, 0);	// 停止編碼器輸入
		IMC_DAC_EnableChannel(i, 0, 0); // DAC中斷功能
		// IMC_ADC_EnableChannel(i, 0, 0); // ADC中斷功能
	}

	// --- 關閉 IMP 模組 ---
	/* IMC_CloseIfOpen(
	WORD wCardIndex) */
	IMC_CloseIfOpen(0);
	puts("OK !");
}

void MotionCard_ServoOn()
{
	puts("Servo turning ON");
	for (int i = 0; i < AXIS; i++)
	{
		// --- Servo On ---
		/* IMC_LIO_SetServoOn(
		WORD wChannel,
		WORD wCardIndex) */
		IMC_LIO_SetServoOn(i, 0); // ServoOn
	}
	// 這裡不要寫東西,應立即進入控制迴圈,避免手臂在無扭矩輸出下,受重力滑落
}

void MotionCard_ServoOff()
{
	// 這裡不要寫東西,緊急狀況發生時,應立即送出煞車命令
	for (int i = 0; i < AXIS; i++)
	{
		// --- Servo Off ---
		/* IMC_LIO_SetServoOff(
		WORD wChannel,
		WORD wCardIndex) */
		IMC_LIO_SetServoOff(i, 0); // ServoOff
	}
	puts("Servo turned OFF OK!");
}

bool MotionCard_ABS(double *JointABSRad) // 讀取絕對型編碼器數值[Rad]
{
	enum DataStatus
	{
		Encoder_Status = 1,
		Zero,
		Encoder_PUU,
		Check_Sum,
		Other
	};
	DataStatus Data_Status;						 // 讀取的資料種類 ( 編碼器狀態 , 0 , PUU , 檢查碼 )
	DWORD input = 0, ABSR_check, Data_reg[AXIS]; // 讀取資料 , ABSR確認 , 讀取資料暫存( 一個bit暫存 共六軸 )

	int PUU_reg[AXIS][32] = {0}, Encoder_check[AXIS][32], PUU_Sign_check = 0;		  // PUU暫存( 32bit 共六軸 ) , 讀取確認碼( 全部0表示無誤 )
	int Bit_cnt = 0, M = 10, N = 128;												  // 讀取資料bit表示( 共80個bit ) , 電子齒輪比;
	double PUU[AXIS] = {0.0}, MotorABSPulse[AXIS] = {0.0}, MotorABSRad[AXIS] = {0.0}; // 絕對型編碼器馬達端 PUU & Pluse & Rad

	IMC_ARIO_EnableSlaveControl(0x00, 1); // 開啓ARIO
	Sleep(10);

	// ABSC -> DI2(OUT0), ABSE -> DI3(OUT1), ABSQ -> DI4(OUT2)

	IMC_ARIO_SetOutputValue(0x00, 0x0000); // Set ABSE為low  ABSQ為low (0000 0000)
	Sleep(10);							   // 用ARIO指令要延遲 因為傳輸要時間

	IMC_ARIO_SetOutputValue(0x00, 0x0006); // Set ABSE為high  ABSQ為high ( 用ARIO OutBit 1&2 ) (0000 0110)
	Sleep(10);

	while (Bit_cnt <= 79)
	{
		IMC_ARIO_SetOutputValue(0x00, 0x0002); // Set ABSE為high  ABSQ為low
		Sleep(10);

		IMC_ARIO_GetInputValue(0x00, &input); // Get ABSR
		Sleep(10);

		ABSR_check = input & (0x003F); // 結取 InputBit 用來判斷是否都有收到ABSR

		switch (Bit_cnt) // 切換讀取的資料種類
		{
		case 0:
			Data_Status = Encoder_Status; // 0~15bit為Encoder_Status
			break;
		case 16:
			Data_Status = Zero; // 16~31bit為Zero
			break;
		case 32:
			Data_Status = Encoder_PUU; // 32~63bit為Encoder_PUU
			break;
		case 64:
			Data_Status = Check_Sum; // 63~79bit為Check_Sum
			break;
		default:
			break;
		}

		if (ABSR_check == (0x0000)) // 判斷ARIO InputBit 是否都有收到ABSR
		{
			// printf("get\n");
			// 擷取 InputBit 即各軸收到的資料值
			Data_reg[0] = input & (0x0100);
			Data_reg[1] = input & (0x0200);
			Data_reg[2] = input & (0x0400);
			Data_reg[3] = input & (0x0800);
			Data_reg[4] = input & (0x1000);
			Data_reg[5] = input & (0x2000);

			switch (Data_Status) // 資料存取轉換
			{
			case Encoder_Status: // 收到0要存1 , 收到1要存0
				if (Data_reg[0] == (0x0100))
					Encoder_check[0][Bit_cnt] = 0;
				if (Data_reg[1] == (0x0200))
					Encoder_check[1][Bit_cnt] = 0;
				if (Data_reg[2] == (0x0400))
					Encoder_check[2][Bit_cnt] = 0;
				if (Data_reg[3] == (0x0800))
					Encoder_check[3][Bit_cnt] = 0;
				if (Data_reg[4] == (0x1000))
					Encoder_check[4][Bit_cnt] = 0;
				if (Data_reg[5] == (0x2000))
					Encoder_check[5][Bit_cnt] = 0;
				break;
			case Zero: // 收到0要存1 , 收到1要存0
				if (Data_reg[0] == (0x0100))
					Encoder_check[0][Bit_cnt] = 0;
				if (Data_reg[1] == (0x0200))
					Encoder_check[1][Bit_cnt] = 0;
				if (Data_reg[2] == (0x0400))
					Encoder_check[2][Bit_cnt] = 0;
				if (Data_reg[3] == (0x0800))
					Encoder_check[3][Bit_cnt] = 0;
				if (Data_reg[4] == (0x1000))
					Encoder_check[4][Bit_cnt] = 0;
				if (Data_reg[5] == (0x2000))
					Encoder_check[5][Bit_cnt] = 0;
				break;
			case Encoder_PUU: // 收到0要存1 , 收到1要存0
				if (Data_reg[0] != (0x0100))
					PUU_reg[0][Bit_cnt - 32] = 1;
				if (Data_reg[1] != (0x0200))
					PUU_reg[1][Bit_cnt - 32] = 1;
				if (Data_reg[2] != (0x0400))
					PUU_reg[2][Bit_cnt - 32] = 1;
				if (Data_reg[3] != (0x0800))
					PUU_reg[3][Bit_cnt - 32] = 1;
				if (Data_reg[4] != (0x1000))
					PUU_reg[4][Bit_cnt - 32] = 1;
				if (Data_reg[5] != (0x2000))
					PUU_reg[5][Bit_cnt - 32] = 1;
				break;
			case Check_Sum:
				break;
			case Other:
				break;
			default:
				break;
			}
			IMC_ARIO_SetOutputValue(0x00, 0x0006); // Set ABSE為high  ABSQ為high
			Sleep(10);
			Bit_cnt++; // 跳下一個bit讀取
		}
	}

	for (int i = 0; i < AXIS; i++) // PUU&Pluse_Calculate
	{
		if (PUU_reg[i][31] == 1) // 判斷PUU值正負
		{
			PUU_Sign_check = 1;
			for (int j = 0; j <= 31; j++) // 如果PUU為負做2補數
			{
				if (PUU_reg[i][j] == 1)
				{
					PUU_reg[i][j] = 0;
				}
				else
				{
					PUU_reg[i][j] = 1;
				}
			}
		}

		for (int j = 0; j < 32; j++) // PUU二進位轉十進位
		{
			PUU[i] = PUU[i] + PUU_reg[i][j] * pow(2, j);
		}

		if (PUU_Sign_check == 1) // PUU為負十進位轉負
		{
			PUU[i] = -PUU[i] - 1;
		}

		PUU_Sign_check = 0;

		MotorABSPulse[i] = PUU[i] * N / M;							 // 經電子齒輪比將馬達端的絕對型編碼器PUU轉成Pulse[Pulse]
		MotorABSRad[i] = (MotorABSPulse[i] / 1280000.0) * CircleRad; // "馬達端"絕對型編碼器數值[Rad] , 絕對型編碼器固定一圈1280000個Pulse

		if (i <= 4)
		{
			JointABSRad[i] = MotorABSRad[i] / GearRatio[i]; // 第1~5關節"關節端"的絕對型編碼器數值[Rad]
		}
		else
		{
			JointABSRad[5] = (MotorABSRad[5] / GearRatio[5]) + K * (MotorABSRad[4] / GearRatio[4]); // 第6關節"關節端"的絕對型編碼器數值[Rad] , 含與第5關節耦合項
		}
	}

	IMC_ARIO_SetOutputValue(0x00, 0x0000); // 關致能
	Sleep(10);

	IMC_ARIO_EnableSlaveControl(0x00, 0); // 關ARIO
	Sleep(10);

	for (int i = 0; i < 6; i++) // 讀取確認碼若沒全為零回傳讀取錯誤
	{
		for (int j = 0; j < 32; j++)
		{
			if (Encoder_check[i][j] != 0)
				return false;
		}
	}
	return true;
}

void MotionCard_Encoder(double (&JointEncRad)[AXIS])
{
	double Direction[AXIS] = {-1.0, 1.0, 1.0, -1.0, 1.0, 1.0};
	long MotorINCRPulse[AXIS] = {0};
	double MotorINCRRad[AXIS] = {0.0};

	for (int i = 0; i < AXIS; i++)
	{
		// --- "馬達端"增量型編碼器數值[Pulse] ---
		/* IMC_ENC_ReadCounter(
		WORD enc_ch_no,			// encoder counter channel number (0~7)
		long *lCounter,			// 讀取Encoder counter 數值
		WORD wCardIndex) */
		IMC_ENC_ReadCounter(i, &MotorINCRPulse[i], 0);
		MotorINCRRad[i] = ((double)MotorINCRPulse[i] / 260000.0) * CircleRad; // "馬達端"增量型編碼器數值[Rad] , 增量型編碼器設定一圈65000*4個Pulse , 驅動器參數P1-46*軸卡4倍頻

		if (i <= 4)
		{
			JointEncRad[i] = Direction[i] * (MotorINCRRad[i] / GearRatio[i]) + JointABSRad[i]; // 第1~5關節"關節端"的增量+絕對型編碼器數值[Rad]
		}
		else
		{
			JointEncRad[5] = Direction[5] * (MotorINCRRad[5] / GearRatio[5]) + K * (Direction[4] * (MotorINCRRad[4] / GearRatio[4])) + JointABSRad[5]; // 第6關節"關節端"的增量+絕對型編碼器數值[Rad] , 含與第5關節耦合項
		}
	}
}

void MotionCard_DAC(double (&JointTorCtrl)[AXIS])
{
	double MotorTorCtrl[AXIS] = {0.0};

	for (int i = 0; i < AXIS; i++)
	{
		if (i != 4)
		{
			MotorTorCtrl[i] = JointTorCtrl[i] / GearRatio[i]; // 除第5關節之外"馬達端"的轉矩控制命令[Nm]
		}
		else
		{
			MotorTorCtrl[4] = (JointTorCtrl[4] + K * JointTorCtrl[5]) / GearRatio[4]; // 第5關節"馬達端"的轉矩控制命令[Nm] , 含與第6關節耦合項
		}

		if (MotorTorCtrl[i] > MaxTorque[i])
		{
			MotorTorCtrl[i] = MaxTorque[i]; // 正方向馬達轉矩限制
		}

		if (MotorTorCtrl[i] < -MaxTorque[i])
		{
			MotorTorCtrl[i] = -MaxTorque[i]; // 負方向馬達轉矩限制
		}

		double ControlSignal = (MotorTorCtrl[i] / MaxTorque[i]) * MaxVoltage; // 馬達轉矩控制命令對應的電壓控制訊號[V]

		IMC_DAC_SetOutputVoltage(i, (float)ControlSignal, 0);
	}
}

void MotionCard_DAC(double (&JointTorCtrl)[AXIS], double(&Voltage))
{
	double MotorTorCtrl[AXIS] = {0.0};

	for (int i = 0; i < AXIS; i++)
	{
		if (i != 4)
		{
			MotorTorCtrl[i] = JointTorCtrl[i] / GearRatio[i]; // 除第5關節之外"馬達端"的轉矩控制命令[Nm]
		}
		else
		{
			MotorTorCtrl[4] = (JointTorCtrl[4] + K * JointTorCtrl[5]) / GearRatio[4]; // 第5關節"馬達端"的轉矩控制命令[Nm] , 含與第6關節耦合項
		}
		if (MotorTorCtrl[i] > MaxTorque[i])
		{
			MotorTorCtrl[i] = MaxTorque[i]; // 正方向馬達轉矩限制
		}

		if (MotorTorCtrl[i] < -MaxTorque[i])
		{
			MotorTorCtrl[i] = -MaxTorque[i]; // 負方向馬達轉矩限制
		}

		Voltage = (MotorTorCtrl[i] / MaxTorque[i]) * MaxVoltage; // 馬達轉矩控制命令對應的電壓控制訊號[V]

		IMC_DAC_SetOutputVoltage(i, (float)Voltage, 0);
	}
}

// ================ ADC設定 ===================
// void MotionCard_ADC(double (&ADCVol)[AXIS])
// {
// 	float ADCVoltage[AXIS];
//
// 	for (int i = 0; i < AXIS; i++)
// 	{
// 		IMC_ADC_GetInputVoltage(i, &ADCVoltage[i], 0);						  	// 讀取ADC電壓
// 		ADCVoltage[i] = ADCVoltage[i] + (0.03482F * ADCVoltage[i] - 0.016577F); // ADC偏置補償 (Offset)
// 	}
// }

void MotionCard_ChangeTimer(TMRISR Timer)
{
	// --- 串接中斷服務函式 ---
	/*  IMC_TMR_SetISRFunction(
	TMRISR   myTMR_ISR,			// User 自己撰寫的 Timer 中斷副程式之 Function Pointer
	WORD wCardIndex)*/
	IMC_TMR_SetISRFunction(Timer, 0); // 串接中斷服務函式
}
