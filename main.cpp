#include "mbed.h"
#include <math.h>	//絶対値を使うために必要

#define z 0.004		//光センサの左右の個体差を無くす
#define p 0.81		//比例ゲイン
#define i 0.002		//積分ゲイン
#define d 0.022		//微分ゲイン

#define TR 0.45		//右光センサ
#define TC 0.5		//中央光センサ
#define TL 0.45		//左光センサ

DigitalIn	start_sw(dp12,PullUp);	//スタートスイッチ

DigitalIn dummy1(dp2,PullNone);
AnalogIn	Ir_L(dp2);				//光センサ　左
DigitalIn dummy2(dp24,PullNone);
AnalogIn	Ir_C(dp24);				//光センサ　中央
DigitalIn dummy3(dp23,PullNone);
AnalogIn	Ir_R(dp23);				//光センサ　右

DigitalIn dummy4(dp22,PullNone);
AnalogIn	SOKKYO(dp22);			//測距センサ前
DigitalIn dummy5(dp21,PullNone);
AnalogIn	SOKUIKI(dp21);			//測距センサ上（トンネル検知）

DigitalOut	ain1(dp7);
DigitalOut	ain2(dp8);
PwmOut		pwma(dp9);		//Rモータ
DigitalOut	bin1(dp15);
DigitalOut	bin2(dp16);
PwmOut		pwmb(dp17);		//Lモータ

PwmOut	servo(dp18);		//サーボモーター

Serial pc(USBTX,USBRX);		//	USB出力

int  main()//-------------------------------------------------------------------------------------------------------------
{
	pwma.period_ms(20);
	pwmb.period_ms(20);
	servo.period_ms(20);
	
	
	int state = 0;		//関数　state
	
	float IR, IC, IL;	//光センサのしきい値
	
	float hensa1 = 0,hensa2 = 0;	//PID制御で使う関数
	float KP=0,KI=0,KD=0,PID=0;	//PID制御で使う関数
	
	while(1)
	{
		if(start_sw == 0)	//スタートスイッチが押されたらスタート---------------------------------------------------------------------------
		{
			servo.pulsewidth_us(2300);
			state = 1;
		}
		
		else if(state == 1)//[1]ライントレース------------------------------------------------------------------------------------
		{
			while(1)
			{
				IL = Ir_L; IC = Ir_C; IR = Ir_R;
				hensa1 = IL - IR + (z);					//光センサの左右の差
				KP = hensa1;								//比例制御
				KI = KI + (hensa1 + hensa2)*0.01;			//積分制御
				KD = fabsf((hensa1 - hensa2)/0.02);		//微分制御
				PID = ((KP) * (p)) + ((KI) * (i)) + ((KD) * (d));	//PID制御
				
				if( hensa1 > 0.05 )//ロボットが黒線よりも左に傾いているとき
				{
					ain1 = 1;
					ain2 = 0;
					pwma = 1 - PID;
					bin1 = 1;
					bin2 = 0;
					pwmb = 1;
				}
				
				else if( hensa1 < -0.05 )//ロボットが黒線よりも右に傾いているとき
				{
					ain1 = 1;
					ain2 = 0;
					pwma = 1;
					bin1 = 1;
					bin2 = 0;
					pwmb = 1 + PID;
					
				}
				
				else //ロボットが黒線の真ん中にいるとき
				{
					ain1 = 1;
					ain2 = 0;
					pwma = 0.8 - PID;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.8 + PID;
					
					KI = 0;
				} 
				
				hensa2 = hensa1;
				
				
				if( SOKUIKI > 0.6)//トンネルを検知
				{
					state = 2;
					break;
				}
				pc.printf("state=1\r\n");
			}
		}
		
		else if( state == 2 )//[2]トンネル内------------------------------------------------------------------------------------
		{
			for( int k = 0; k <= 1900; k++ )
			{
				IL = Ir_L; IC = Ir_C; IR = Ir_R;
				
				if(IL > TL && IC > TC && IR > TR)//白ー白ー白 前の動作の繰り返し
				{
					wait(0.1);
					if(IL > TL && IC > TC && IR > TR)
					{
						servo.pulsewidth_us(2400);
					}
				}
					
				else if(IL < TL && IC > TC && IR > TR)//黒ー白ー白
				{
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.8;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.7;		
				}
					
				else if(IL > TL && IC < TC && IR > TR)//白ー黒ー白
				{
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.6;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.6;		
				}
					
				else if(IL > TL && IC > TC && IR < TR)//白ー白ー黒
				{		
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.7;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.8;		
				}
			}
					
			servo.pulsewidth_us(2400);//トンネル内で3秒止まる
			ain1 = 1;
			ain2 = 1;
			pwma = 0.8;
			bin1 = 1;
			bin2 = 1;
			pwmb = 0.8;
			wait(2.5);
			
			pc.printf("state=2\r\n");
			
			state = 3;
		}
		
		else if(state == 3)//[3]ライントレース------------------------------------------------------------------------------------
		{
			while(1)
			{
				IL = Ir_L; IC = Ir_C; IR = Ir_R;
				hensa1 = IL - IR + (z);					//光センサの左右の差
				KP = hensa1;								//比例制御
				KI = KI + (hensa1 + hensa2)*0.01;			//積分制御
				KD = fabsf((hensa1 - hensa2)/0.02);		//微分制御
				PID = ((KP) * (p)) + ((KI) * (i)) + ((KD) * (d));
				
				if( hensa1 > 0.05 )//ロボットが黒線よりも左に傾いているとき
				{
					ain1 = 1;
					ain2 = 0;
					pwma = 1 - PID;
					bin1 = 1;
					bin2 = 0;
					pwmb = 1;
				}
				
				else if( hensa1 < -0.05 )//ロボットが黒線よりも右に傾いているとき
				{
					ain1 = 1;
					ain2 = 0;
					pwma = 1;
					bin1 = 1;
					bin2 = 0;
					pwmb = 1 + PID;
					
				}
				
				else //ロボットが黒線の真ん中にいるとき
				{
					ain1 = 1;
					ain2 = 0;
					pwma = 0.8 - PID;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.8 + PID;
					
					KI = 0;
				}
				
				hensa2 = hensa1;
				
				if(IL < TL && IC < TC && IR < TR)//黒ー黒ー黒
				{
					wait(0.05);
					if(IL < TL && IC < TC && IR < TR)
					{
						state = 4;
						break;
					}
				}
				pc.printf("state=3\r\n");
			}
		}
		
		else if(state == 4)//[4]黒線検知 ・ 槍を倒す--------------------------------------------------------------------------------------
		{
			
			for( int s = 0; s <= 1000; s++)
			{
				IL = Ir_L; IC = Ir_C; IR = Ir_R;
				
				
				if(IL > TL && IC > TC && IR > TR)//白ー白ー白 前の動作の繰り返し
				{
					wait(0.1);
					if(IL > TL && IC > TC && IR > TR)
					{
						servo.pulsewidth_us(1600);
					}
				}
				
				else if(IL < TL && IC > TC && IR > TR)//黒ー白ー白
				{
					servo.pulsewidth_us(1600);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.8;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.6;		
				}
				
				else if(IL > TL && IC < TC && IR > TR)//白ー黒ー白
				{
					servo.pulsewidth_us(1600);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.7;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.7;		
				}
				
				else if(IL > TL && IC > TC && IR < TR)//白ー白ー黒
				{
					servo.pulsewidth_us(1600);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.6;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.8;		
				}
				pc.printf("state=4\r\n");
			}
			
			state = 5;
		}
		
		
		else if(state == 5)//[5]ライントレース-------------------------------------------------------------------------------------
		{
			while(1)
			{
				IL = Ir_L; IC = Ir_C; IR = Ir_R;
				hensa1 = IL - IR + (z);					//光センサの左右の差
				KP = hensa1;								//比例制御
				KI = KI + (hensa1 + hensa2)*0.01;			//積分制御
				KD = fabsf((hensa1 - hensa2)/0.02);		//微分制御
				PID = ((KP) * (p)) + ((KI) * (i)) + ((KD) * (d));
				
				if( hensa1 > 0.05 )//ロボットが黒線よりも左に傾いているとき
				{
					ain1 = 1;
					ain2 = 0;
					pwma = 1 - PID;
					bin1 = 1;
					bin2 = 0;
					pwmb = 1;
				}
				
				else if( hensa1 < -0.05 )//ロボットが黒線よりも右に傾いているとき
				{
					ain1 = 1;
					ain2 = 0;
					pwma = 1;
					bin1 = 1;
					bin2 = 0;
					pwmb = 1 + PID;
					
				}
				
				else //ロボットが黒線の真ん中にいるとき
				{
					ain1 = 1;
					ain2 = 0;
					pwma = 0.8 - PID;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.8 + PID;
					
					KI = 0;
				}
				
				hensa2 = hensa1;
				
				if(IL < TL && IC < TC && IR < TR)//グレーゾーン検知
				{
					wait(0.05);
					if(IL < 0.5 && IC < 0.5 && IR < 0.5)//黒ー黒ー黒
					{
						ain1 = 1;
						ain2 = 0;
						pwma = 0.6;
						bin1 = 1;
						bin2 = 0;
						pwmb = 0.6;
						wait(0.5);
					
						state = 6;
						break;
					}
				}
				pc.printf("state=5\r\n");
			}
		}
		
		else if( state == 6 )//[6]壁検知----------------------------------------------------------------------------------------
		{
			while(1)
			{
				IL = Ir_L; IC = Ir_C; IR = Ir_R;
				
			
				ain1 = 1;
				ain2 = 0;
				pwma = 0.7;
				bin1 = 1;
				bin2 = 0;
				pwmb = 0.7;	//直進
				
				
				if( SOKKYO > 0.7 )//壁検知
				{
					state = 7;
					break;
				}
				pc.printf("state=6\r\n");
			}
		}
		
		else if(state == 7)//[7]ターン----------------------------------------------------------------------------------------
		{
			while(1)
			{
				IL = Ir_L; IC = Ir_C; IR = Ir_R;
				
				ain1 = 1;
				ain2 = 0;
				pwma = 0.2;
				bin1 = 1;
				bin2 = 0;
				pwmb = 0.8;		//右折
				
				if(IL > 0.6 || IC > 0.6 || IR > 0.6)//白検知
				{
					if(IL < 0.4 || IC < 0.4 || IR < 0.4)//黒検知
					{
						state = 8;
						break;
					}
				}
				pc.printf("state=7\r\n");
			}		
		}
		
		else if(state == 8)//[8]ライントレース---------------------------------------------------------------------------------------
		{
			while(1)
			{
				IL = Ir_L; IC = Ir_C; IR = Ir_R;
				
				if(IL < TL && IC < TC && IR < TR)//黒ー黒ー黒
				{
					wait(0.05);
					if(IL < TL && IC < TC && IR < TR)
					{
						state = 9;
						break;
					}
				}
				
				else if(IL > TL && IC > TC && IR > TR)//白ー白ー白 前の動作の繰り返し
				{
					wait(0.1);
					if(IL > TL && IC > TC && IR > TR)
					{
						servo.pulsewidth_us(2400);
					}
				}
				
				else if(IL < TL && IC > TC && IR > TR)//黒ー白ー白
				{
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.8;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.6;		
				}
				
				else if(IL > TL && IC < TC && IR > TR)//白ー黒ー白
				{
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.7;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.7;		
				}
				
				else if(IL > TL && IC > TC && IR < TR)//白ー白ー黒
				{
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.6;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.8;		
				}
				pc.printf("state=8\r\n");
			}
		}
		
		else if(state == 9)//[9]槍を倒す---------------------------------------------------------------------------------------
		{
			servo.pulsewidth_us(1850);
			wait(0.2);
			
			pc.printf("state=9\r\n");
			
			state = 10;
		}
		
		else if(state == 10)//[10]ライントレース----------------------------------------------------------------------------------------
		{
			while(1)
			{
				IL = Ir_L; IC = Ir_C; IR = Ir_R;
				
				if(IL > TL && IC > TC && IR > TR)//白ー白ー白 前の動作の繰り返し
				{
					wait(0.1);
					if(IL > TL && IC > TC && IR > TR)
					{
						servo.pulsewidth_us(2400);
					}
				}
				
				else if(IL < TL && IC > TC && IR > TR)//黒ー白ー白
				{
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.8;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.7;		
				}
				
				else if(IL > TL && IC < TC && IR > TR)//白ー黒ー白
				{
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.6;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.6;		
				}
				
				else if(IL > TL && IC > TC && IR < TR)//白ー白ー黒
				{	
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 0;
					pwma = 0.7;
					bin1 = 1;
					bin2 = 0;
					pwmb = 0.8;		
				}
				
				if( SOKKYO > 0.9 )//ガレージの壁を検知
				{
					servo.pulsewidth_us(2400);
					ain1 = 1;
					ain2 = 1;
					pwma = 1.0;
					bin1 = 1;
					bin2 = 1;
					pwmb = 1.0;
					
					state = 11;
					break;
				}
			}
			pc.printf("state=10\r\n");
		}
		
		else if(state == 11)//[11]停止----------------------------------------------------------------------
		{
			while(1)
			{
				servo.pulsewidth_us(2400);
				ain1 = 1;
				ain2 = 1;
				pwma = 1.0;
				bin1 = 1;
				bin2 = 1;
				pwmb = 1.0;
				wait(10);
				
				pc.printf("state=11\r\n");
				
			}
		}
	}
}
