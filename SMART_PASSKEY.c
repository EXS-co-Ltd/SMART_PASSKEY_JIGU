#include<wiringPi.h>
#include<stdio.h>
#include<string.h>
#include<ctype.h>
#include<stdlib.h>
#include<stdbool.h>

#define LED_PIN 4
#define LED_MASK 0x00001U
#define NOMAL	0U
#define ERROR	1U

//信号開始合図のエラー値
#define START_SHORT_ON	9U
#define START_LONG_ON	11U
#define START_SHORT_OFF	39U
#define START_LONG_OFF	41U

//表示停止合図のエラー値
#define END_SHORT_ON	14U
#define END_LONG_ON		16U
#define END_SHORT_OFF	34U
#define END_LONG_OFF	36U

static void start_sign(void);
static void start_on_err_status_check (void);
static void start_off_err_status_check (void);
static void stop_sign(void);
static void stop_on_err_status_check (void);
static void stop_off_err_status_check (void);
static void send_data(char *arg_str, int bit_num);

static int	S_On_Count;		/* 開始合図点灯データ送信カウント */
static int	S_Off_Count;	/* 開始合図消灯データ送信カウント */
static int	E_On_Count;		/* 停止合図点灯データ送信カウント */
static int	E_Off_Count;	/* 停止合図消灯データ送信カウント */
static int	Err_status;		/* エラー状態 */

int main(int argc, char *argv[])
{
	int led_mode;
	Err_status = 0;

	wiringPiSetupGpio();
	pinMode(LED_PIN,OUTPUT);
	digitalWrite(LED_PIN,HIGH);
	
	if (argc >= 4) {
		printf("引数エラーです。多いです。\n");
		return 1;
	}

	if (strcmp(argv[1], "start") == 0) {
		if (argc < 3) {
			printf("引数エラーです。少ないです。\n");
			return 1;
		}
		if (strlen(argv[2]) == 6) {

//			check_num(argv[2]);		//すべての文字が数字かチェック
			bool is_all_digit = TRUE;
			for (int i = 0; argv[2][i] != '\0'; i++) {		//argv[1]文字列のi番目の文字を取得
				if (!isdigit(argv[2][i])) {
					is_all_digit = FALSE; 			//数字でない文字が見つかった
					break;
				}
			}
			if (is_all_digit == TRUE) {
				printf("'%s' は6桁の数字です。\n", argv[2]);
				int num = atoi(argv[2]);		//6桁の数字を整数とする
				//開始合図
				start_sign();
				if (Err_status != ERROR) {
					//データ送信
					send_data(argv[2], num);
					digitalWrite(LED_PIN,HIGH);
				}
			} else {
				printf("'%s' は数字を含んでいません。\n", argv[2]);
				digitalWrite(LED_PIN,HIGH);
			}
		} else if (strlen(argv[2]) == 7) {
			bool is_all_digit = TRUE;
			for (int i = 0; argv[2][i] != '\0'; i++) {		//argv[1]文字列のi番目の文字を取得
				if (!isdigit(argv[2][i])) {
					is_all_digit = FALSE; 			//数字でない文字が見つかった
					break;
				}
			}
			if (is_all_digit == TRUE) {
				printf("'%s' は7桁の数字です。\n", argv[2]);
				int num = atoi(argv[2]);		//7桁の数字を整数とする
				if (num <= 1048575) {
					//開始合図
					start_sign();
					if (Err_status != ERROR) {
						//データ送信
						send_data(argv[2], num);
						digitalWrite(LED_PIN,HIGH);
					}
				} else {
					printf("'%s' は最大値を超えています。\n", argv[2]);
					digitalWrite(LED_PIN,HIGH);
				}
			} else {
				printf("'%s' は数字を含んでいません。\n", argv[2]);
				digitalWrite(LED_PIN,HIGH);
			}
		} else {
			printf("'%s' は6桁でも7桁でもありません。\n", argv[2]);
			digitalWrite(LED_PIN,HIGH);
		}
	} else if (strcmp(argv[1], "stop") == 0) {
		stop_sign();
		if (Err_status != ERROR) {
			digitalWrite(LED_PIN,HIGH);
		}
	} else {
		printf("'%s' は未対応です。\n", argv[1]);
		digitalWrite(LED_PIN,HIGH);
	}
	printf("'%s' は終了です。\n", argv[1]);
	digitalWrite(LED_PIN,HIGH);
	return 0;
}



void start_sign (void)
{
	for (int j = 0; j <= 1; j++){
		S_On_Count = 0;
		S_Off_Count = 0;
		printf("開始合図です。\n");
		for (int c = 0; c <= 7; c++) {
			digitalWrite(LED_PIN,LOW);
			delay(10);
			S_On_Count++;
		}
		start_on_err_status_check();
		if (Err_status == ERROR) {
			printf("開始合図エラーです。\n");
			break;
		}
		printf("開始合図です。\n");
		for (int c = 0; c <= 39; c++) {
			digitalWrite(LED_PIN,HIGH);
			delay(10);
			S_Off_Count++;
		}
		start_off_err_status_check();
		if (Err_status == ERROR) {
			printf("開始合図エラーです。\n");
			break;
		}

	}
}

void start_on_err_status_check (void)
{
	Err_status = NOMAL;
	if ((S_On_Count <= START_SHORT_ON) || (S_On_Count >= START_LONG_ON)) {
		Err_status = ERROR;
	}
}
void start_off_err_status_check (void)
{
	Err_status = NOMAL;
	if ((S_Off_Count <= START_SHORT_OFF) || (S_Off_Count >= START_LONG_OFF)) {
		Err_status = ERROR;
	}
}


void stop_sign (void)
{
	for (int j = 0; j <= 1; j++){
		E_On_Count = 0;
		E_Off_Count = 0;
		printf("表示停止合図です。\n");
		for (int c = 0; c <= 14; c++) {
			digitalWrite(LED_PIN,LOW);
			delay(10);
			E_On_Count++;
		}
		stop_on_err_status_check();
		if (Err_status == ERROR) {
			printf("表示停止合図エラーです。\n");
			break;
		}
		printf("表示停止合図です。\n");
		for (int c = 0; c <= 34; c++) {
			digitalWrite(LED_PIN,HIGH);
			delay(10);
			E_Off_Count++;
		}
		stop_off_err_status_check();
		if (Err_status == ERROR) {
			printf("表示停止合図エラーです。\n");
			break;
		}

	}
}

void stop_on_err_status_check (void)
{
	Err_status = NOMAL;
	if ((E_On_Count <= END_SHORT_ON) || (E_On_Count >= END_LONG_ON)) {
		Err_status = ERROR;
	}
}
void stop_off_err_status_check (void)
{
	Err_status = NOMAL;
	if ((E_Off_Count <= END_SHORT_OFF) || (E_Off_Count >= END_LONG_OFF)) {
		Err_status = ERROR;
	}
}


void send_data (char *arg_str, int bit_num)
{
	bool is_led_light;

	printf("データ送信します。\n");
	for (int i = 19; i >= 0; i--){
		if (((bit_num >> i) & LED_MASK) != 0) {
			is_led_light = TRUE;
		} else {
			is_led_light = FALSE;
		}
		if (is_led_light == TRUE) {
			printf("'%s'の'%d'bit目は点灯です。\n", arg_str,i);
			digitalWrite(LED_PIN,LOW);
		} else {
			printf("'%s'の'%d'bit目は消灯です。\n", arg_str,i);
			digitalWrite(LED_PIN,HIGH);
		}
		delay(100);
	}
}



