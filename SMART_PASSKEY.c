#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>

#define LED_PIN 4
#define LED_MASK 0x00001U
#define NOMAL 0U
#define ERROR 1U

// 信号開始合図のエラー値
#define START_SHORT_ON 9U
#define START_LONG_ON 11U
#define START_SHORT_OFF 39U
#define START_LONG_OFF 41U

// 表示停止合図のエラー値
#define END_SHORT_ON 14U
#define END_LONG_ON 16U
#define END_SHORT_OFF 34U
#define END_LONG_OFF 36U

// エラー発生タイミング
#define ERROR_SIGNAL_STOP 0U  // 停止時
#define ERROR_SIGNAL_START 1U // 開始時

// エラー発生条件（信号状態）
#define ERROR_SIGNAL_OFF 0U // OFF時
#define ERROR_SIGNAL_ON 1U  // ON時

// エラーパターンエラー値
#define SIGNAL_OFF 0U
#define SIGNAL_ON 1U

// パターン順序エラーパターン
#define PATTERN_OFF 0U        // OFF連続
#define PATTERN_ON 1U         // ON連続
#define PATTERN_INCOMPLETE 2U // パターン不足
#define SIGNAL_NONE (-1)      // 前回値がない

static void start_sign(void);
static void start_on_err_status_check(void);
static void start_off_err_status_check(void);
static void stop_sign(void);
static void stop_on_err_status_check(void);
static void stop_off_err_status_check(void);
static void send_data(char *arg_str, int bit_num);
static void raise_timing_error(char trigger_time_ms, char error_timing, char signal_state);
void run_signal_pattern_error(char error_timing, char error_pattern, char count);

static int S_On_Count;  /* 開始合図点灯データ送信カウント */
static int S_Off_Count; /* 開始合図消灯データ送信カウント */
static int E_On_Count;  /* 停止合図点灯データ送信カウント */
static int E_Off_Count; /* 停止合図消灯データ送信カウント */
static int Err_status;  /* エラー状態 */

int main(int argc, char *argv[])
{
    int led_mode;
    Err_status = 0;

    wiringPiSetupGpio();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    if (argc >= 4)
    {
        printf("引数エラーです。多いです。\n");
        return 1;
    }

    if (strcmp(argv[1], "start") == 0)
    {
        if (argc < 3)
        {
            printf("引数エラーです。少ないです。\n");
            return 1;
        }
        if (strlen(argv[2]) == 6)
        {
            //			check_num(argv[2]);		//すべての文字が数字かチェック
            bool is_all_digit = TRUE;
            for (int i = 0; argv[2][i] != '\0'; i++)
            { // argv[1]文字列のi番目の文字を取得
                if (!isdigit(argv[2][i]))
                {
                    is_all_digit = FALSE; // 数字でない文字が見つかった
                    break;
                }
            }
            if (is_all_digit == TRUE)
            {
                printf("'%s' は6桁の数字です。\n", argv[2]);
                int num = atoi(argv[2]); // 6桁の数字を整数とする
                // 開始合図
                start_sign();
                if (Err_status != ERROR)
                {
                    // データ送信
                    send_data(argv[2], num);
                    digitalWrite(LED_PIN, HIGH);
                }
            }
            else
            {
                printf("'%s' は数字を含んでいません。\n", argv[2]);
                digitalWrite(LED_PIN, HIGH);
            }
        }
        else if (strlen(argv[2]) == 7)
        {
            bool is_all_digit = TRUE;
            for (int i = 0; argv[2][i] != '\0'; i++)
            { // argv[1]文字列のi番目の文字を取得
                if (!isdigit(argv[2][i]))
                {
                    is_all_digit = FALSE; // 数字でない文字が見つかった
                    break;
                }
            }
            if (is_all_digit == TRUE)
            {
                printf("'%s' は7桁の数字です。\n", argv[2]);
                int num = atoi(argv[2]); // 7桁の数字を整数とする
                if (num <= 1048575)
                {
                    // 開始合図
                    start_sign();
                    if (Err_status != ERROR)
                    {
                        // データ送信
                        send_data(argv[2], num);
                        digitalWrite(LED_PIN, HIGH);
                    }
                }
                else
                {
                    printf("'%s' は最大値を超えています。\n", argv[2]);
                    digitalWrite(LED_PIN, HIGH);
                }
            }
            else
            {
                printf("'%s' は数字を含んでいません。\n", argv[2]);
                digitalWrite(LED_PIN, HIGH);
            }
        }
        else
        {
            printf("'%s' は6桁でも7桁でもありません。\n", argv[2]);
            digitalWrite(LED_PIN, HIGH);
        }
    }
    else if (strcmp(argv[1], "error_type") == 0)
    {
        if (argc < 3)
        {
            printf("引数エラーです。少ないです。/n");
            return 1;
        }

        if ((strlen(argv[2]) >= 1) && (strlen(argv[2]) <= 2))
        {
            bool is_digit = TRUE;
            for (int i = 0; argv[2][i] != '\0'; i++)
            {
                // argv[1]文字列のi番目の文字を取得
                if (!isdigit(argv[2][i]))
                {
                    is_digit = FALSE; // 数字でない文字が見つかった
                    break;
                }
            }
            if (is_digit == TRUE)
            {
                int num = atoi(argv[2]); // 2桁の数字を整数とする
                if ((num >= 1) && (num <= 14))
                {
                    printf("'%s' は入力範囲内です。\n", argv[2]);
                    switch (num)
                    {
                    case 1:
                        /* 開始 / 短すぎるON期間 */
                        raise_timing_error(9 - 1, ERROR_SIGNAL_START, ERROR_SIGNAL_ON);
                        break;

                    case 2:
                        /* 開始 / 長すぎるON期間 */
                        raise_timing_error(11 + 1, ERROR_SIGNAL_START, ERROR_SIGNAL_ON);
                        break;

                    case 3:
                        /* 開始 / 短すぎるOFF期間 */
                        raise_timing_error(39 - 1, ERROR_SIGNAL_START, ERROR_SIGNAL_OFF);
                        break;

                    case 4:
                        /* 開始 / 長すぎるON期間 */
                        raise_timing_error(41 + 1, ERROR_SIGNAL_START, ERROR_SIGNAL_OFF);
                        break;

                    case 5:
                        /* 開始 / ON連続 */
                        run_signal_pattern_error(ERROR_SIGNAL_START, PATTERN_ON, 4 - 1);
                        break;

                    case 6:
                        /* 開始 / OFF連続 */
                        run_signal_pattern_error(ERROR_SIGNAL_START, PATTERN_OFF, 4 - 1);
                        break;

                    case 7:
                        /* 開始 / パターン不足 */
                        run_signal_pattern_error(ERROR_SIGNAL_START, PATTERN_INCOMPLETE, 4 - 2);
                        break;

                    case 8:
                        /* 停止 / 短すぎるON期間 */
                        raise_timing_error(14 - 1, ERROR_SIGNAL_STOP, ERROR_SIGNAL_ON);
                        break;

                    case 9:
                        /* 停止 / 長すぎるON期間 */
                        raise_timing_error(16 + 1, ERROR_SIGNAL_STOP, ERROR_SIGNAL_ON);
                        break;

                    case 10:
                        /* 停止 / 短すぎるOFF期間 */
                        raise_timing_error(34 - 1, ERROR_SIGNAL_STOP, ERROR_SIGNAL_OFF);
                        break;

                    case 11:
                        /* 停止 / 長すぎるOFF期間 */
                        raise_timing_error(36 + 1, ERROR_SIGNAL_START, ERROR_SIGNAL_OFF);
                        break;

                    case 12:
                        /* 停止 / ON連続 */
                        run_signal_pattern_error(ERROR_SIGNAL_STOP, PATTERN_ON, 4 - 1);
                        break;

                    case 13:
                        /* 停止 / OFF連続 */
                        run_signal_pattern_error(ERROR_SIGNAL_STOP, PATTERN_OFF, 4 - 1);
                        break;

                    case 14:
                        /* 停止 / パターン不足 */
                        run_signal_pattern_error(ERROR_SIGNAL_STOP, PATTERN_INCOMPLETE, 4 - 2);
                        break;
                    }
                    printf("error_type: %d\n", num);
                }
                else
                {
                    printf("'%s' は最大値を超えています。\n", argv[2]);
                    digitalWrite(LED_PIN, HIGH);
                }
            }
            else
            {
                printf("'%s' は数字以外を含んでいます。\n", argv[2]);
                digitalWrite(LED_PIN, HIGH);
            }
        }
        else
        {
            printf("'%s' は入力範囲外です。1桁以上2桁以下で入力してください。\n", argv[2]);
            digitalWrite(LED_PIN, HIGH);
        }
    }
    else if (strcmp(argv[1], "stop") == 0)
    {
        stop_sign();
        if (Err_status != ERROR)
        {
            digitalWrite(LED_PIN, HIGH);
        }
    }
    else
    {
        printf("'%s' は未対応です。\n", argv[1]);
        digitalWrite(LED_PIN, HIGH);
    }
    printf("'%s' は終了です。\n", argv[1]);
    digitalWrite(LED_PIN, HIGH);
    return 0;
}

void raise_timing_error(char trigger_time_ms, char error_timing, char signal_state)
{
    int c = 0;

    for (int j = 0; j <= 1; j++)
    {
        S_On_Count = 0;
        printf("開始合図です。\n");

        if (error_timing == ERROR_SIGNAL_START)
        {
            /* 開始合図 */
            for (c = 0; c < trigger_time_ms; c++)
            {
                digitalWrite(LED_PIN, LOW);
                delay(10);
            }

            Err_status = NOMAL;
            if (signal_state == ERROR_SIGNAL_ON)
            {
                S_On_Count = c;
                start_on_err_status_check();
            }
            else if (signal_state == ERROR_SIGNAL_OFF)
            {
                /* 停止合図 */
                S_Off_Count = c;
                start_off_err_status_check();
            }
            if (Err_status == ERROR)
            {
                printf("開始合図エラーです。\n");
                break;
            }
        }
        else if (error_timing == ERROR_SIGNAL_STOP)
        {
            /* 信号停止時 */
            for (c = 0; c < trigger_time_ms; c++)
            {
                digitalWrite(LED_PIN, LOW);
                delay(10);
            }

            Err_status = NOMAL;
            if (signal_state == ERROR_SIGNAL_ON)
            {
                E_On_Count = c;
                stop_on_err_status_check();
            }
            else if (signal_state == ERROR_SIGNAL_OFF)
            {
                E_Off_Count = c;
                stop_off_err_status_check();
            }
            if (Err_status == ERROR)
            {
                printf("停止合図エラーです。\n");
                break;
            }
        }
    }
}

void start_sign(void)
{
    for (int j = 0; j <= 1; j++)
    {
        S_On_Count = 0;
        printf("開始合図です。\n");

        /* 短すぎるON期間 */
        for (int c = 0; c < 9; c++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10);
            S_On_Count++;
        }
        start_on_err_status_check();
        if (Err_status == ERROR)
        {
            printf("開始合図エラーです。\n");
            break;
        }

        /* 短すぎるOFF期間 */
        S_Off_Count = 0;
        printf("開始合図です。\n");

        for (int c = 0; c < 39; c++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(10);
            S_Off_Count++;
        }
        start_on_err_status_check();
        if (Err_status == ERROR)
        {
            printf("開始合図エラーです。\n");
            break;
        }
    }
}

void start_on_err_status_check(void)
{
    Err_status = NOMAL;
    if ((S_On_Count < START_SHORT_ON) || (S_On_Count > START_LONG_ON))
    {
        Err_status = ERROR;
    }
}
void start_off_err_status_check(void)
{
    Err_status = NOMAL;
    if ((S_Off_Count < START_SHORT_OFF) || (S_Off_Count > START_LONG_OFF))
    {
        Err_status = ERROR;
    }
}

void stop_sign(void)
{
    for (int j = 0; j <= 1; j++)
    {
        printf("表示停止合図です。\n");

        /* 短すぎるON期間 */
        E_On_Count = 0;
        for (int c = 0; c < 14; c++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10);
            E_On_Count++;
        }
        stop_on_err_status_check();
        if (Err_status == ERROR)
        {
            printf("表示停止合図エラーです。\n");
            break;
        }

        /* 長すぎるON期間 */
        E_On_Count = 0;
        for (int c = 0; c < 16; c++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10);
            E_On_Count++;
        }
        stop_on_err_status_check();
        if (Err_status == ERROR)
        {
            printf("表示停止合図エラーです。E1\n");
            break;
        }

        printf("表示停止合図です。\n");

        /* 短すぎるOFF期間 */
        E_Off_Count = 0;
        for (int c = 0; c < 34; c++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(10);
            E_Off_Count++;
        }
        stop_off_err_status_check();
        if (Err_status == ERROR)
        {
            printf("表示停止合図エラーです。\n");
            break;
        }

        /* 長すぎるOFF期間 */
        E_Off_Count = 0;
        for (int c = 0; c < 36; c++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(10);
            E_Off_Count++;
        }
        stop_off_err_status_check();
        if (Err_status == ERROR)
        {
            printf("表示停止合図エラーです。E2\n");
            break;
        }
    }
}

void stop_on_err_status_check(void)
{
    Err_status = NOMAL;
    if ((E_On_Count < END_SHORT_ON) || (E_On_Count > END_LONG_ON))
    {
        Err_status = ERROR;
    }
}
void stop_off_err_status_check(void)
{
    Err_status = NOMAL;
    if ((E_Off_Count < END_SHORT_OFF) || (E_Off_Count > END_LONG_OFF))
    {
        Err_status = ERROR;
    }
}

void run_signal_pattern_error(char error_timing, char error_pattern, char count)
{
    const int signal_pattern_on[4] = {SIGNAL_ON, SIGNAL_ON, SIGNAL_OFF, SIGNAL_ON};    // 2回連続ONを含むパターン
    const int signal_pattern_off[4] = {SIGNAL_ON, SIGNAL_OFF, SIGNAL_OFF, SIGNAL_OFF}; // 2回連続OFFを含むパターン
    const int signal_patterns_incomplete[3] = {SIGNAL_ON, SIGNAL_OFF, SIGNAL_ON};      // パターン不足
    int prev_signal_state = SIGNAL_NONE;                                               // 前回値保存
    int curr_signal_state = SIGNAL_NONE;                                               // 今回値保存
    int i = 0;

    if (count == (4 - 1))
    {

        for (i = 0; i < count; i++)
        {
            if (error_pattern == PATTERN_ON)
            {
                curr_signal_state = signal_pattern_on[i];
            }
            else if (error_pattern == PATTERN_OFF)
            {
                curr_signal_state = signal_pattern_off[i];
            }

            if (error_timing == ERROR_SIGNAL_START)
            {
                if (curr_signal_state == SIGNAL_ON)
                {
                    digitalWrite(LED_PIN, LOW);
                    delay(100);
                }
                else
                {
                    digitalWrite(LED_PIN, HIGH);
                    delay(400);
                }
            }
            else if (error_timing == ERROR_SIGNAL_STOP)
            {
                if (curr_signal_state == SIGNAL_ON)
                {
                    digitalWrite(LED_PIN, LOW);
                    delay(150);
                }
                else
                {
                    digitalWrite(LED_PIN, HIGH);
                    delay(350);
                }
            }

            if (i == 4)
            {
                printf("任務完了テスト用\n");
            }
            if (prev_signal_state == curr_signal_state)
            {
                printf("パターンエラーです。不正な順序です。\n");
                i = count;
            }
            else
            {
                prev_signal_state = curr_signal_state;
            }
        }
    }
    else
    {
        printf("パターンエラーです。パターンが不足しています。\n");
    }
}

void send_data(char *arg_str, int bit_num)
{
    bool is_led_light;

    printf("データ送信します。\n");
    for (int i = 19; i >= 0; i--)
    {
        if (((bit_num >> i) & LED_MASK) != 0)
        {
            is_led_light = TRUE;
        }
        else
        {
            is_led_light = FALSE;
        }
        if (is_led_light == TRUE)
        {
            printf("'%s'の'%d'bit目は点灯です。\n", arg_str, i);
            digitalWrite(LED_PIN, LOW);
        }
        else
        {
            printf("'%s'の'%d'bit目は消灯です。\n", arg_str, i);
            digitalWrite(LED_PIN, HIGH);
        }
        delay(100);
    }
}
