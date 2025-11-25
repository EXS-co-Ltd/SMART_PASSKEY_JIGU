#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>

#define LED_PIN (4U)
#define LED_MASK (0x00001U)
#define NOMAL (0U) /* error_statusが正常 */
#define ERROR (1U) /* error_statusがエラー */

#define MAX_CMD_ARGS (4U) /* 多い引数 */
#define MIN_CMD_ARGS (3U) /* 少ない引数 */
#define EXIT_ERROR (1U)   /* エラー発生のリターン時 */

#define COUNT_MAX_INDEX (4U - 1U) /* 最大値（4回だが、0~3のため-1している） */
#define MAX_DATA_NUM (20 - 1)     /* 最大値（20回だが、0~19のため-1している） */

#define SIGNAL_PATTERN_MAX_INDEX  (4U - 1U)/* 最大値（4回だが、0~3のため-1している） */
#define SIGNAL_PATTERN_ERROR_INDEX  (3U - 1U)/* 最大値（3回だが、0~2のため-1している） */

/* "start"入力時に使用 */
#define ARG2_START_EXPECT_DIGITS (6U)          /* 正常（許容）桁数 */
#define ARG2_START_OVER_DIGITS (7U)            /* 許容より1多い */
#define ARG2_START_EXPECT_MAX_VALUE (1048575U) /* 最大値（20bitの最大値：0xFFFFF = 1048575） */

/* "error_type"入力時に使用 */
#define ARG2_ERROR_TYPE_DIGITS_MIN (1U) /* 最小1桁 */
#define ARG2_ERROR_TYPE_DIGITS_MAX (2U) /* 最大2桁 */
/* スタート時のエラー(データを10ms間隔でX回ずつ送信) */
#define START_ON_SHORT_MS (9U - 1U)   /* 基準90msから10ms短い値 */
#define START_ON_LONG_MS (11U + 1U)   /* 基準110msから10ms長い値 */
#define START_OFF_SHORT_MS (39U - 1U) /* 基準390msから10ms短い値 */
#define START_OFF_LONG_MS (41U + 1U)  /* 基準410msから10ms長い値 */
/* ストップ時のエラー(データを10ms間隔でX回ずつ送信) */
#define STOP_ON_SHORT_MS (14U - 1U)  /* 基準140msから10ms短い値 */
#define STOP_ON_LONG_MS (16U + 1U)   /* 基準160msから10ms長い値 */
#define STOP_OFF_SHORT_MS (34U - 1U) /* 基準340msから10ms短い値 */
#define STOP_OFF_LONG_MS (36U + 1U)  /* 基準360msから10ms長い値 */

/* 信号開始合図のエラー値(データを10ms間隔でX回ずつ送信) */
#define START_SHORT_ON (9U)   /* 基準値90ms */
#define START_LONG_ON (11U)   /* 基準値110ms */
#define START_SHORT_OFF (39U) /* 基準値390ms */
#define START_LONG_OFF (41U)  /* 基準値410ms */

/* 表示停止合図のエラー値(データを10ms間隔でX回ずつ送信) */
#define END_SHORT_ON (14U)  /* 基準値140ms */
#define END_LONG_ON (16U)   /* 基準値160ms */
#define END_SHORT_OFF (34U) /* 基準値340ms */
#define END_LONG_OFF (36U)  /* 基準値360ms */

/* エラー発生タイミング */
#define ERROR_SIGNAL_STOP (0U)  /* 停止時 */
#define ERROR_SIGNAL_START (1U) /* 開始時 */

/* エラー発生条件（信号状態）*/
#define ERROR_SIGNAL_OFF (0U) /* OFF時 */
#define ERROR_SIGNAL_ON (1U)  /* ON時 */

/* エラーパターンエラー値 */
#define SIGNAL_OFF (0U) /* OFF時 */
#define SIGNAL_ON (1U)  /* ON時 */

/* パターン順序エラーパターン */
#define PATTERN_OFF (0U)        /* OFF連続 */
#define PATTERN_ON (1U)         /* ON連続 */
#define PATTERN_INCOMPLETE (2U) /* パターン不足 */
#define SIGNAL_NONE (-1)        /* 前回値がない */

/* エラー種別番号 */
typedef enum
{
    ERROR_TYPE_1 = 1, /* 1番目のエラー種別 */
    ERROR_TYPE_2,     /* 2 */
    ERROR_TYPE_3,     /* 3 */
    ERROR_TYPE_4,     /* 4 */
    ERROR_TYPE_5,     /* 5 */
    ERROR_TYPE_6,     /* 6 */
    ERROR_TYPE_7,     /* 7 */
    ERROR_TYPE_8,     /* 8 */
    ERROR_TYPE_9,     /* 9 */
    ERROR_TYPE_10,    /* 10 */
    ERROR_TYPE_11,    /* 11 */
    ERROR_TYPE_12,    /* 12 */
    ERROR_TYPE_13,    /* 13 */
    ERROR_TYPE_14     /* 14 */
} ERROR_TYPE_t;

static void StartSign(void);
static void StartOnErrStatusCheck(void);
static void StartOffErrStatusCheck(void);
static void StopSign(void);
static void StopOnErrStatusCheck(void);
static void StopOffErrStatusCheck(void);
static void SendData(char *arg_str, int bit_num);
static void RaiseTimingError(char trigger_time_ms, char error_timing, char signal_state);
static void RunSignalPatternError(char error_timing, char error_pattern, char count);

static int start_on_count;  /* 開始合図点灯データ送信カウント */
static int start_off_count; /* 開始合図消灯データ送信カウント */
static int end_on_count;    /* 停止合図点灯データ送信カウント */
static int end_off_count;   /* 停止合図消灯データ送信カウント */
static int error_status;    /* エラー状態 */

int main(int argc, char *argv[])
{
    bool is_all_digit = TRUE; /* 数字のみのデータか確認用 */
    int num;                  /* 整数変換後データ格納用 */
    int i;
    error_status = 0;

    wiringPiSetupGpio();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    /* 入力引数が多いか確認 */
    if (argc >= MAX_CMD_ARGS)
    {
        printf("引数エラーです。多いです。\n");
        return EXIT_ERROR;
    }

    if (strcmp(argv[1], "start") == 0)
    {
        /* "start"入力時処理 */

        /* 入力引数が少ないか確認 */
        if (argc < MIN_CMD_ARGS)
        {
            printf("引数エラーです。少ないです。\n");
            return EXIT_ERROR;
        }

        /* 桁数確認 */
        if (strlen(argv[2]) == ARG2_START_EXPECT_DIGITS)
        {
            //			check_num(argv[2]);		//すべての文字が数字かチェック

            for (i = 0; argv[2][i] != '\0'; i++)
            {
                /* 文字列が数字か確認 */
                if (!isdigit(argv[2][i]))
                {
                    is_all_digit = FALSE; /* 数字ではない文字が含まれていた */
                    break;
                }
            }

            if (is_all_digit == TRUE)
            {
                printf("'%s' は%u桁の数字です。\n", argv[2], ARG2_START_EXPECT_DIGITS);
                num = atoi(argv[2]); /* 文字列を整数に変換 */

                /* 開始合図 */
                StartSign();
                if (error_status != ERROR)
                {
                    /* エラーがなければデータ送信 */
                    SendData(argv[2], num);
                    digitalWrite(LED_PIN, HIGH);
                }
            }
            else
            {
                /* 数字ではない文字が含まれていた */
                printf("'%s' は数字以外を含んでいます。\n", argv[2]);
                digitalWrite(LED_PIN, HIGH);
            }
        }
        else if (strlen(argv[2]) == ARG2_START_OVER_DIGITS)
        {
            /* 入力桁数が1多い */
            for (i = 0; argv[2][i] != '\0'; i++)
            {
                /* 文字列が数字か確認 */
                if (!isdigit(argv[2][i]))
                {
                    is_all_digit = FALSE; /* 数字ではない文字が含まれていた */
                    break;
                }
            }

            if (is_all_digit == TRUE)
            {
                printf("'%s' は%u桁の数字です。\n", argv[2], ARG2_START_OVER_DIGITS);
                num = atoi(argv[2]); /* 文字列を整数に変換 */

                if (num <= ARG2_START_EXPECT_MAX_VALUE)
                {
                    /* 開始合図 */
                    StartSign();
                    if (error_status != ERROR)
                    {
                        /* エラーがなければデータ送信 */
                        SendData(argv[2], num);
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
                printf("'%s' は数字以外を含んでいます。\n", argv[2]);
                digitalWrite(LED_PIN, HIGH);
            }
        }
        else
        {
            printf("'%s' は%u桁でも%u桁でもありません。\n", argv[2], ARG2_START_EXPECT_DIGITS, ARG2_START_OVER_DIGITS);
            digitalWrite(LED_PIN, HIGH);
        }
    }
    else if (strcmp(argv[1], "error_type") == 0)
    {
        /* "error_type"入力時処理 */

        /* 入力引数が少ないか確認 */
        if (argc < MIN_CMD_ARGS)
        {
            printf("引数エラーです。少ないです。/n");
            return EXIT_ERROR;
        }

        /* 桁数確認 */
        if ((strlen(argv[2]) >= ARG2_ERROR_TYPE_DIGITS_MIN) && (strlen(argv[2]) <= ARG2_ERROR_TYPE_DIGITS_MAX))
        {
            for (i = 0; argv[2][i] != '\0'; i++)
            {
                /* 文字列が数字か確認 */
                if (!isdigit(argv[2][i]))
                {
                    /* 数字ではない文字が含まれていた */
                    is_all_digit = FALSE;
                    break;
                }
            }

            if (is_all_digit == TRUE)
            {
                num = atoi(argv[2]); /* 文字列を整数に変換 */

                /* 範囲内のデータか確認 */
                if ((num >= ERROR_TYPE_1) && (num <= ERROR_TYPE_14))
                {
                    switch (num)
                    {
                    case ERROR_TYPE_1:
                        /* 開始 / 短すぎるON期間 */
                        RaiseTimingError(START_ON_SHORT_MS, ERROR_SIGNAL_START, ERROR_SIGNAL_ON);
                        break;

                    case ERROR_TYPE_2:
                        /* 開始 / 長すぎるON期間 */
                        RaiseTimingError(START_ON_LONG_MS, ERROR_SIGNAL_START, ERROR_SIGNAL_ON);
                        break;

                    case ERROR_TYPE_3:
                        /* 開始 / 短すぎるOFF期間 */
                        RaiseTimingError(START_OFF_SHORT_MS, ERROR_SIGNAL_START, ERROR_SIGNAL_OFF);
                        break;

                    case ERROR_TYPE_4:
                        /* 開始 / 長すぎるON期間 */
                        RaiseTimingError(START_OFF_LONG_MS, ERROR_SIGNAL_START, ERROR_SIGNAL_OFF);
                        break;

                    case ERROR_TYPE_5:
                        /* 開始 / ON連続 */
                        RunSignalPatternError(ERROR_SIGNAL_START, PATTERN_ON, SIGNAL_PATTERN_MAX_INDEX);
                        break;

                    case ERROR_TYPE_6:
                        /* 開始 / OFF連続 */
                        RunSignalPatternError(ERROR_SIGNAL_START, PATTERN_OFF, SIGNAL_PATTERN_MAX_INDEX);
                        break;

                    case ERROR_TYPE_7:
                        /* 開始 / パターン不足 */
                        RunSignalPatternError(ERROR_SIGNAL_START, PATTERN_INCOMPLETE, SIGNAL_PATTERN_ERROR_INDEX);
                        break;

                    case ERROR_TYPE_8:
                        /* 停止 / 短すぎるON期間 */
                        RaiseTimingError(STOP_ON_SHORT_MS, ERROR_SIGNAL_STOP, ERROR_SIGNAL_ON);
                        break;

                    case ERROR_TYPE_9:
                        /* 停止 / 長すぎるON期間 */
                        RaiseTimingError(STOP_ON_LONG_MS, ERROR_SIGNAL_STOP, ERROR_SIGNAL_ON);
                        break;

                    case ERROR_TYPE_10:
                        /* 停止 / 短すぎるOFF期間 */
                        RaiseTimingError(STOP_OFF_SHORT_MS, ERROR_SIGNAL_STOP, ERROR_SIGNAL_OFF);
                        break;

                    case ERROR_TYPE_11:
                        /* 停止 / 長すぎるOFF期間 */
                        RaiseTimingError(STOP_OFF_LONG_MS, ERROR_SIGNAL_START, ERROR_SIGNAL_OFF);
                        break;

                    case ERROR_TYPE_12:
                        /* 停止 / ON連続 */
                        RunSignalPatternError(ERROR_SIGNAL_STOP, PATTERN_ON, SIGNAL_PATTERN_MAX_INDEX);
                        break;

                    case ERROR_TYPE_13:
                        /* 停止 / OFF連続 */
                        RunSignalPatternError(ERROR_SIGNAL_STOP, PATTERN_OFF, SIGNAL_PATTERN_MAX_INDEX);
                        break;

                    case ERROR_TYPE_14:
                        /* 停止 / パターン不足 */
                        RunSignalPatternError(ERROR_SIGNAL_STOP, PATTERN_INCOMPLETE, SIGNAL_PATTERN_ERROR_INDEX);
                        break;
                    }

                    printf("error_type: %d\n", num);
                }
                else
                {
                    printf("'%s' は入力範囲外です。%u～%uまでの数字を入力してください。\n", argv[2], ERROR_TYPE_1, ERROR_TYPE_14);
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
            printf("'%s' は入力範囲外です。%u桁以上%u桁以下で入力してください。\n", argv[2], ARG2_ERROR_TYPE_DIGITS_MIN, ARG2_ERROR_TYPE_DIGITS_MAX);
            digitalWrite(LED_PIN, HIGH);
        }
    }
    else if (strcmp(argv[1], "stop") == 0)
    {
        /* "stop"入力時処理 */

        /* 停止合図 */
        StopSign();
        if (error_status != ERROR)
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

void RaiseTimingError(char trigger_time_ms, char error_timing, char signal_state)
{
    int i = 0;

    start_on_count = 0;
    printf("開始合図です。\n");

    if (error_timing == ERROR_SIGNAL_START)
    {
        /* 開始合図 */
        for (i = 0; i < trigger_time_ms; i++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10); /* 10msウェイト */
        }

        error_status = NOMAL;
        if (signal_state == ERROR_SIGNAL_ON)
        {
            /* ON時の処理 */
            start_on_count = i;
            StartOnErrStatusCheck(); /* errorステータスをチェックする */
        }
        else if (signal_state == ERROR_SIGNAL_OFF)
        {
            /* OFF時の処理 */
            start_off_count = i;
            StartOffErrStatusCheck();
        }

        if (error_status == ERROR)
        {
            printf("開始合図エラーです。\n");
        }
    }
    else if (error_timing == ERROR_SIGNAL_STOP)
    {
        /* 停止合図 */
        for (i = 0; i < trigger_time_ms; i++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10); /* 10msウェイト */
        }

        error_status = NOMAL;
        if (signal_state == ERROR_SIGNAL_ON)
        {
            end_on_count = i;
            StopOnErrStatusCheck();
        }
        else if (signal_state == ERROR_SIGNAL_OFF)
        {
            end_off_count = i;
            StopOffErrStatusCheck();
        }
        if (error_status == ERROR)
        {
            printf("停止合図エラーです。\n");
        }
    }
}

void StartSign(void)
{
    int j;
    int i;
    for (j = 0; j <= 1; j++)
    {
        start_on_count = 0;
        printf("開始合図です。\n");

        /* 短すぎるON期間 */
        for (i = 0; i < START_ON_SHORT_MS; i++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10); /* 10msウェイト */
            start_on_count++;
        }
        StartOnErrStatusCheck();
        if (error_status == ERROR)
        {
            printf("開始合図エラーです。\n");
            break;
        }

        /* 短すぎるOFF期間 */
        start_off_count = 0;
        printf("開始合図です。\n");

        for (i = 0; i < START_OFF_SHORT_MS; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(10); /* 10msウェイト */
            start_off_count++;
        }
        StartOnErrStatusCheck();
        if (error_status == ERROR)
        {
            printf("開始合図エラーです。\n");
            break;
        }
    }
}

void StartOnErrStatusCheck(void)
{
    error_status = NOMAL;
    if ((start_on_count < START_SHORT_ON) || (start_on_count > START_LONG_ON))
    {
        error_status = ERROR;
    }
}
void StartOffErrStatusCheck(void)
{
    error_status = NOMAL;
    if ((start_off_count < START_SHORT_OFF) || (start_off_count > START_LONG_OFF))
    {
        error_status = ERROR;
    }
}

void StopSign(void)
{
    int j;
    int i;
    for (j = 0; j <= 1; j++)
    {
        printf("表示停止合図です。\n");

        /* 短すぎるON期間 */
        end_on_count = 0;
        for (i = 0; i < STOP_ON_SHORT_MS; i++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10); /* 10msウェイト */
            end_on_count++;
        }
        StopOnErrStatusCheck();
        if (error_status == ERROR)
        {
            printf("表示停止合図エラーです。\n");
            break;
        }

        /* 長すぎるON期間 */
        end_on_count = 0;
        for (i = 0; i < STOP_ON_LONG_MS; i++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10); /* 10msウェイト */
            end_on_count++;
        }
        StopOnErrStatusCheck();
        if (error_status == ERROR)
        {
            printf("表示停止合図エラーです。\n");
            break;
        }

        printf("表示停止合図です。\n");

        /* 短すぎるOFF期間 */
        end_off_count = 0;
        for (i = 0; i < STOP_OFF_SHORT_MS; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(10); /* 10msウェイト */
            end_off_count++;
        }
        StopOffErrStatusCheck();
        if (error_status == ERROR)
        {
            printf("表示停止合図エラーです。\n");
            break;
        }

        /* 長すぎるOFF期間 */
        end_off_count = 0;
        for (i = 0; i < STOP_OFF_LONG_MS; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(10); /* 10msウェイト */
            end_off_count++;
        }
        StopOffErrStatusCheck();
        if (error_status == ERROR)
        {
            printf("表示停止合図エラーです。\n");
            break;
        }
    }
}

void StopOnErrStatusCheck(void)
{
    error_status = NOMAL;
    if ((end_on_count < END_SHORT_ON) || (end_on_count > END_LONG_ON))
    {
        error_status = ERROR;
    }
}
void StopOffErrStatusCheck(void)
{
    error_status = NOMAL;
    if ((end_off_count < END_SHORT_OFF) || (end_off_count > END_LONG_OFF))
    {
        error_status = ERROR;
    }
}

void RunSignalPatternError(char error_timing, char error_pattern, char count)
{
    const int signal_pattern_on[4] = {SIGNAL_ON, SIGNAL_ON, SIGNAL_OFF, SIGNAL_ON};    /* 2回連続ONを含むパターン */
    const int signal_pattern_off[4] = {SIGNAL_ON, SIGNAL_OFF, SIGNAL_OFF, SIGNAL_OFF}; /* 2回連続OFFを含むパターン */
    const int signal_patterns_incomplete[3] = {SIGNAL_ON, SIGNAL_OFF, SIGNAL_ON};      /* パターン不足 */
    int prev_signal_state = SIGNAL_NONE;                                               /* 前回値保存 */
    int curr_signal_state = SIGNAL_NONE;                                               /* 今回値保存 */
    int i = 0;

    /* count数が不足していないか確認 */
    if (count == (COUNT_MAX_INDEX))
    {

        for (i = 0; i < count; i++)
        {
            if (error_pattern == PATTERN_ON)
            {
                /* ONが2回連続パターン */
                curr_signal_state = signal_pattern_on[i];
            }
            else if (error_pattern == PATTERN_OFF)
            {
                /* OFFが2回連続パターン */
                curr_signal_state = signal_pattern_off[i];
            }

            if (error_timing == ERROR_SIGNAL_START)
            {
                /* スタート時*/
                if (curr_signal_state == SIGNAL_ON)
                {
                    digitalWrite(LED_PIN, LOW);
                    delay(100); /* ON期間100ms送信 */
                }
                else
                {
                    digitalWrite(LED_PIN, HIGH);
                    delay(400); /* OFF期間100ms送信 */
                }
            }
            else if (error_timing == ERROR_SIGNAL_STOP)
            {
                if (curr_signal_state == SIGNAL_ON)
                {
                    digitalWrite(LED_PIN, LOW);
                    delay(150); /* ON期間150ms送信 */
                }
                else
                {
                    digitalWrite(LED_PIN, HIGH);
                    delay(350); /* OFF期間350ms送信 */
                }
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

void SendData(char *arg_str, int bit_num)
{
    bool is_led_light;
    int i;

    printf("データ送信します。\n");
    for (i = MAX_DATA_NUM; i >= 0; i--)
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
        delay(100); /* 100ms送信 */
    }
}
