#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>

#define LED_PIN (4U)
#define LED_MASK (0x00001U)
#define NOMAL (0U)      /* error_statusが正常 */
#define ERROR (1U)      /* error_statusがエラー */
#define EXIT_ERROR (1U) /* 異常終了 */

#define START_CMD_ARGS_NUM (3U)           /* スタートコマンドを実行するときに必要な引数の数 */
#define ERROR_TYPE_CMD_ARGS_NUM_SEND (4U) /* 送信中エラー発生時に必要な引数の数 */
#define ERROR_TYPE_CMD_ARGS_NUM (3U)      /* 送信中エラー以外のエラーコマンドを実行するときに必要な引数の数 */
#define STOP_CMD_ARGS_NUM (2U)            /* ストップコマンドを実行するときに必要な引数の数 */
#define CMD_ARGS_NUM_MIN (1U)             /* 実行するときに必要な引数の数 */
#define ERROR_TYPE_CMD_ARGS_NUM_MIN (2U)  /* エラータイプを実行するときに必要な引数の数 */

#define COUNT_MAX_INDEX (4U - 1U) /* 最大値（4回だが、0~3のため-1している） */
#define MAX_DATA_NUM (20 - 1)     /* 最大値（20回だが、0~19のため-1している） */

#define SIGNAL_PATTERN_MAX_INDEX (4U - 1U)   /* 最大値（4回だが、0~3のため-1している） */
#define SIGNAL_PATTERN_ERROR_INDEX (3U - 1U) /* 最大値（3回だが、0~2のため-1している） */

/* "start"入力時に使用 */
#define ARG2_START_EXPECT_DIGITS (6U)                         /* 正常（許容）桁数 */
#define ARG2_START_OVER_DIGITS (ARG2_START_EXPECT_DIGITS + 1) /* 許容数より1多い */
#define ARG2_START_EXPECT_MAX_VALUE (1048575U)                /* 最大値（20bitの最大値：0xFFFFF = 1048575） */

/* "error_type"入力時に使用 */
#define ARG2_ERROR_TYPE_EXPECT_DIGITS_SEND (6U) /* 正常（許容）桁数 */
#define ARG2_ERROR_TYPE_DIGITS_MIN (1U)         /* 最小1桁 */
#define ARG2_ERROR_TYPE_DIGITS_MAX (2U)         /* 最大2桁 */
/* スタート時のエラー(データを10ms間隔でX回ずつ送信) */
#define START_ON_MS (10U)                      /* 基準値 */
#define START_ON_SHORT_MS (START_ON_MS - 2U)   /* 基準値から20ms短い値 */
#define START_ON_LONG_MS (START_ON_MS + 2U)    /* 基準値から20ms長い値 */
#define START_OFF_MS (40U)                     /* 基準値 */
#define START_OFF_SHORT_MS (START_OFF_MS - 2U) /* 基準値から20ms短い値 */
#define START_OFF_LONG_MS (START_OFF_MS + 2U)  /* 基準値から20ms長い値 */
/* ストップ時のエラー(データを10ms間隔でX回ずつ送信) */
#define STOP_ON_MS (15U)                     /* 基準値 */
#define STOP_ON_SHORT_MS (STOP_ON_MS - 2U)   /* 基準値から20ms短い値 */
#define STOP_ON_LONG_MS (STOP_ON_MS + 2U)    /* 基準値から20ms長い値 */
#define STOP_OFF_MS (35U)                    /* 基準値 */
#define STOP_OFF_SHORT_MS (STOP_OFF_MS - 2U) /* 基準値から20ms短い値 */
#define STOP_OFF_LONG_MS (STOP_OFF_MS + 2U)  /* 基準値から20ms長い値 */

/* 信号開始合図のエラー値(データを10ms間隔でX回ずつ送信) */
#define START_SHORT_ON (9U)
#define START_LONG_ON (11U)
#define START_SHORT_OFF (39U)
#define START_LONG_OFF (41U)

/* 表示停止合図のエラー値(データを10ms間隔でX回ずつ送信) */
#define END_SHORT_ON (14U)
#define END_LONG_ON (16U)
#define END_SHORT_OFF (34U)
#define END_LONG_OFF (36U)

/* データ送信中エラー */
#define SEND_DATA_MAX_COUNT (20U)    /* 送信データの最大数 */
#define SEND_TIMEOUT_LIMIT_MS (200U) /* データ送信時間の上限(ms) */

/* パターン順序エラーパターン */
#define PATTERN_OFF (0U)        /* OFF連続 */
#define PATTERN_ON (1U)         /* ON連続 */
#define PATTERN_INCOMPLETE (2U) /* パターン不足 */

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
    ERROR_TYPE_14,    /* 14 */
    ERROR_TYPE_15,    /* 15 */
} ERROR_TYPE_t;

/* エラー発生タイミング */
typedef enum
{
    ERROR_TIMING_STOP = 0, /* 停止時 */
    ERROR_TIMING_START,    /* 開始時 */
} ERROR_TIMING_t;

/* エラー発生条件（信号状態）*/
typedef enum
{
    ERROR_SIGNAL_OFF = 0, /* OFF時 */
    ERROR_SIGNAL_ON,      /* ON時 */
} ERROR_SIGNAL_t;

/* エラーパターン */
typedef enum
{
    SIGNAL_OFF = 0, /* OFFパターン */
    SIGNAL_ON,      /* ONパターン */
    SIGNAL_NONE,    /* 前回値がない */
} ERROR_PATTERN_t;

/* 正常かエラーか */
typedef enum
{
    CMD_SEND_ERROR = 0, /* 送信中エラー時 */
    CMD_SEND_NORMAL,    /* その他正常時、エラー時 */
} CMD_SEND_STATE_t;

static void StartSign(void);
static void StartOnErrStatusCheck(void);
static void StartOffErrStatusCheck(void);
static void StopSign(void);
static void StopOnErrStatusCheck(void);
static void StopOffErrStatusCheck(void);
static int SendData(char *arg_str, int bit_num);
static void RaiseTimingError(char trigger_time_ms, ERROR_TIMING_t error_timing, ERROR_SIGNAL_t signal_state);
static void RunSignalPatternError(char error_timing, ERROR_PATTERN_t error_pattern, char count);
static int CheckSendDelay(int wait_time_ms);
static int CheckCmdArgsNum(int check_argc, int expected_args);

static int start_on_count;              /* 開始合図点灯データ送信カウント */
static int start_off_count;             /* 開始合図消灯データ送信カウント */
static int end_on_count;                /* 停止合図点灯データ送信カウント */
static int end_off_count;               /* 停止合図消灯データ送信カウント */
static int error_status;                /* エラー状態 */
static CMD_SEND_STATE_t cmd_send_state; /* 送信中エラー確認用 */

int main(int argc, char *argv[])
{
    bool is_all_digit = TRUE;    /* 数字のみのデータか確認用 */
    int num = 0;                 /* 整数変換後データ格納用 */
    int error_type_send_num = 0; /* 送信中エラー時の整数6桁変換後データ格納用 */
    int i;
    error_status = 0;
    cmd_send_state = CMD_SEND_NORMAL;

    wiringPiSetupGpio();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    /* 引数最低限の入力確認 */
    if (argc <= CMD_ARGS_NUM_MIN)
    {
        printf("引数エラーです。引数が少ないです。\n");
        return EXIT_ERROR;
    }

    if (strcmp(argv[1], "start") == 0)
    {
        /* "start"入力時処理 */

        /* 引数の数が正しいか確認 */
        if (CheckCmdArgsNum(argc, START_CMD_ARGS_NUM) == EXIT_ERROR)
        {
            return EXIT_ERROR;
        }

        /* 桁数確認 */
        if (strlen(argv[2]) == ARG2_START_EXPECT_DIGITS)
        {
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
                cmd_send_state = CMD_SEND_NORMAL;

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
                printf("'%s' は数字以外の文字を含んでいます。\n", argv[2]);
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

        /* 引数最低限の入力確認 */
        if (argc <= ERROR_TYPE_CMD_ARGS_NUM_MIN)
        {
            printf("引数エラーです。引数が少ないです。\n");
            return EXIT_ERROR;
        }

        for (i = 0; argv[2][i] != '\0'; i++)
        {
            /* 文字列が数字か確認 */
            if (!isdigit(argv[2][i]))
            {
                /* 数字以外の文字が含まれていた */
                printf("'%s' は数字以外の文字を含んでいます。\n", argv[2]);
                return EXIT_ERROR;
            }
            else
            {
                /* 文字列を整数に変換 */
                num = atoi(argv[2]);
            }
        }

        if (num == ERROR_TYPE_15)
        {
            /* 引数の数が正しいか確認 */
            if ((CheckCmdArgsNum(argc, ERROR_TYPE_CMD_ARGS_NUM_SEND) == EXIT_ERROR))
            {
                /* データ送信中エラー時 */
                return EXIT_ERROR;
            }
        }
        else if (CheckCmdArgsNum(argc, ERROR_TYPE_CMD_ARGS_NUM) == EXIT_ERROR)
        {
            /* その他エラー時 */
            return EXIT_ERROR;
        }

        /* データ送信中エラー時 */
        if (num == ERROR_TYPE_15)
        {
            /* 桁数確認 */
            if (strlen(argv[3]) == ARG2_ERROR_TYPE_EXPECT_DIGITS_SEND)
            {
                for (i = 0; argv[3][i] != '\0'; i++)
                {
                    /* 文字列が数字か確認 */
                    if (!isdigit(argv[3][i]))
                    {
                        /* 数字以外の文字が含まれていた */
                        printf("'%s' は数字以外の文字を含んでいます。\n", argv[3]);
                        return EXIT_ERROR;
                    }
                    else
                    {
                        /* 文字列を整数に変換 */
                        error_type_send_num = atoi(argv[3]);
                    }
                }
            }
            else
            {
                printf("入力桁数が違います。\n");
                digitalWrite(LED_PIN, HIGH);
                printf("'%s' は終了です。\n", argv[1]);
                return EXIT_ERROR;
            }
        }

        switch (num)
        {
        case ERROR_TYPE_1:
            /* 開始 / 短すぎるON期間 */
            RaiseTimingError(START_ON_SHORT_MS, ERROR_TIMING_START, ERROR_SIGNAL_ON);
            break;

        case ERROR_TYPE_2:
            /* 開始 / 長すぎるON期間 */
            RaiseTimingError(START_ON_LONG_MS, ERROR_TIMING_START, ERROR_SIGNAL_ON);
            break;

        case ERROR_TYPE_3:
            /* 開始 / 短すぎるOFF期間 */
            RaiseTimingError(START_OFF_SHORT_MS, ERROR_TIMING_START, ERROR_SIGNAL_OFF);
            break;

        case ERROR_TYPE_4:
            /* 開始 / 長すぎるON期間 */
            RaiseTimingError(START_OFF_LONG_MS, ERROR_TIMING_START, ERROR_SIGNAL_OFF);
            break;

        case ERROR_TYPE_5:
            /* 開始 / ON連続 */
            RunSignalPatternError(ERROR_TIMING_START, PATTERN_ON, SIGNAL_PATTERN_MAX_INDEX);
            break;

        case ERROR_TYPE_6:
            /* 開始 / OFF連続 */
            RunSignalPatternError(ERROR_TIMING_START, PATTERN_OFF, SIGNAL_PATTERN_MAX_INDEX);
            break;

        case ERROR_TYPE_7:
            /* 開始 / パターン不足 */
            RunSignalPatternError(ERROR_TIMING_START, PATTERN_INCOMPLETE, SIGNAL_PATTERN_ERROR_INDEX);
            break;

        case ERROR_TYPE_8:
            /* 停止 / 短すぎるON期間 */
            RaiseTimingError(STOP_ON_SHORT_MS, ERROR_TIMING_STOP, ERROR_SIGNAL_ON);
            break;

        case ERROR_TYPE_9:
            /* 停止 / 長すぎるON期間 */
            RaiseTimingError(STOP_ON_LONG_MS, ERROR_TIMING_STOP, ERROR_SIGNAL_ON);
            break;

        case ERROR_TYPE_10:
            /* 停止 / 短すぎるOFF期間 */
            RaiseTimingError(STOP_OFF_SHORT_MS, ERROR_TIMING_STOP, ERROR_SIGNAL_OFF);
            break;

        case ERROR_TYPE_11:
            /* 停止 / 長すぎるOFF期間 */
            RaiseTimingError(STOP_OFF_LONG_MS, ERROR_TIMING_START, ERROR_SIGNAL_OFF);
            break;

        case ERROR_TYPE_12:
            /* 停止 / ON連続 */
            RunSignalPatternError(ERROR_TIMING_STOP, PATTERN_ON, SIGNAL_PATTERN_MAX_INDEX);
            break;

        case ERROR_TYPE_13:
            /* 停止 / OFF連続 */
            RunSignalPatternError(ERROR_TIMING_STOP, PATTERN_OFF, SIGNAL_PATTERN_MAX_INDEX);
            break;

        case ERROR_TYPE_14:
            /* 停止 / パターン不足 */
            RunSignalPatternError(ERROR_TIMING_STOP, PATTERN_INCOMPLETE, SIGNAL_PATTERN_ERROR_INDEX);
            break;

        case ERROR_TYPE_15:
            /* データ送信中のエラー */
            cmd_send_state = CMD_SEND_ERROR; /* エラーステートに切り替える */

            /* 開始合図 */
            StartSign();
            if (error_status != ERROR)
            {
                /* エラーがなければデータ送信 */
                SendData(argv[3], error_type_send_num);
                digitalWrite(LED_PIN, HIGH);
            }

            cmd_send_state = CMD_SEND_NORMAL; /* 正常ステートに戻す */

            break;

        default:
            printf("'%s' は入力範囲外です。%u～%uまでの数字を入力してください。\n", argv[2], ERROR_TYPE_1, ERROR_TYPE_15);
            digitalWrite(LED_PIN, HIGH);
            break;
        }

        printf("error_type: %d\n", num);
    }
    else if (strcmp(argv[1], "stop") == 0)
    {
        /* "stop"入力時処理 */

        /* 引数の数が正しいか確認 */
        if ((CheckCmdArgsNum(argc, STOP_CMD_ARGS_NUM) == EXIT_ERROR))
        {
            /* データ送信中エラー時 */
            return EXIT_ERROR;
        }

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

int CheckCmdArgsNum(int check_argc, int expected_args)
{
    /* 引数が正しいか確認 */
    if (check_argc != expected_args)
    {
        printf("引数エラーです。引数の数が違います。\n");
        digitalWrite(LED_PIN, HIGH);
        return EXIT_ERROR;
    }
    return EXIT_SUCCESS;
}

void RaiseTimingError(char trigger_time_ms, ERROR_TIMING_t error_timing, ERROR_SIGNAL_t signal_state)
{
    int i = 0;

    start_on_count = 0;
    error_status = NOMAL;
    printf("開始合図です。\n");

    if (error_timing == ERROR_TIMING_START)
    {
        if (signal_state == ERROR_SIGNAL_ON)
        {
            /* ON時の処理 */
            start_on_count = i;
            StartOnErrStatusCheck(); /* errorステータスをチェックする */
        }
        else
        {
            /* OFF時の処理 */
            start_off_count = i;
            StartOffErrStatusCheck();
        }
    }
    else
    {
        if (signal_state == ERROR_SIGNAL_ON)
        {
            end_on_count = i;
            StopOnErrStatusCheck();
        }
        else
        {
            end_off_count = i;
            StopOffErrStatusCheck();
        }
    }

    if (error_status == ERROR)
    {
        printf("開始合図エラーです。\n");
    }

    if (signal_state == ERROR_SIGNAL_ON)
    {
        for (i = 0; i < trigger_time_ms; i++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10); /* 10ms送信 */
        }
    }
    else
    {
        /* 動作確認用 / OFF期間が正しいか */
        //digitalWrite(LED_PIN, LOW);
        //delay(10); /* 10ms送信 */

        for (i = 0; i < trigger_time_ms; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(10); /* 10ms送信 */
        }
        
        /* 動作確認用 / OFF期間が正しいか */
        //digitalWrite(LED_PIN, LOW);
        //delay(10); /* 10ms送信 */
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

        for (i = 0; i < START_ON_MS; i++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10); /* 10ms送信 */
            start_on_count++;
        }

        StartOnErrStatusCheck();

        if (error_status == ERROR)
        {
            printf("開始合図エラーです。\n");
            break;
        }

        start_off_count = 0;
        printf("開始合図です。\n");

        for (i = 0; i < START_OFF_MS; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(10); /* 10ms送信 */
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

        end_on_count = 0;
        for (i = 0; i < STOP_ON_MS; i++)
        {
            digitalWrite(LED_PIN, LOW);
            delay(10); /* 10ms送信 */
            end_on_count++;
        }
        StopOnErrStatusCheck();
        if (error_status == ERROR)
        {
            printf("表示停止合図エラーです。\n");
            break;
        }

        printf("表示停止合図です。\n");
        end_off_count = 0;

        for (i = 0; i < STOP_OFF_MS; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(10); /* 10ms送信 */
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

void RunSignalPatternError(char error_timing, ERROR_PATTERN_t error_pattern, char count)
{
    const ERROR_PATTERN_t signal_pattern_on[4] = {SIGNAL_ON, SIGNAL_ON, SIGNAL_OFF, SIGNAL_ON};    /* 2回連続ONを含むパターン */
    const ERROR_PATTERN_t signal_pattern_off[4] = {SIGNAL_ON, SIGNAL_OFF, SIGNAL_OFF, SIGNAL_OFF}; /* 2回連続OFFを含むパターン */
    const ERROR_PATTERN_t signal_patterns_incomplete[3] = {SIGNAL_ON, SIGNAL_OFF, SIGNAL_ON};      /* パターン不足 */
    ERROR_PATTERN_t prev_signal_state = SIGNAL_NONE;                                               /* 前回値保存 */
    ERROR_PATTERN_t curr_signal_state = SIGNAL_NONE;                                               /* 今回値保存 */
    int i = 0;

    /* パターン数が不足していないか確認 */
    if (count == (COUNT_MAX_INDEX))
    {

        for (i = 0; i < count; i++)
        {
            if (error_pattern == PATTERN_ON)
            {
                /* ONが2回連続パターン */
                curr_signal_state = signal_pattern_on[i];
            }
            else
            {
                /* OFFが2回連続パターン */
                curr_signal_state = signal_pattern_off[i];
            }

            if (error_timing == ERROR_TIMING_START)
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
                    delay(400); /* OFF期間400ms送信 */
                }
            }
            else
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

int CheckSendDelay(int wait_time_ms)
{
    int i;
    static int wait_time_count = 0; /* 送信にかかった時間をカウント */
    static int send_data_count = 0; /* 現在の送信済データカウント */

    error_status = NOMAL;

    for (i = 0; i < wait_time_ms; i++)
    {
        wait_time_count++;
        delay(10); /* 10ms送信 */
    }

    if (wait_time_count > SEND_TIMEOUT_LIMIT_MS)
    {
        error_status = ERROR;
        printf("データ送信中エラーです。\n");
        return EXIT_ERROR;
    }

    if (send_data_count >= SEND_DATA_MAX_COUNT)
    {
        /* 送信完了でリセット */
        wait_time_count = 0;
        send_data_count = 0;
    }

    send_data_count++;
}

int SendData(char *arg_str, int bit_num)
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

        if (cmd_send_state == CMD_SEND_ERROR)
        {
            CheckSendDelay(11); /* error用に110ms送信用 */
        }
        else
        {
            CheckSendDelay(10); /* 正常用に100ms送信用 */
        }

        if (error_status == ERROR)
        {
            return EXIT_ERROR;
        }
    }
    return EXIT_SUCCESS;
}
