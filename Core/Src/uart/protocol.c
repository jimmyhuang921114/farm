#include "main.h"
#include "protocol.h"
#include "cJSON.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// 從 JSON 讀取有限且落在 float 表示範圍內的數值。
static bool ReadFloat(
    const cJSON *object,
    const char *name,
    float *output
)
{
    const cJSON *item =
        cJSON_GetObjectItemCaseSensitive(object, name);

    if (!cJSON_IsNumber(item))
        return false;

    double value = item->valuedouble;

    if (!isfinite(value) ||
        value > FLT_MAX ||
        value < -FLT_MAX)
    {
        return false;
    }

    *output = (float)value;
    return true;
}


// 解析一行雲台指令或車體 JSON，成功時才更新輸出。
bool Protocol_Parse(
    const char *line,
    RobotCommand *command
)
{
    if (line == NULL || command == NULL)
        return false;

#if GIMBAL_ENABLED
    // 雲台沿用純文字指令，其他訊息繼續走既有 JSON 解析。
    if (strcmp(line, "ZERO") == 0)
    {
        *command = (RobotCommand){.type = COMMAND_GIMBAL_ZERO};
        return true;
    }
    if (strcmp(line, "YAW?") == 0)
    {
        *command = (RobotCommand){.type = COMMAND_GIMBAL_YAW_QUERY};
        return true;
    }

    // 整行只包含角度與結尾空白時，才視為相對角度指令。
    char *end = NULL;
    float delta = strtof(line, &end);
    if (end != line)
    {
        while (*end == ' ' || *end == '\t')
            ++end;

        if (*end == '\0')
        {
            *command = (RobotCommand){
                .type = COMMAND_GIMBAL_RELATIVE,
                .gimbal_delta_deg = delta
            };
            return true;
        }
    }
#endif

    // 要求整行都是有效 JSON，拒絕尾端多餘資料
    cJSON *root = cJSON_ParseWithOpts(line, NULL, 1);

    if (root == NULL)
        return false;

    RobotCommand result = {0};
    bool valid = false;

    const cJSON *cmd =
        cJSON_GetObjectItemCaseSensitive(root, "cmd");

    const cJSON *data =
        cJSON_GetObjectItemCaseSensitive(root, "data");

    if (!cJSON_IsObject(root) ||
        !cJSON_IsString(cmd) ||
        !cJSON_IsObject(data))
    {
        goto cleanup;
    }

    if (strcmp(cmd->valuestring, "person_position") == 0)
    {
        result.type = COMMAND_PERSON_POSITION;

        valid =
            ReadFloat(data, "x", &result.person.x) &&
            ReadFloat(data, "y", &result.person.y) &&
            ReadFloat(data, "z", &result.person.z);
    }
    else if (strcmp(cmd->valuestring, "person_lost") == 0)
    {
        result.type = COMMAND_PERSON_LOST;
        valid = true;
    }
    else if (strcmp(cmd->valuestring, "stop") == 0)
    {
        result.type = COMMAND_STOP;
        valid = true;
    }

cleanup:
    cJSON_Delete(root);

    // 整包有效才更新呼叫端資料
    if (valid)
        *command = result;

    return valid;
}


// 將車體前進速度與偏航角速度編成一行 JSON。
size_t Protocol_EncodeCmdVel(
    char *buffer,
    size_t capacity,
    float linear_mps,
    float angular_radps
)
{
    if (buffer == NULL || capacity == 0 ||
        !isfinite(linear_mps) || !isfinite(angular_radps))
    {
        return 0;
    }

    int length = snprintf(buffer, capacity,
                          "{\"cmd\":\"cmd_vel\",\"data\":{\"v\":%.4f,\"w\":%.4f}}\n",
                          (double)linear_mps, (double)angular_radps);

    return (length > 0 && (size_t)length < capacity) ? (size_t)length : 0;
}

// 將左右車輪目標速度編成一行 JSON。
size_t Protocol_EncodeWheelTarget(
    char *buffer,
    size_t capacity,
    float left_mps,
    float right_mps
)
{
    if (buffer == NULL || capacity == 0 ||
        !isfinite(left_mps) || !isfinite(right_mps))
    {
        return 0;
    }

    int length = snprintf(
        buffer,
        capacity,
        "{\"cmd\":\"wheel_target\","
        "\"data\":{\"left\":%.3f,\"right\":%.3f}}\n",
        (double)left_mps,
        (double)right_mps
    );

    if (length < 0 || (size_t)length >= capacity)
        return 0;

    return (size_t)length;
}
