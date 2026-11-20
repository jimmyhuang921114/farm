#include "protocol.h"
#include "cJSON.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


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


bool Protocol_Parse(
    const char *line,
    RobotCommand *command
)
{
    if (line == NULL || command == NULL)
        return false;

    /* 要求整行都是有效 JSON，拒絕尾端多餘資料 */
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

        /* 依照目前相機座標約定，人物深度必須大於 0 */
        if (valid && result.person.z <= 0.0f)
            valid = false;
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

    /* 整包有效才更新呼叫端資料 */
    if (valid)
        *command = result;

    return valid;
}


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
