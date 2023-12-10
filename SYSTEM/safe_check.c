#include "safe_check.h"

Lost_Protect head_list = {.next_protect_obj = NULL};
QueueHandle_t safe_task_queue = {0};

static Lost_Protect *find_object_by_name(uint8_t *name)
{
    Lost_Protect *temp = &head_list;
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t namelen = (uint16_t)strlen((const char *)name);
    while (temp->next_protect_obj != NULL)
    {
        temp = temp->next_protect_obj;
        if (i++ >= Max_protect_objects)
        {
            // out of maximum
            return NULL;
        }
        if (namelen == strlen((const char *)temp->queue.object_name))
        {
            const uint8_t *scanpoint = name;
            j = 0;
            while (*scanpoint == temp->queue.object_name[j])
            {
                if (j == namelen)
                {
                    return temp;
                }
                j++;
                scanpoint++;
            }
        }
    }
    return NULL;
}

Lost_Protect *Protect_object_create(uint32_t MAX_lost_time, uint8_t *name)
{
    Lost_Protect *temp = &head_list;
    Lost_Protect *Lost_Protect_create_p;
    uint16_t i = 0;
    while (temp->next_protect_obj != NULL)
    {
        if (i++ >= Max_protect_objects)
        {
            // out of maximum
            return NULL;
        }
        temp = temp->next_protect_obj;
    }

    MAX_lost_time *= portTICK_PERIOD_MS;
    Lost_Protect_create_p = (Lost_Protect *)malloc(sizeof(Lost_Protect));
    memset(Lost_Protect_create_p, 0, sizeof(Lost_Protect));
    Lost_Protect_create_p->Max_Lost_Time = MAX_lost_time;
    Lost_Protect_create_p->next_protect_obj = NULL;
    i = 0;
    while (*name != '\0')
    {
        Lost_Protect_create_p->queue.object_name[i] = *name;
        name++;
        i++;
        if (i >= Max_protect_objects)
        {
            // out of maximum
            return NULL;
        }
    }

    temp->next_protect_obj = Lost_Protect_create_p;
    return Lost_Protect_create_p;
}

void Protect_object_delete(Lost_Protect *object)
{
    Lost_Protect *temp = &head_list;
    uint16_t i = 0;
    while (temp->next_protect_obj != object)
    {
        if (i++ >= Max_protect_objects)
        {
            // out of maximum
            return;
        }
        temp = temp->next_protect_obj;
    }
    temp->next_protect_obj = object->next_protect_obj;
    free(object);
    *(int *)object = NULL;
}
void Protect_object_delete_by_name(uint8_t *name)
{
    Protect_object_delete(find_object_by_name(name));
}
void Lost_check_all(void)
{
    Lost_Protect *temp = &head_list;
    uint16_t i = 0;
    do
    {
        temp = temp->next_protect_obj;
        if (i++ >= Max_protect_objects)
        {
            // out of maximum
            return;
        }
        Lost_check(temp);
    } while (temp->next_protect_obj != NULL);
}
safe_error_type Lost_check(Lost_Protect *object)
{
    TickType_t currentTime;
    currentTime = xTaskGetTickCount();
    object->Lost_Time = currentTime - object->Last_Systime;
    if (object->Lost_Time > object->Max_Lost_Time)
    {
        object->queue.error = safe_ERROR;
        xQueueSend(safe_task_queue, &object->queue, 0);
        return safe_ERROR;
    }
    return safe_OK;
}
void Lost_time_fresh(Lost_Protect *object)
{
    TickType_t currentTime;
    if (object->queue.error != safe_OK)
    {
        object->queue.error = safe_OK;
        xQueueSendFromISR(safe_task_queue, &object->queue, 0);
    }

    currentTime = xTaskGetTickCount();
    object->Last_Systime = currentTime;
}
void Lost_time_fresh_by_name(uint8_t *name)
{
    Lost_time_fresh(find_object_by_name(name));
}
