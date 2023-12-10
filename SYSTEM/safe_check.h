
#ifndef __SAFE_CHECK_H // 如果未定义
#define __SAFE_CHECK_H // 那么定义
#include "main.h"
#include "stdlib.h"
#include "string.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#define Max_protect_objects 100
typedef enum
{
  safe_OK = 0,
  safe_WARNING = 1,
  safe_ERROR = 2,
} safe_error_type;

typedef struct
{
  safe_error_type error;
  uint8_t object_name[32];
} safe_message_queue;

typedef struct Lost_Protect
{
  uint32_t Max_Lost_Time;
  uint32_t Lost_Time;
  TickType_t Last_Systime;
  safe_message_queue queue;
  struct Lost_Protect *next_protect_obj;
} Lost_Protect;

Lost_Protect *Protect_object_create(uint32_t MAX_lost_time, uint8_t *name);

void Protect_object_delete(Lost_Protect *object);
void Protect_object_delete_by_name(uint8_t *name);
void Lost_time_fresh(Lost_Protect *object);
void Lost_time_fresh_by_name(uint8_t *name);

safe_error_type Lost_check(Lost_Protect *object);

void Lost_check_all(void);

#endif
