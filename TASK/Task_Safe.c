#include "Task_Safe.h"
#include "stdlib.h"
#include "string.h"
#include "tim.h"
#include "bsp_buzzer.h"
float note_list[] = 
	{
		1000000,
		NOTE_C4   ,
		NOTE_CS4  ,
		NOTE_D4   ,
		NOTE_DS4  ,
		NOTE_E4   ,
		NOTE_F4   ,
		NOTE_FS4  ,
		NOTE_G4   ,
		NOTE_GS4  ,
		NOTE_A4   ,
		NOTE_AS4  ,
		NOTE_B4   ,
		NOTE_C5   ,
		NOTE_CS5  ,
		NOTE_D5   ,
		NOTE_DS5  ,
		NOTE_E5   ,
		NOTE_F5   ,
		NOTE_FS5  ,
		NOTE_G5   ,
		NOTE_GS5  ,
		NOTE_A5   ,
		NOTE_AS5  ,
		NOTE_B5   ,
		NOTE_C6   ,
		NOTE_CS6  ,
		NOTE_D6   ,
		NOTE_DS6  ,
		NOTE_E6   ,
		NOTE_F6   ,
		NOTE_FS6  ,
		NOTE_G6   ,
		NOTE_GS6  ,
		NOTE_A6   ,
		NOTE_AS6  ,
		-1.0f
	};
	
unsigned int Music[]={
//	17,16,15,16,13,16,12,16,10,16,8,16,10,16,12,16,20,16,20,16,17,16,17,16,13,16,13,16,13,16,15,16,20,16,20,8,18,8,25,16,24,8,20,8,22,16,20,8,17,8,18,16,15,8,18,8,25,4,24,4,25,4,17,4,20,8,24,8,25,8,29,8,32,4,29,4,32,4,34,4,30,4,29,4,27,4,30,4,29,4,27,4,25,4,24,4,22,4,18,4,25,8,24,4,20,4,25,4,24,4,25,4,24,4,25,4,17,4,20,8,24,8,25,8,29,4,25,4,32,4,29,4,32,4,34,4,30,4,29,4,27,4,30,4,29,4,27,4,25,4,24,4,22,4,20,4,18,8,25,6,20,2,24,4,27,2,32,2,29,6,20,2,29,2,27,2,25,2,27,2,27,6,29,2,30,2,29,2,27,2,29,2,25,6,25,2,25,4,24,2,25,2,24,4,20,2,17,2,17,4,20,4,22,8,24,4,25,4,20,8,17,4,20,4,18,8,18,2,22,2,25,4,25,6,24,2,24,2,25,2,27,2,32,2,	//音符,时值,
0,8,0,8,0,8,0,6,8,4,18,4,18,4,18,4,18,4,18,4,17,2,17,2,17,8,18,4,18,4,18,4,20,4,17,4,15,4,13,4,12,4,12,4,13,4,13,8,13,8,0,8,0,8,0,8,0,6,8,4,20,4,20,4,20,4,20,4,20,4,18,2,17,2,17,8,20,4,20,4,20,4,20,4,20,4,18,2,18,2,18,4,17,4,17,4,15,4,15,8,15,8,0,8,0,8,0,8,0,6,8,4,18,4,18,4,18,4,18,4,18,4,17,2,17,2,17,8,18,4,18,4,18,4,20,4,17,4,15,4,13,4,12,4,12,2,15,2,13,4,13,8,13,8,0,8,0,12,10,4,13,4,20,4,20,4,22,4,13,4,13,4,13,8,20,4,20,4,20,4,20,2,20,2,20,4,18,4,18,4,17,4,17,4,15,8,15,8,15,8,0,8,0,4,13,4,25,4,24,4,25,4,25,4,25,4,20,4,20,4,20,4,20,8,18,4,18,4,18,4,17,4,0,4,13,4,25,4,24,4,25,4,25,4,25,4,20,4,20,4,20,4,20,8,18,4,18,4,18,4,17,4,0,4,25,4,25,4,24,4,25,4,24,4,25,4,20,4,20,4,20,4,20,8,18,4,18,4,18,4,17,4,0,4,13,4,25,4,24,4,25,4,25,4,25,4,20,4,20,4,20,4,20,8,18,4,18,4,18,4,17,4,0,4,25,4,25,4,24,4,25,4,24,4,24,4,22,4,22,2,0,2,25,4,24,4,22,4,24,4,22,4,22,4,20,2,0,2,25,4,22,4,20,4,22,4,20,4,20,4,18,4,18,4,17,4,18,4,20,4,22,4,22,4,22,4,20,4,20,2,0,2,13,4,25,4,24,4,25,4,25,4,25,4,20,4,20,4,20,4,20,8,18,4,18,4,18,4,17,4,0,4,13,4,25,4,24,4,25,4,25,4,25,4,20,4,20,4,20,4,20,8,27,4,27,4,27,4,25,4,0,4,13,4,25,4,24,4,25,4,24,4,24,4,22,4,22,2,0,2,13,4,24,4,22,4,24,4,22,4,22,4,20,4,20,2,0,2,13,4,22,4,20,4,22,4,20,4,20,4,18,4,18,4,17,4,18,4,20,4,22,4,22,4,22,4,20,4,20,2,0,2,18,4,17,8,24,4,15,4,15,4,13,4,13,8,13,8,
//20,6,17,2,25,2,25,2,20,6,17,2,22,6,20,6,27,24,25,6,20,2,29,2,29,2,25,6,20,2,29,6,25,6,32,24,20,4,17,2,25,2,25,2,20,6,17,2,22,4,20,6,27,24,25,6,20,2,29,2,29,2,25,6,20,2,29,6,25,6,32,24,8,2,10,2,13,4,13,4,8,2,10,2,13,4,13,4,8,2,10,2,13,4,13,4,0,12,13,2,15,2,13,4,17,2,17,4,13,4,
	//8,2,10,2,13,4,13,4,8,2,22,2,13,4,13,4,8,2,10,2,13,4,13,4,0,12,13,2,15,2,13,4,17,2,17,4,13,4,17,12,15,2,18,2,18,2,17,2,15,4,12,2,13,2,13,24,
//17,2,17,4,17,2,0,2,13,2,17,4,20,4,0,4,8,4,0,4,13,4,0,2,8,2,0,4,5,4,0,2,10,4,12,2,0,2,12,2,10,4,8,4,17,4,20,4,22,4,18,2,20,2,0,2,17,4,13,2,15,2,12,4,0,2,//	H3,1,
//	H4,1,
//	H5,2,
//	H3,1,
//	H4,1,
//	H5,1,
//	M7,1,
//	M6,1,
//	M7,1,
//	H1,1,
//	H2,1,
//	H3,1,
//	H4,1,
//	
//	//2
//	H3,2,
//	H1,1,
//	H2,1,
//	H3,2,
//	M3,1,
//	M4,1,
//	M5,1,
//	M6,1,
//	M5,1,
//	M4,1,
//	M5,1,
//	H1,1,
//	M7,1,
//	H1,1,
//	
//	//3
//	M6,2,
//	H1,1,
//	M7,1,
//	M6,2,
//	M5,1,
//	M4,1,
//	M5,1,
//	M4,1,
//	M3,1,
//	M4,1,
//	M5,1,
//	M6,1,
//	M7,1,
//	H1,1,
//	
//	//4
//	M6,2,
//	H1,1,
//	M7,1,
//	H1,2,
//	M7,1,
//	H1,1,
//	M7,1,
//	M6,1,
//	M7,1,
//	H1,1,
//	H2,1,
//	H3,1,
//	H4,1,
//	H5,1,
//	
//	//5
//	H5,2,
//	H3,1,
//	H4,1,
//	H5,2,
//	H3,1,
//	H4,1,
//	H5,1,
//	M7,1,
//	M6,1,
//	M7,1,
//	H1,1,
//	H2,1,
//	H3,1,
//	H4,1,
//	
//	//6
//	H3,2,
//	H1,1,
//	H2,1,
//	H3,2,
//	M3,1,
//	M4,1,
//	M5,1,
//	M6,1,
//	M5,1,
//	M4,1,
//	M5,1,
//	H1,1,
//	M7,1,
//	H1,1,
	0xFF	//终止标志
};

uint32_t *stack_base_p = 0x00000000;
uint32_t stack_base;
uint32_t heap_free;
uint32_t stack_now;
uint32_t stack_free = 0;

extern QueueHandle_t safe_task_queue;

static int my_heapprt(void *param, char const *format, ...)
{
    if ((*(int *)param) != 0)
        return 0;
    va_list argp;
    va_start(argp, format);
    int data = va_arg(argp, int);
    va_end(argp);
    *(int *)param = data;
    return -1;
}

int heap_free_size()
{
    int size = 0;
    __heapstats((__heapprt)my_heapprt, &size);
    return size;
}

/**
 * @brief      安全任务
 * @param[in]  *pvParameters
 * @retval     none
 * @attention
 */
void Safe_Task(void const *argument)
{
    stack_base = *stack_base_p;
//    safe_message_queue rec_message;
//   	static TickType_t currentTime;
//    safe_task_queue = xQueueCreate(10, sizeof(safe_message_queue));

//   	currentTime = xTaskGetTickCount();
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    /*------------can1----------*/
//    Protect_object_create(300, "p轴");

    /*------------can2----------*/
//    Protect_object_create(100, (uint8_t*)"referee");
//    Protect_object_create(100, (uint8_t*)"rc");
//    Protect_object_create(100, "y轴");

    while (1)
    {
//				uint8_t error_object_flag = 0;
			
        stack_now = __get_MSP();
        stack_free = 0x2000 - (stack_base - stack_now);
        heap_free = heap_free_size();

//        Lost_check_all();
//        while (xQueueReceive(safe_task_queue, &rec_message, 0) == pdTRUE)
//        {
//            if (rec_message.error == safe_ERROR)
//            {
//                error_object_flag = 1;
//            }
//        }
//        if (error_object_flag)
//        {
					static TickType_t last_note_systick = 0;
					static int music_num = 0;
					if(xTaskGetTickCount() - last_note_systick >Music[music_num+1]*73)
						buzzer_off();
					if(xTaskGetTickCount() - last_note_systick >Music[music_num+1]*80)
					{	
						last_note_systick = xTaskGetTickCount();
						music_num+=2;		
						if(Music[music_num] == 0xff)
							music_num = 0;
						buzzer_test(83, 1000000 / note_list[Music[music_num]]);
					}
					
//					static int note_list_num = 0;
//          buzzer_test(0, 84000000 / (int)note_list[note_list_num]);
//					if(xTaskGetTickCount() - last_note_systick >100)
//					{
//						last_note_systick = xTaskGetTickCount();
//						note_list_num++;
//						if(note_list[note_list_num] == -1.0f)
//						note_list_num = 0;
//					}
//        }
//        else
//        {
//            buzzer_off();
//        }

        vTaskDelay(1);
    }
}
