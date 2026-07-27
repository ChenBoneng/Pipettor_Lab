//
// 创建时间：26-7-25
//

#include "MyTask.h"
#include "cmsis_os2.h"
#include "main.h"
#include "Keyboard.h"
#include "Communication.h"



void KeyboardTask(void *argument)
{
    (void)argument;

    Keypad_Init();
    Communication_Init();

    for(;;)
    {
        /* 每 1ms 扫描一次，连续 4 次稳定后确认按键。 */
        Keypad_Process();
        osDelay(1);
    }

}
