/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-02-28     Bernard      the first version
 */

#include <board.h>
#include <rtthread.h>

#include <components.h>


static void thread_entry(void* parameter)
{
    sys_led_init();

    rt_platform_init();

#ifdef RT_USING_DFS
   dfs_init();
#ifdef RT_USING_DFS_ELMFAT
	  elm_init();
        if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
        {
            rt_kprintf("SDCard File System initialized!\n");
        }
		else
		{
			rt_kprintf("SDCard File System initialzation failed!\n");
		}
#endif
#endif


  mpu6050_thread_init();
  mavlink_thread_init();
#ifdef RT_USING_FINSH
		finsh_system_init();
		finsh_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

}

int rt_application_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
                           thread_entry, RT_NULL,
                           2048, RT_THREAD_PRIORITY_MAX/3, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}

/*@}*/
