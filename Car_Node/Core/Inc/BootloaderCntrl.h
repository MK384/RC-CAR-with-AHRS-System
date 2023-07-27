/*
 * BootloaderCntrl.h
 *
 *  Created on: Jul 25, 2023
 *      Author: moham
 */

#ifndef INC_BOOTLOADERCNTRL_H_
#define INC_BOOTLOADERCNTRL_H_



// Function to jump to the system memory bootloader
void jumpToBootloader(void) {

    // 1. Disable all interrupts
    __disable_irq();

    // 2. Deinitialize peripherals
    SysTick->CTRL = 0;
    HAL_RCC_DeInit();

    /* Clear Interrupt Enable Register & Interrupt Pending Register */
	for (int i=0;i<5;i++)
	{
		NVIC->ICER[i]=0xFFFFFFFF;
		NVIC->ICPR[i]=0xFFFFFFFF;
	}

    HAL_DeInit();


    /* Re-enable all interrupts */
    	__enable_irq();

    // Function pointer type for the system memory (bootloader) function
    typedef void (*BootloaderFunction)(void);

    // 4. Perform the jump to the system memory
    // Function pointer to the system memory address
    BootloaderFunction SysMemBootJump  = (BootloaderFunction)(*(uint32_t *)(0x1FFF0004));

    /* Set the main stack pointer to the boot loader stack */
    	__set_MSP(*(uint32_t *)0x1FFF0000);

    // Call the system memory function
    SysMemBootJump();

    /* Jump is done successfully */
    	while (1)
    	{
    	    // Note: The code execution will not reach here after the jump.
    	}

}


#endif /* INC_BOOTLOADERCNTRL_H_ */
