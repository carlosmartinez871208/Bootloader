/*
** main.c file made by Carlos Martinez
** Project bootloader.
*/
#include <stdio.h>
#include <stdbool.h>

#ifndef EXIT_SUCCESS
 #define EXIT_SUCCESS 0u
#endif

/*                              Peripherals                                 */
/* Cortex M4 internal peripherals base address. */
#define CORTEX_M4_INT_PERIPH_ADDRESS (0xE0000000ul)

/* Peripherals base address. */
#define PERIPHERAL_BASE (0x40000000ul)

/* APB2 base address. */
#define APB2_PERIPHERAL_OFFSET (0x10000ul)
#define APB2_PERIPHERAL_BASE (PERIPHERAL_BASE + APB2_PERIPHERAL_OFFSET)

/* AHB1 base address. */
#define AHB1_PERIPHERAL_OFFSET (0x00020000ul)
#define AHB1_PERIPHERAL_BASE   (PERIPHERAL_BASE + AHB1_PERIPHERAL_OFFSET)

/* GPIOA base address. */
#define GPIOA_OFFSET (0x0ul)
#define GPIOA_BASE   (AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)

/* GPIOC base address */
#define GPIOC_OFFSET (0x800ul)
#define GPIOC_BASE   (AHB1_PERIPHERAL_BASE + GPIOC_OFFSET)

/* RCC base address. */
#define RCC_OFFSET (0x3800ul)
#define RCC_BASE   (AHB1_PERIPHERAL_BASE + RCC_OFFSET)

/* USART6 base address. */
#define USART1_OFFSET (0x1000ul)
#define USART1_BASE (APB2_PERIPHERAL_BASE + USART1_OFFSET)

/* Systick base address */
#define SYSTICK_OFFSET (0xE010ul)
#define SYSTICK_BASE   (CORTEX_M4_INT_PERIPH_ADDRESS + SYSTICK_OFFSET)

/* FPU Base address */
#define FPU_OFFSET     (0xED88ul)
#define FPU_BASE       (CORTEX_M4_INT_PERIPH_ADDRESS + FPU_OFFSET)

/*                                Registers                                 */
#define FPU_CPACR_OFFSET (0x0ul)
#define FPU_CPACR (*(volatile unsigned int*)(FPU_BASE + FPU_CPACR_OFFSET))

/* RCC AHB1 enable clock register. */
#define RCC_AHB1ENR_OFFSET (0x30ul)
#define RCC_AHB1ENR (*(volatile unsigned int*)(RCC_BASE + RCC_AHB1ENR_OFFSET))

/* RCC APB2 enable clock register. */
#define RCC_APB2ENR_OFFSET (0x44ul)
#define RCC_APB2ENR (*(volatile unsigned int*)(RCC_BASE + RCC_APB2ENR_OFFSET))

/* GPIOA port mode register. */
#define GPIOA_MODER_OFFSET (0x0ul)
#define GPIOA_MODER (*(volatile unsigned int*)(GPIOA_BASE + GPIOA_MODER_OFFSET))

/* GPIOA port pull-up/pull-down register */
#define GPIOA_PUPDR_OFFSET (0xCul)
#define GPIOA_PUPDR (*(volatile unsigned int*)(GPIOA_BASE + GPIOA_PUPDR_OFFSET))

/* GPIOA alternate function high register. */
#define GPIOA_AFRH_OFFSET (0x24ul)
#define GPIOA_AFRH (*(volatile unsigned int*)(GPIOA_BASE + GPIOA_AFRH_OFFSET))

/* USART6 status register. */
#define USART1_SR_OFFSET (0x0ul)
#define USART1_SR (*(volatile unsigned int*)(USART1_BASE + USART1_SR_OFFSET))

/* USART6 Data register. */
#define USART1_DR_OFFSET (0x4ul)
#define USART1_DR (*(volatile unsigned int*)(USART1_BASE + USART1_DR_OFFSET))

/* Baudrate register */
#define USART1_BRR_OFFSET (0x8ul)
#define USART1_BRR (*(volatile unsigned int*)(USART1_BASE + USART1_BRR_OFFSET))

/* Control Register 1 */
#define USART1_CR1_OFFSET (0xCul)
#define USART1_CR1 (*(volatile unsigned int*)(USART1_BASE + USART1_CR1_OFFSET))

/* Control Register 2 */
#define USART1_CR2_OFFSET (0x10ul)
#define USART1_CR2 (*(volatile unsigned int*)(USART1_BASE + USART1_CR2_OFFSET))

/* Control Register 3 */
#define USART1_CR3_OFFSET (0x14ul)
#define USART1_CR3 (*(volatile unsigned int*)(USART1_BASE + USART1_CR3_OFFSET))

/* Systick control and status register. */
#define SYST_CSR_OFFSET (0x0ul)
#define SYST_CSR (*(volatile unsigned int*)(SYSTICK_BASE + SYST_CSR_OFFSET))

/* Systick reload value register */
#define SYST_RVR_OFFSET (0x4ul)
#define SYST_RVR (*(volatile unsigned int*)(SYSTICK_BASE + SYST_RVR_OFFSET))

/* Systick current value register */
#define SYST_CVR_OFFSET (0x8ul)
#define SYST_CVR (*(volatile unsigned int*)(SYSTICK_BASE + SYST_CVR_OFFSET))

/* LED */
#define GPIOA_ODR_OFFSET (0x14ul)
#define GPIOA_ODR (*(volatile unsigned int*)(GPIOA_BASE + GPIOA_ODR_OFFSET))

/* Button: GPIOC input mode access */
#define GPIOC_MODER_OFFSET (0x0ul)
#define GPIOC_MODER (*(volatile unsigned int*)(GPIOC_BASE + GPIOC_MODER_OFFSET))

#define GPIOC_IDR_OFFSET (0x10ul)
#define GPIOC_IDR (*(volatile unsigned int*)(GPIOC_BASE + GPIOC_IDR_OFFSET))

/*                              Configuration values                                    */
/* CPACR register specifies the access privileges for cooprocessor: 0b11 Full access. */
#define FPU_CPACR_CP10       (3ul << 20)
#define FPU_CPACR_CP11       (3ul << 22)
/* Enable clock to IO Port A */
#define RCC_AHB1ENR_GPIOAEN  (1ul << 0)
/* Set GPIOA pins 11 and 12 as alternate function  */
#define GPIOA_MODER9         (2ul << 18)
#define GPIOA_MODER10        (2ul << 20)
/* Configure GPIO AFR(Alternate Function register) for USART6 */
#define GPIOA_AFRH9_AF7      (7ul << 4)
#define GPIOA_AFRH10_AF7     (7ul << 8)
/* USART lines must be held high */
#define GPIOA_PUPDR9         (1ul << 18)
#define GPIOA_PUPDR10        (1ul << 20)
/* Enable APB2 peripheral clock. */
#define RCC_APB2ENR_USART1EN (1ul << 4)
/* Baudrate configuration */
#define USART1_CR1_OVER8     (1ul << 15)
#define USART1_BRR_MANTISSA  (45ul << 4) /* Mantissa = SystemClock/(8*(2-OVER8)*BAUDRATE) = 84000000/(16*115200) = 45.652 = 45*/
#define USART1_BRR_FRACTION  (9ul << 0)  /* Fraction = 0.5625 (page 527 Reference manual), 16 * 0.5625 = 9 */
/* Enable transciver and receiver */
#define USART1_CR1_TE        (1ul << 3)
#define USART1_CR1_RE        (1ul << 2)
/* Configure word length */
#define USART1_CR1_M         (1ul << 12)
/* Configure parity control */
#define USART1_CR1_PCE       (1ul << 10)
/* Configure stop bits */
#define USART1_CR2_STOP      (3ul << 12)
/* Disable RTS*/
#define USART1_CR3_RTSE      (1ul << 8)
/* Disable CTS */
#define USART1_CR3_CTSE      (1ul << 9)
/* Enable USART6 module */
#define USART1_CR1_UE        (1ul << 13)
/* Check data register empty */
#define USART1_SR_TXE        (1ul << 7)
/* MASK Data register */
#define USART1_DR_MASK       (0xFFul << 0)
/* Check transmission complete */
#define USART1_SR_TC         (1ul << 6)
/* Enable Systick counter */
#define SYST_CSR_ENABLE      (1ul << 0)
/* Enable Systick exception request */
#define SYST_CSR_TICKINT     (1ul << 1)
/* Indicates Systick source clock */
#define SYST_CSR_CLKSOURCE   (1ul << 2)
/* SysTick count flag, returns 1 if timer counted to 0 since last time this was read. */
#define SYST_CSR_COUNTFLAG   (1ul << 16)
/* SysTick down counter */
#define DELAY_MS             (0x1481Ful & 0xFFFFFFul) /* 83,999 pulses = 1 mS */
/* Clear Systick value register */
#define SYST_CVR_CURRENT_CLR (0x000000ul)
/* Determine how many ticks are need */
#define TICK_FREQUENCY       (1ul)
/* Maximun delay allowed */
#define MAX_DELAY            (0xFFFFFFFFul)
/* PA5 configured as output */
#define GPIOA_MODER5         (1ul << 10)
/* Port output data */
#define GPIOA_ODR_ODR5       (1U << 5)
/* Port A pin 5  */
#define PORT_A_P5            (GPIOA_ODR_ODR5)
/* Use a system name for PA5 */
#define GREEN_LED            (PORT_A_P5)
/* Enable clock to GPIOC */
#define RCC_AHB1ENR_GPIOCEN  (1ul << 2)
/* Port C pin 13 as input */
#define GPIOC_MODER13        (3ul << 26)
#define PORT_C_P13           (1ul << 13)
#define BLUE_BUTTON          (PORT_C_P13)

/*                              Variables                            */
/* SysTick Variables */
volatile unsigned int get_current_tick;
volatile unsigned int get_current_tick_p;

/*                         Function prototypes                        */
/*             FPU functions           */
void config_CPACR_privileges(void);
/*            USART functions          */
void init_USART (void);
int __io_putchar (int ch);
/*           SysTick functions         */
void init_timebase (void);
void tick_increment (void);
unsigned int get_tick (void);
void delay_ms (unsigned int delay);
/*           LED functions         */
void init_LED (void);
void LED_On (void);
void LED_Off (void);
/*         Button functions        */
void init_button(void);
bool get_button_status (void);

/*                         Special functions                         */
/* The compiler attempts to inline the function, regardless of the characteristics of the function. 
** However, the compiler does not inline a function if doing so causes problems. For example, a 
** recursive function is inlined into itself only once. 
** The inline keyword suggests to the compiler that the function be inlined. Normally, when a 
** function is called, the current contents of the registers are pushed (copied) to memory. 
** Once the function returns, they are popped (copied back).
** Inline functions are not always important, but it is good to understand them. The basic idea is 
** to save time at a cost in space. Inline functions are a lot like a placeholder. Once you define 
** an inline function, using the 'inline' keyword, whenever you call that function the compiler will 
** replace the function call with the actual code from the function. 
*/
/* Enable IRQ interrupts: by clearing the I-bit in the CPSR. */
__attribute__((__always_inline__)) static inline void __enable_irq (void)
{
    asm volatile ("cpsie i" : : : "memory");
}

/* Enable IRQ interrupts: by setting the I-bit in the CPSR. */
__attribute__((__always_inline__)) static inline void __disable_irq (void)
{
    asm volatile ("cpsid i" : : : "memory");
}

/* Set Main Stack pointer */
__attribute__((__always_inline__)) static inline void __set_MSP (unsigned int MainStackTop)
{
    asm volatile ("MSR msp, %0\n" : : "r" (MainStackTop) : "sp");
}

/*                                   Bootloader                                   */
/* Function pointer type definition, points a void function type. */
typedef void(*func_ptr)(void);
/* Address where application will start. */
#define APPLICATION_ADDRESS (0x08018000ul) /* Sector 2, Page 45 from STM32F401RE Reference Manual */
#define EMPTY_MEMORY        (0xFFFFFFFFul)
#define MSP_LOCATION        (0x20018000ul)
#define MSP_MASK            (0x2FF18000ul)

/* Aplication jump function */
void load_default_app(void);

/*                            Shared APIs                                         */
/* Struct shared APIS */
struct bl_shared_apis
{
    /* Pointers to functions  */
    void (*config_CPACR_privileges)(void);
    void (*init_USART) (void);
    void (*init_timebase) (void);
    void (*init_LED) (void);
    void (*LED_On) (void);
    void (*LED_Off) (void);
    void (*init_button) (void);
    bool (*get_button_status) (void);
};

struct bl_shared_apis apis_callouts __attribute__((section(".shared_apis"))) = {\
    config_CPACR_privileges,
    init_USART,
    init_timebase,
    init_LED,
    LED_On,
    LED_Off,
    init_button,
    get_button_status
};

int main (void)
{
    config_CPACR_privileges();
    init_USART();
    init_timebase();
    init_LED();
    init_button();
    load_default_app();
    while(1u)
    {

    }
    return EXIT_SUCCESS;
}

/*                         Function definitions                        */
/*             FPU functions           */
void config_CPACR_privileges(void)
{
    /* Co-processor full access privileges. */
    FPU_CPACR |= FPU_CPACR_CP10;
    FPU_CPACR |= FPU_CPACR_CP11;
}
/*            USART functions          */
/* USART initialization */
void init_USART(void)
{
    /* Enable clock access to GPIOA. */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    /* Set PA11 and PA12 as alternate function mode. */
    GPIOA_MODER |= GPIOA_MODER9;
    GPIOA_MODER |= GPIOA_MODER10;
    /* Set function alternate type: AF7 USART6. See Microcontroller Reference Manual, page 151.*/
    GPIOA_AFRH |= GPIOA_AFRH9_AF7;
    GPIOA_AFRH |= GPIOA_AFRH10_AF7;
    /* USART lines must be held high, it is needed to pull-up resistors. */
    GPIOA_PUPDR |= GPIOA_PUPDR9;
    GPIOA_PUPDR |= GPIOA_PUPDR10;
    /* Enable clock access to UART6. */
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    /* Configure UART baudrate: SysClock= 84 MHz, AHB1= 84 MHz, APB2= 42 Mhz, Oversample= 16, Baudrate= 115200, data= 8 bits, Parity= none, stop bit= 1. */
    /* Oversampling 16 */
    USART1_CR1 &= ~USART1_CR1_OVER8;
    /* Configure baudrate */
    USART1_BRR |= USART1_BRR_MANTISSA;
    USART1_BRR |= USART1_BRR_FRACTION;
    /* Configure transfer direction. */
    USART1_CR1 |= USART1_CR1_TE;
    /* Configure receive direction. */
    USART1_CR1 |= USART1_CR1_RE;
    /* Configure word length */
    USART1_CR1 &= ~USART1_CR1_M;
    /* Configure parity control */
    USART1_CR1 &= ~USART1_CR1_PCE;
    /* Configure stop bits: 1 stop bit */
    USART1_CR2 &= ~USART1_CR2_STOP;
    /* Disable RTS */
    USART1_CR3 &= ~USART1_CR3_RTSE;
    /* Disable CTS */
    USART1_CR3 &= ~USART1_CR3_CTSE;
    /* Enable UART Module. */
    USART1_CR1 |= USART1_CR1_UE;
}
/* Send char function */
int __io_putchar(int ch)
{
    while(USART1_SR_TXE!=(USART1_SR & USART1_SR_TXE)); /* make sure transmit data is empty */
    USART1_DR = (USART1_DR_MASK & ch);
    while(USART1_SR_TC!=(USART1_SR & USART1_SR_TC));
    return ch;
}
/*           SysTick functions         */
void init_timebase (void)
{
    /* Disable global interrupts. */
    __disable_irq();
    /* Load the timer with number of clock cycles per second. */
    SYST_RVR = DELAY_MS;
    /* Clear current SysTick value register. */
    SYST_CVR = SYST_CVR_CURRENT_CLR;
    /* Select internal clock source. */
    SYST_CSR |= SYST_CSR_CLKSOURCE;
    /* Enable interrupt. */
    SYST_CSR |= SYST_CSR_TICKINT;
    /* Enable SysTick. */
    SYST_CSR |= SYST_CSR_ENABLE;
    /* Enable global interrupts. */
    __enable_irq();
}
/* Function to count Ticks. */
void tick_increment (void)
{
    get_current_tick += TICK_FREQUENCY;
}

/* Interrupt handler */
void Systick_Handler (void)
{
    tick_increment();
}

/* Return the number of ticks */
unsigned int get_tick (void)
{
    __disable_irq();
    get_current_tick_p = get_current_tick;
    __enable_irq();
    return get_current_tick_p;
}

/* Delay function in miliseconds */
void delay_ms (unsigned int delay)
{
    unsigned int tickstart = get_tick();
    unsigned int wait = delay;
    if(wait < MAX_DELAY)
    {
        wait += TICK_FREQUENCY;
    }
    else
    {/* Do nothing */}
    while ((get_tick() - tickstart) < wait){}
}

/*           LED functions         */
/* LED initialization. */
void init_LED (void)
{
    /* Enable clock access to GPIOA. */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    /* Configure PA5 as output. */
    GPIOA_MODER |= GPIOA_MODER5;
}
/* LED ON */
void LED_On (void)
{
    GPIOA_ODR |= GREEN_LED;
}
/* LED OFF */
void LED_Off (void)
{
    GPIOA_ODR &= ~GREEN_LED;
}
/*         Button functions        */
void init_button(void)
{
    /* Enable clock access to PORTC */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    /* Set PC13 as an input */
    GPIOC_MODER &= ~GPIOC_MODER13;
}

bool get_button_status (void)
{
    /* Note: Button is active low*/
    /* Check if button is pressed */
    if(GPIOC_IDR & BLUE_BUTTON)
    {
        return false;
    }
    else
    {
        return true;
    }
}
/*                                   Bootloader                                   */
/* Aplication jump function */
void load_default_app(void)
{
    unsigned int start_app_address;
    func_ptr load_app; /* load_app is a pointer to function to load application firmware. */
    printf("Bootloader started...\n\r");
    delay_ms(500ul);
    start_app_address = *(unsigned int*)(APPLICATION_ADDRESS + 4ul); /* Application will start at 0x08008004, see STM32F401 reference page 41 for explanation. */
    /* Checks if application memory is empty before loading. */
    if(MSP_LOCATION == (*(unsigned int*)APPLICATION_ADDRESS & MSP_MASK))
    {
        printf("Starting application...\n\r");
        /* Jump to application address */
        load_app = (func_ptr)start_app_address;
        /* Initialize main stack pointer */
        __set_MSP(*(unsigned int*)APPLICATION_ADDRESS);
        /* Loading application */
        load_app();
    }
    else
    {
        printf("No application found at location...\n\r");
    }
}