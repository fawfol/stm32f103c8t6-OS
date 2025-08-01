Designing and Implementing a Custom Embedded OS for STM32F103C8T6: A Comprehensive Guide

1. Introduction

This report details the architectural design and implementation considerations for a custom operating system (OS) tailored for the STM32F103C8T6 microcontroller. The primary objective is to develop a functional embedded system capable of file navigation, audio playback, and basic game execution. This endeavor necessitates the intricate integration of various hardware components, including tactile buttons for input, a 512MB 7-pin SD card for storage, a 128x64 I2C LCD for visual output, and a piezo speaker for audio. A critical architectural requirement is the OS's residence in flash memory, coupled with a robust boot mechanism that allows for booting from the SD card if the OS image size surpasses the internal flash capacity.

1.1. Project Overview and Objectives

The project aims to construct a custom embedded OS that provides a rich user experience on resource-constrained hardware. The core functionalities include a file navigation system, enabling users to browse and select audio and game files stored on an external SD card. Audio playback will be facilitated through a piezo speaker, while game execution will leverage the display and audio capabilities. User interaction will be managed via five tactile buttons, providing intuitive control over the system's features. The design prioritizes efficiency and responsiveness, essential characteristics for embedded applications. The innovative boot-from-SD-card capability is a cornerstone of this design, offering unparalleled flexibility for OS updates and the deployment of larger, more feature-rich applications that might otherwise exceed the microcontroller's inherent storage limitations.

1.2. STM32F103C8T6 Microcontroller Overview

The STM32F103C8T6 is a 32-bit microcontroller based on the ARM Cortex-M3 core, a processor optimized for low-cost and energy-efficient integrated circuits. This microcontroller is a popular choice in embedded systems due to its balance of performance and peripheral richness. Key features include its 32-bit RISC architecture, a comprehensive range of peripherals, and integrated flash memory and SRAM. The typical memory map of such a device delineates distinct regions for Flash (non-volatile memory for firmware and application code) and SRAM (volatile memory for temporary data, variables, and the stack).

A crucial aspect of the STM32F103C8T6, particularly for OS design, is the absence of a Memory Management Unit (MMU). This characteristic fundamentally shapes the OS architecture, making traditional "full-fledged" operating systems, which rely on virtual memory and hardware-enforced memory protection, impractical. Consequently, the custom OS must adopt a bare-metal approach or utilize a very minimal Real-Time Operating System (RTOS) kernel designed specifically for Cortex-M processors.

A significant consideration for this microcontroller is its flash memory size. Officially, the STM32F103C8T6 is specified to have 64KB of Flash memory. However, empirical observations and community reports indicate that many popular development boards, such as the "Bluepill," equipped with MCUs marked "STM32F103C8T6," actually possess 128KB of Flash. This extended capacity has been verified by successfully programming binaries exceeding 64KB (e.g., a 90KB Forth binary) by overriding the chip's reported size in flashing tools like OpenOCD.

This discrepancy between the officially stated and often physically available flash size presents a critical architectural decision point that influences the entire OS design. If the OS is designed assuming only 64KB, it might be unnecessarily constrained, leading to a more limited feature set or requiring aggressive memory optimization. Conversely, leveraging the full 128KB allows for a richer and more complex OS, but it necessitates specific toolchain configurations, such as modifying linker scripts and flashing utility settings, to unlock the entire capacity. This is not merely a factual detail but a foundational constraint or opportunity that directly impacts the feasibility and scope of the custom OS, influencing decisions on code size, feature inclusion, and the complexity of the bootloader. The development approach must account for this potential variability and configure the build environment accordingly to maximize available resources.

2. Core OS Architecture

The design of a custom OS for the STM32F103C8T6 requires a meticulous approach to task management, memory allocation, and hardware interaction, all within the inherent limitations of a microcontroller environment.

2.1. OS Paradigm: Bare-Metal with Cooperative Multitasking

The STM32F103C8T6, like other ARM Cortex-M processors, is optimized for low-cost and energy-efficient integrated circuits and typically lacks a Memory Management Unit (MMU). The absence of an MMU renders "full-fledged" operating systems, which rely on virtual memory and hardware-enforced memory protection, impractical for this architecture. Consequently, the custom OS must adopt either a bare-metal programming approach or utilize a Real-Time Operating System (RTOS) specifically designed for Cortex-M devices.

Bare-metal programming involves direct control over hardware without an intervening OS, offering maximum control and minimal overhead. An RTOS, conversely, provides features such as task scheduling, inter-task communication, and real-time capabilities. Given the custom nature of this project and its resource constraints, a bare-metal approach augmented with cooperative multitasking has been selected. This choice prioritizes simplicity, direct hardware control, and a predictable execution environment.

The absence of an MMU, while a limitation for complex OSes, also simplifies memory management by removing the overhead associated with virtual memory translation and protection. This characteristic, however, shifts the burden of memory protection and isolation onto the developer. Without hardware-enforced separation between tasks or between kernel and user space, a bug in one part of the code could potentially corrupt any other part of memory, leading to unpredictable system behavior. Therefore, developers must meticulously manage memory regions, implement boundary checks, and employ defensive programming techniques. This requires a disciplined approach to software design, where explicit management of memory regions and access permissions replaces hardware-level safeguards. The trade-off is between the reduced complexity and overhead of an MMU and the increased responsibility for software robustness.

2.2. Task Scheduling and Management

The custom OS employs a cooperative multitasking scheduler, where tasks voluntarily relinquish control of the CPU to allow other tasks to execute. This approach simplifies implementation by avoiding complex reentrancy issues inherent in preemptive multitasking. The scheduler's operation is driven by a real-time clock counter, typically implemented through a periodic timer interrupt. The SysTick timer, a 24-bit system timer universally present in Cortex-M microcontrollers, is an ideal candidate for generating these regular interrupts to drive the scheduler.

Tasks within this OS are defined as functions, each assigned a unique identifier and capable of receiving parameters via a pointer. Context switching, the process of saving the state of the currently executing task and loading the state of the next task, is a core function of the scheduler. The Cortex-M architecture facilitates this by providing two stack pointers: the Main Stack Pointer (MSP), typically used by the kernel and interrupt handlers, and the Process Stack Pointer (PSP), generally used by application tasks. This dual-stack mechanism allows for a clean separation between the kernel's operational context and individual task contexts.

The SysTick handler is configured to trigger context switches periodically, implementing a round-robin scheduling style. To enhance the robustness and efficiency of context switching, the PendSV (Pendable Service) exception is utilized. PendSV is a software-triggerable interrupt that can be configured to the lowest interrupt priority level. This design ensures that the actual context switch operation is deferred until no other higher-priority Interrupt Service Routines (ISRs) are active. This prevents context switches from occurring in the middle of critical ISRs, thereby enhancing determinism and mitigating potential race conditions. The core context switching routine, which involves direct manipulation of CPU registers and stack pointers, is often implemented in assembly language for optimal performance and precise control.

The strategic combination of SysTick for periodic task triggering and PendSV for deferred, low-priority context switching enables a robust and efficient "pseudo-preemptive" RTOS-like behavior in a bare-metal environment. This architectural pattern optimizes interrupt latency by ensuring that time-critical ISRs are not interrupted by a context switch. It also prevents race conditions by centralizing the context switch logic within a dedicated, lowest-priority handler. This effectively provides a lightweight, custom RTOS kernel without the memory footprint or complexity overhead typically associated with commercial RTOS products, a crucial advantage for resource-constrained systems like the STM32F103C8T6.

For managing asynchronous events, an event handling framework is implemented using a central event queue. This queue is typically realized as a fixed-size circular buffer (also known as a ring buffer), which stores notifications or requests in a First-In-First-Out (FIFO) manner. This structure serves to decouple event producers (e.g., button interrupts, peripheral drivers) from event consumers (e.g., UI logic, game engine), allowing for asynchronous processing and improving overall system responsiveness.

2.3. Memory Management for Resource-Constrained Systems

Effective memory management is paramount for the reliable operation of the custom OS on the STM32F103C8T6, given its limited resources.

Flash memory serves as the non-volatile storage for the firmware and application code, while SRAM provides volatile storage for temporary data, variables, the stack, and the heap. The linker script, typically a 

.ld file for GCC toolchains, is indispensable for defining the precise memory layout, including the start addresses and lengths of both Flash and RAM regions. This script partitions the executable into distinct sections: the 

.text section (containing executable code and read-only data, residing in Flash), the .data section (for initialized global variables, copied from Flash to SRAM at startup), and the .bss section (for zero-initialized global variables, allocated in SRAM).

Memory allocation strategies must be carefully balanced. Static allocation, where memory is reserved at compile-time, offers predictability and avoids runtime overhead, making it generally preferred for critical components and fixed-size buffers in embedded systems. Dynamic allocation (e.g., using 

malloc/free), while offering flexibility, can lead to memory fragmentation and unpredictable delays, rendering it less suitable for real-time systems without an MMU.

To mitigate the challenges associated with dynamic allocation, custom memory pools or allocators are employed. These involve pre-allocating fixed-size blocks of memory, which can then provide deterministic allocation times and significantly reduce fragmentation. Examples of such custom fixed-block allocators include 

fb_allocator and x_allocator, which are particularly useful for objects of known sizes that are frequently created and destroyed.

Stack management is critical to prevent overflows, which can lead to system crashes. This involves monitoring stack usage and allocating sufficient space to accommodate worst-case scenarios, such as deep function call nesting or large local variables. Similarly, the heap, used for dynamic memory, must be carefully managed to prevent fragmentation that could lead to allocation failures.

Given the tight memory constraints of the STM32F103C8T6 (20KB SRAM and 64KB/128KB Flash), the choice of memory allocation strategy is not merely a preference but a critical factor for system stability and performance. Uncontrolled use of standard malloc/free without an MMU can rapidly lead to heap fragmentation, memory leaks, and unpredictable behavior, compromising the system's reliability. This underscores the necessity for a disciplined approach to memory management in bare-metal environments, where the developer assumes the role of the memory manager.

The following table outlines a proposed memory map for the STM32F103C8T6, illustrating the partitioning of its internal Flash and SRAM. This visual representation is crucial for understanding how the bootloader and OS components coexist and where different data types reside, providing a clear blueprint for linker script configuration and capacity planning.

Table 1: Proposed Memory Map for STM32F103C8T6
Memory Region	Start Address (Hex)	End Address (Hex)	Size (KB)	Purpose/Contents
Flash	0x08000000	0x0801FFFF	128 (or 64)	Program Code, Read-Only Data, Vector Table
SRAM	0x20000000	0x20004FFF	20	Initialized Data (.data), Zero-Initialized Data (.bss), Heap, Stack, Custom Memory Pools

2.4. Hardware Abstraction Layer (HAL) Design

A Hardware Abstraction Layer (HAL) is a software layer that provides a standardized interface for accessing and controlling hardware components, effectively abstracting away low-level complexities. The principles behind HAL design include improved code portability across different hardware platforms, reduced development time, enhanced maintainability, and increased flexibility. The HAL typically resides between the application layer and the physical hardware, presenting a set of high-level APIs that allow developers to interact with peripherals without needing to understand their specific register details.

For this custom OS, a HAL structure is adopted to manage complexity, even if the primary goal is not immediate portability to other microcontrollers. A custom HAL can be implemented using a set of function pointers or C structures that define common peripheral interfaces, such as init, read, write, and toggle for GPIO operations. This approach encapsulates direct register access within a defined interface, allowing for modular testing of individual drivers and facilitating future expansion or modification without affecting the entire OS. This strategy is particularly effective in mitigating the "spaghetti code" risk often associated with extensive direct register manipulation in bare-metal projects.

While commercial HALs, such as those provided by the STM32Cube ecosystem, offer convenience and standardized APIs, they often come with a larger memory footprint due to their generality. For a custom OS on a resource-constrained microcontroller, a bare-metal or highly optimized Low-Layer (LL) driver approach is often preferred for its minimal code footprint and fine-grained control over hardware timing and register settings. This represents a classic embedded trade-off between development ease and resource optimization. By designing a lean, custom HAL, the project can achieve a balance, ensuring that only necessary features are included, thereby minimizing flash and RAM usage while maintaining a structured and maintainable codebase.

3. Bootloader and Firmware Management

The requirement for the OS to boot from an SD card, especially if it exceeds the internal flash capacity, mandates a sophisticated multi-stage bootloader architecture and careful flash memory management.

3.1. Multi-Stage Bootloader Architecture

The STM32F103C8T6 features a built-in system bootloader residing in ROM, accessible via specific BOOT pins. This factory-programmed bootloader supports various communication interfaces like UART, I2C, SPI, and USB-DFU for initial firmware loading. However, this ROM-based bootloader does not inherently support SDIO or SPI-connected SD cards, nor does it understand file systems like FATFS. Therefore, a custom primary bootloader is essential for enabling SD card booting.

This custom primary bootloader will reside at the very beginning of the internal flash memory (e.g., starting at address 0x08000000). Its design prioritizes minimalism and robustness. Upon system reset, this primary bootloader executes first. Its core function is to evaluate specific conditions—such as the presence of an SD card, the state of a designated button, or the integrity of an existing application in flash—to determine the subsequent boot path. This decision logic allows it to either jump to a main application already residing in internal flash or initiate the process of loading a new OS image from the SD card.

If the main OS image is too large to fit entirely within the microcontroller's internal flash, or if a firmware update is required, the primary bootloader will act as a secondary application loader by fetching the OS image from the SD card. This process involves several complex steps: initializing the SD card, parsing its file system (FATFS), locating the OS binary file, and then programming this binary into the appropriate region of the internal flash memory. Alternatively, for very large OS images or specific use cases, the OS could potentially be loaded directly into SRAM for execution, though this would be volatile.

The necessity to boot from the SD card, especially when the OS image exceeds the internal flash capacity, implies a sophisticated two-stage bootloader design rather than a simple direct load. The primary bootloader, residing in a small, fixed, and protected region of internal flash, functions as a "boot selector" or "bootstrapping agent." This initial stage must be minimal and highly reliable, containing only the essential code to initialize critical peripherals (e.g., basic GPIO, a minimal SPI interface for SD card detection) and execute the decision logic. The more complex tasks, such as full SD card initialization, comprehensive file system parsing, and the actual flash programming or direct RAM loading of the OS image, are handled by a secondary stage. This secondary stage can be larger and more feature-rich because it is loaded and executed by the primary bootloader. This multi-stage architecture is fundamental for achieving flexibility and future-proofing the system, enabling the deployment of larger OS images and facilitating easy field updates. It also enhances system resilience; should the secondary stage or the main OS image become corrupted, the minimal primary bootloader can still provide a fallback mechanism, such as entering a recovery mode or a DFU (Device Firmware Update) state.

3.2. Flash Memory Layout and Linker Script Modifications

The internal flash memory of the STM32F103C8T6 must be carefully partitioned into distinct regions to accommodate both the primary bootloader and the main custom OS application. A common partitioning scheme involves reserving the initial portion of the flash, for instance, the first 16KB (from 0x08000000 to 0x08003FFF), for the primary bootloader. The main OS application would then commence from a subsequent address, such as 0x08004000. This logical division necessitates precise modifications to the linker scripts (

.ld files) for both the bootloader project and the main OS application project. The linker script defines the memory regions and specifies where different code and data sections (e.g., 

.text, .data, .bss) for each binary should be placed.

A critical aspect of this multi-application memory layout is vector table relocation. Upon reset, the Cortex-M processor fetches the initial stack pointer and the address of the reset handler from fixed locations (0x0 and 0x4, respectively) within the aliased memory space, which typically maps to the beginning of the flash memory (0x08000000). When the primary bootloader transfers control to the main OS application, the application must use its own set of interrupt handlers and its own stack. To achieve this, the application's vector table must be correctly positioned and its address communicated to the CPU. This is typically accomplished by programming the Vector Table Offset Register (VTOR) within the System Control Block (SCB) to point to the application's base address in flash. While some Cortex-M variants (like M0) might present challenges with direct VTOR relocation, often requiring the vector table to be copied to SRAM and remapped, the Cortex-M3 (used in STM32F103) generally supports straightforward VTOR relocation.

The dynamic capability of booting the OS from either internal flash or an SD card mandates a flexible memory map and meticulous vector table management. The practice of relocating the vector table, even if not strictly necessary for all Cortex-M3 scenarios compared to M0, provides a robust and clean mechanism for application jumps. This ensures that the application operates within its own defined context, utilizing its dedicated interrupt handlers and stack, rather than relying on the bootloader's context. This separation is crucial for system stability, modularity, and facilitates independent development and debugging of the bootloader and the main OS. The linker script modifications and VTOR manipulation are not merely technical steps but represent a deliberate architectural choice to ensure the integrity and independence of the loaded OS or application.

The following table presents a proposed flash memory map, illustrating the partitioning for the primary bootloader and the main OS application. This serves as a blueprint for configuring the linker scripts and understanding the boot flow.

Table 2: Proposed Flash Memory Map for Bootloader and Application
Memory Segment	Start Address (Hex)	End Address (Hex)	Size (KB)	Purpose/Contents
Primary Bootloader	0x08000000	0x08003FFF	16	Bootloader Code, Minimal Drivers, Boot Logic, Vector Table
Main OS Application	0x08004000	0x0801FFFF	112 (or 48)	Custom OS Code, Application Data, Vector Table (relocated)

3.3. SD Card Boot Process

The process of booting the OS from an SD card begins with the precise initialization of the SD card in SPI mode. This sequence involves a power-up phase where the SD card is given sufficient time to stabilize. Subsequently, the microcontroller sends at least 74 clock cycles with the Chip Select (CS) and Master Out Slave In (MOSI) lines held high. Following this, the critical CMD0 (GO_IDLE_STATE) command is sent with the CS line pulled low, which transitions the SD card into SPI mode. The expected response is an R1 response with a value of 0x01, indicating the idle state. Subsequent commands, such as CMD8 (SEND_IF_COND) for SDHC/SDXC cards and ACMD41 (APP_SEND_OP_COND) for initialization, are then issued to fully prepare the card for operation. It is important to initiate communication with a low SPI clock frequency (100-400 kHz) during initialization, which can then be increased for faster data transfer once the card is ready.

SD cards typically store data in 512-byte blocks and are commonly formatted with FAT16 or FAT32 file systems. To interact with these file systems, the lightweight and platform-independent FatFs library (a C implementation for FAT16/FAT32) is integrated. FatFs provides standard file I/O APIs such as 

fopen(), fread(), and fwrite(), simplifying file system operations for the application layer. The integration process involves providing low-level disk I/O functions (e.g., 

disk_initialize, disk_read, disk_write) that FatFs invokes to interact with the underlying SPI hardware. Configuration of FatFs options, such as enabling long filename support (

FF_USE_LFN) and optimizing buffer sizes (FAT_BUFFER_SECTORS, FAT_BUFFERS), is crucial to balance performance and memory usage, given the microcontroller's limited resources.

The SD card functions as a secondary storage for the OS, which implies not just raw block access but a full-fledged file system (FATFS). This adds significant code overhead due to the FatFs library and requires careful integration with the SPI driver, transforming a simple storage read into a complex file system operation. The necessity for a robust file system underscores that the SD card is more than just a memory extension; it serves as a dynamic storage medium for interchangeable applications and data. While FatFs simplifies file operations by offering a standard API, its memory footprint, particularly when features like Long File Name (LFN) support are enabled, must be meticulously managed on a resource-constrained microcontroller. This presents a practical trade-off: the convenience and enhanced features of LFN come at the cost of precious RAM and Flash space. For instance, LFN support can add tens to hundreds of kilobytes to the code size, a substantial portion of the STM32F103's available memory. Thus, enabling specific FatFs features must be a deliberate decision, potentially requiring the OS to forgo LFN support and rely on 8.3 filenames if memory is extremely tight, impacting user experience but preserving critical resources.

After the firmware binary is loaded from the SD card, its integrity must be verified before it is programmed into flash or executed. This is typically achieved by calculating a checksum (e.g., CRC32) over the received binary data and comparing it against a pre-computed checksum, which is often appended to the binary file itself.

The following table provides essential SD card SPI commands and their expected responses during the initialization phase, serving as a quick reference for implementation and troubleshooting.

Table 3: Essential SD Card SPI Commands and Responses (Initialization)
Command	Description	Argument (Hex)	CRC (Hex)	Expected Response	Phase
CMD0	GO_IDLE_STATE	0x00000000	0x94	R1: 0x01	Power-up, Idle State
CMD8	SEND_IF_COND	0x000001AA	0x87	R7: 0x01AA (for SDHC/SDXC)	Initialization
ACMD41	APP_SEND_OP_COND	0x40000000 (HCS bit set)	0x01	R1: 0x00 (after initialization)	Initialization

3.4. In-Application Programming (IAP) Considerations

The requirement for the OS to "boot from the SD card if it exceeds flash capacity" directly implies the implementation of an In-Application Programming (IAP) mechanism. This means that the primary bootloader must possess the capability to erase and reprogram the main application area within the internal flash memory.

The IAP process involves several critical steps. First, the bootloader must check for and potentially disable any write protection on the flash memory sectors designated for the application. After ensuring write access, the flash memory is initialized, and the old application space is erased. Flash programming is then performed block-by-block, often in specific sizes (e.g., 8 bytes at a time for STM32 flash programming), with the data read from the SD card. Finally, the programmed content is verified by reading it back from flash and comparing it against the original data from the SD card to ensure data integrity. Robust error handling throughout the IAP process is paramount to prevent "bricking" the device, as a failed update could render the microcontroller unusable.

The "boot from SD card if it exceeds flash capacity" is not merely about loading the OS; it represents a full-fledged firmware update mechanism. This necessitates that the bootloader incorporates robust IAP capabilities, encompassing flash erase, write, and verification procedures. This adds considerable complexity to the bootloader's design, requiring meticulous attention to fault tolerance. A failure during the update process, such as a power interruption or data corruption, could render the device inoperable. Consequently, the bootloader must be engineered as a highly reliable and fault-tolerant piece of software, as its proper functioning directly impacts the device's ability to operate. This is a critical feature for the long-term deployability and maintainability of the custom OS, transforming the device into a system that can be easily updated and adapted in the field.

4. Peripheral Drivers and Integration

The successful operation of the custom OS hinges on the meticulous implementation of low-level drivers for each specified hardware peripheral, prioritizing direct register access or minimal HAL usage for optimal performance and memory footprint.

4.1. SD Card Interface (SPI)

The SD card interface relies on the Serial Peripheral Interface (SPI) protocol. For the STM32F103C8T6, the standard SPI1 pins are typically PA4 (NSS/CS), PA5 (SCK), PA6 (MISO), and PA7 (MOSI). The user's query specifies the following connections: "SD card pin 1 (DAT3/CD) to STM32 A4 for CS, pin 2 (CMD - MOSI) to A7, pin 5 (CLK - SCK) to A5, and pin 7 (DAT0) to A7."

A critical contradiction arises in the user's specified pinout: connecting both CMD (MOSI) and DAT0 to STM32 A7. In standard SPI mode, DAT0 typically functions as the MISO (Master In Slave Out) line, while CMD is MOSI (Master Out Slave In). Assigning both MOSI and MISO to the same physical pin (A7) is electrically impossible for full-duplex SPI communication, which is the standard mode for SD card data transfer. This discrepancy suggests either a misunderstanding of SPI pin roles or a typographical error in the query. For a functional and efficient implementation, the standard SPI pinout must be followed, where MOSI and MISO are distinct pins. Therefore, it is assumed that DAT0 should be connected to STM32 PA6, which is the standard MISO pin for SPI1. Adhering to the standard hardware SPI peripheral is crucial for performance, as attempting a custom bit-banging SPI implementation for full-duplex communication would be significantly more complex, slower, and incur substantial CPU overhead, severely impacting the OS's ability to perform other tasks during SD card access.

The STM32F103C8T6's SPI peripheral must be configured as a master. This involves enabling the necessary clocks for SPI1, Alternate Function I/O (AFIO), and GPIOA. The GPIO pins for SCK, MISO, and MOSI are configured as alternate function push-pull outputs or inputs, respectively, with appropriate speed settings. The SPI peripheral settings include full-duplex communication, 8-bit data size, and MSB-first transmission. Clock polarity (CPOL) and phase (CPHA) must be set to values compatible with the SD card (typically CPOL=0/CPHA=0 or CPOL=1/CPHA=1, though CPOL=0/CPHA=1 has been observed to work with some devices). A software slave select (NSS Soft) is generally preferred to manage multiple SPI slave devices. The baud rate prescaler is selected to achieve a suitable clock frequency; for instance, 

SPI_BaudRatePrescaler_64 on the APB2 bus (72 MHz) yields a 1.125 MHz SPI clock. Direct register access can be used for SPI configuration to ensure minimal footprint and maximum control.

Beyond the initialization sequence, the SD card low-level protocol involves specific commands for data transfer, such as CMD17 (READ_SINGLE_BLOCK) and CMD24 (WRITE_BLOCK). Data is transferred in 512-byte blocks, and Cyclic Redundancy Check (CRC) is used for data integrity, though it can often be ignored by default for most commands after initialization.

The following table provides the corrected and recommended mapping for the SD card's 7-pin interface to the STM32F103C8T6's SPI pins, addressing the user's specific query and resolving the identified contradiction for a functional implementation.

Table 4: SD Card SPI Pinout and STM32 Connections (Corrected)
SD Card Pin #	SD Card Pin Name	STM32F103C8T6 Pin	STM32 Function	Notes
1	DAT3 / CS (Chip Select)	PA4	SPI1_NSS / GPIO (Software CS)	User specified, standard for CS
2	CMD (MOSI)	PA7	SPI1_MOSI	User specified, standard for MOSI
3	GND	GND	Ground	
4	VCC	3.3V	Power Supply	
5	CLK (SCK)	PA5	SPI1_SCK	User specified, standard for SCK
6	GND	GND	Ground	
7	DAT0 (MISO)	PA6	SPI1_MISO	User specified A7 is incorrect; PA6 is standard MISO for SPI1

4.2. User Input: 5 Tactile Buttons

The 5 tactile buttons serve as the primary user input mechanism. Each button is connected to a General Purpose Input/Output (GPIO) pin on the STM32F103C8T6. These GPIO pins are configured as inputs, typically with internal pull-up or pull-down resistors to ensure a defined state when the button is not pressed. Direct register access (e.g., 

GPIOx_CRL/CRH for configuration and GPIOx_IDR for reading input data) is employed for optimal control and minimal overhead. It is crucial to enable the clock for the relevant GPIO port (e.g., 

RCC_APB2ENR for GPIOA) before configuring the pins.

A significant challenge with mechanical buttons is "bouncing," a phenomenon that causes multiple rapid electrical transitions for a single physical press. Simple delays for debouncing are not suitable in a multitasking OS, as they would block the entire system, rendering it unresponsive. Instead, a timer-based software debouncing technique is implemented. This approach leverages the SysTick timer to track elapsed time (e.g., a 20ms debounce delay) and confirm a stable button state before registering a press event. This non-blocking method allows the CPU to continue executing other tasks while waiting for button stability, maintaining system responsiveness.

Button presses are handled via external interrupts (EXTI). GPIO pins connected to buttons are configured to trigger an interrupt on a specific edge (e.g., falling edge for a pull-up button press). An Interrupt Service Routine (ISR) for the EXTI line is implemented. This ISR does not directly process the button press but rather triggers the debouncing logic. Once the debouncing logic confirms a stable button press, it enqueues a button event into the OS's central event queue. This event-driven approach decouples the low-level hardware interrupt from the high-level application logic, improving modularity and responsiveness.

The implementation of button debouncing via a timer-based software approach (SysTick) is crucial for reliable user input in a responsive OS. This method avoids blocking delays, allowing the OS to continue processing other tasks while awaiting button stability. This is a common and elegant solution to a fundamental embedded system challenge, directly impacting user experience and system reliability by ensuring that only valid, stable button presses are registered.

4.3. Display Output: 128x64 I2C LCD (SSD1306)

The 128x64 I2C LCD is typically driven by an SSD1306 controller. Inter-Integrated Circuit (I2C) is a two-wire serial communication protocol (SDA for data, SCL for clock) well-suited for connecting multiple slave devices like the SSD1306.

The STM32F103's I2C peripheral is configured for communication. This involves enabling the clocks for both the I2C peripheral (e.g., I2C1) and the associated GPIO port (e.g., GPIOB for PB6/PB7). The GPIO pins are configured as alternate function open-drain outputs with internal or external pull-up resistors, a requirement for I2C bus operation. The I2C peripheral's clock frequency is set, typically to 100kHz for standard mode, and the peripheral is enabled.

I2C master operations involve generating a START condition, sending the slave device's 7-bit address along with a read/write bit, transmitting or receiving data bytes, and handling Acknowledge (ACK) or Not Acknowledge (NACK) signals.

The SSD1306 driver implementation involves sending a specific initialization sequence of commands to configure the display (e.g., display off, set multiplex ratio, set memory addressing mode, set contrast). Communication with the SSD1306 distinguishes between sending commands (control bytes) and sending display data (pixel buffer). Display data is typically buffered in the microcontroller's RAM (requiring approximately 1KB for a 128x64 monochrome display). This RAM buffer is then transferred to the SSD1306 via I2C to update the display. Open-source libraries, such as 

afiskon/stm32-ssd1306 or Matiasus/SSD1306, provide valuable examples for bare-metal or minimal HAL-based driver implementations.

Graphics primitives are implemented to draw on the display. These functions include drawing individual pixels, lines (horizontal, vertical, and diagonal), rectangles, circles, and rendering text. All these operations manipulate the in-RAM display buffer, which is then periodically transferred to the SSD1306 via I2C to update the physical screen.

The choice between a HAL-based or bare-metal I2C driver for the SSD1306 significantly impacts the custom OS's code size and performance. While a HAL simplifies development by offering high-level APIs, a bare-metal approach provides fine-grained control over I2C timing and register settings. This often results in a smaller code footprint and potentially faster display updates, which are critical for an OS running on a resource-constrained microcontroller. This decision embodies a classic embedded trade-off between development convenience and resource optimization. For a custom OS, a bare-metal or highly optimized Low-Layer (LL) approach for the display driver is generally preferred to align with the goals of a lean and efficient system.

4.4. Audio Output: Piezo Speaker

A passive piezo speaker requires an alternating current (AC) signal to produce sound, which can be effectively generated using Pulse Width Modulation (PWM) from the microcontroller. The frequency of the PWM signal directly determines the pitch of the tone, while the duty cycle can be modulated to control the volume or amplitude of the sound.

STM32 timers are configured in PWM mode for this purpose. The TIMx_ARR (Auto-Reload Register) sets the PWM signal's frequency, and the TIMx_CCRx (Capture/Compare Register) controls the duty cycle.

For efficient audio streaming of WAV PCM data, Direct Memory Access (DMA) is crucial. DMA enables the transfer of audio samples to the PWM peripheral without constant CPU intervention, thereby offloading the main processor. In this setup, DMA is configured in a circular mode to continuously transfer duty cycle values from a memory buffer (containing the processed WAV PCM samples) to the timer's 

CCRx register. This allows for continuous audio playback with minimal CPU usage, freeing the processor for other OS tasks. Projects like 

0xAA55-rs/PWMAudio demonstrate PWM-driven 48kHz 16-bit stereo audio playback on the STM32F103C8T6, although practical resolution limitations exist due to the MCU's clock speed.

WAV files typically consist of a RIFF header, followed by a "fmt" chunk (which specifies audio format parameters like sample rate, bit depth, and number of channels), and a "data" chunk containing the raw PCM audio samples. The OS requires a WAV parser to read this header, extract the audio parameters, and then stream the raw PCM data to the audio output system.

For continuous audio streaming, a circular buffer (ring buffer) is an ideal data structure. This buffer allows the DMA to continuously read samples from one end, while the file system and WAV parser concurrently write new samples to the other end, implementing a First-In-First-Out (FIFO) logic that ensures a smooth audio stream.

PWM-driven audio on a piezo speaker, while seemingly straightforward, requires careful consideration of sample rate, bit depth, and duty cycle. While DMA is essential for offloading the CPU during WAV PCM data streaming, the STM32F103C8T6's 72MHz clock imposes practical limitations on achievable audio fidelity. Theoretically, achieving full 16-bit resolution at a 48kHz sample rate would necessitate approximately 65,536 clock cycles per PWM period (48 kHz × 65,536 = 3.15 GHz), which vastly exceeds the microcontroller's 72MHz clock capabilities. Practical implementations, therefore, must compromise. For instance, at a 48kHz sample rate, the 72MHz clock provides only 1,500 clock cycles per PWM period (72 MHz / 48 kHz = 1,500). This limits the PWM resolution to roughly 10-11 bits (since 2^10 = 1024 and 2^11 = 2048), meaning 16-bit audio must be downsampled to fit this resolution. Alternatively, to maintain higher bit depth, the sample rate must be reduced, leading to lower audio fidelity (e.g., 8kHz or 16kHz audio). Furthermore, while DMA handles data transfer, parsing WAV files and potentially performing real-time resampling or bit-depth conversion still consumes valuable CPU cycles. This highlights a critical trade-off between desired audio quality and the inherent limitations of the microcontroller's processing speed, necessitating downsampling or reduced bit depth for practical implementation.

5. File Navigation System and Application Layer

This section details the high-level software components responsible for user interaction, file management, and the execution of application-specific logic, including basic game capabilities.

5.1. File System Abstraction Layer

The file navigation system relies heavily on the integration of the FatFs library, building upon its initial setup for the bootloader. The application layer utilizes FatFs APIs, such as f_opendir for opening directories, f_readdir for iterating through directory entries, f_open for opening files, f_read for reading file contents, and f_close for closing files. These functions enable the OS to navigate through directories and access audio and game files stored on the SD card.

FatFs provides a standard, POSIX-like file I/O interface, which significantly simplifies application development by abstracting the complexities of the underlying SD card and SPI communication. This abstraction allows developers to work with files using familiar functions, without needing to delve into the low-level details of sector reads or writes.

Careful consideration is given to FatFs configuration options to balance performance and memory usage. Features such as long filename support (FF_USE_LFN) can be enabled, but this comes with a memory cost, particularly for the necessary buffers and code page conversions. Optimizing buffer sizes (

FAT_BUFFER_SECTORS, FAT_BUFFERS) is also crucial to ensure efficient data transfer while minimizing the consumption of the microcontroller's limited RAM.

While FatFs simplifies file operations by providing a standard API, its memory footprint, especially with features like Long File Name (LFN) support, must be meticulously managed on a resource-constrained microcontroller. This represents a practical trade-off: the convenience and enhanced user experience of LFN come at the cost of precious RAM and Flash space. For instance, LFN support can add tens to hundreds of kilobytes to the code size, depending on the character sets included, which is a substantial portion of the STM32F103's available 64KB/128KB flash and 20KB RAM. Therefore, the decision to enable specific FatFs features must be a conscious trade-off. If memory is extremely tight, the OS might need to sacrifice LFN support and rely on 8.3 filenames, which would impact the user experience but preserve critical memory resources. This highlights the continuous optimization required in embedded systems development.

5.2. User Interface Logic

The five tactile buttons are logically mapped to common navigation actions: Up, Down, Select, Back, and Play/Pause. The debounced button events, processed as described in Section 4.2, are consumed from the OS's central event queue. A state machine approach is employed to manage the user interface flow, transitioning between states such as "Browsing Menu," "File Selected," "Playing Audio," and "In Game." This state machine interprets button presses contextually based on the current state, ensuring intuitive and predictable user interaction.

File and directory listings are rendered on the 128x64 I2C LCD using the SSD1306 graphics primitives detailed in Section 4.3. This involves drawing text for file names and directory entries, as well as highlighting selected items. Scrolling functionality is implemented to handle long lists of files that exceed the display's vertical resolution.

An event-driven architecture, underpinned by a central event queue implemented as a ring buffer, provides a robust pattern for decoupling UI input (button presses) from the core processing logic. This design significantly improves responsiveness by allowing button events to be processed asynchronously, preventing the UI from freezing even if background tasks are busy. This approach also enhances maintainability by centralizing event handling. New features or modifications to how an event is handled can be localized to the event processing logic, without affecting the button input routines or other parts of the OS. This makes the system more modular and easier to debug. This architectural pattern is fundamental for building interactive and responsive embedded systems, transforming raw hardware inputs into meaningful, actionable events that form the backbone of the OS's interaction with the user.

5.3. Basic Game Engine Concepts for Embedded Systems

The inclusion of "game files" necessitates a design approach that allows for dynamic content and logic without requiring recompilation of the entire OS. This is achieved through a data-driven design paradigm, where game logic and assets (e.g., level layouts, character sprites, game rules) are stored in external data files rather than being hardcoded into the executable.

This approach allows game content to be easily modified, or entirely new games to be added, simply by updating the data files on the SD card, without the need to recompile or reflash the core OS. For an embedded system, this means game data (e.g., simple text-based levels, small bitmap sprites, basic game scripts) resides on the SD card and is loaded into RAM at runtime as needed.

Games developed for this platform will leverage the existing graphics primitives for the SSD1306 LCD (pixel, line, text, and simple bitmap drawing) and the PWM audio output. Due to the limited processing power and display resolution of the STM32F103C8T6, the focus will be on simple 2D games, such as text-based adventures, puzzle games, or basic platformers.

Implementing "game files" through a data-driven approach means that game logic and assets are externalized from the core OS. This enables easier updates and new game development without recompiling the entire OS, effectively transforming the microcontroller into a "platform" for interchangeable game content. This is a powerful abstraction for extending functionality and user engagement. The core OS provides a "game engine" framework (offering graphics, audio, and input APIs), and individual "games" are essentially data sets that drive this engine. This design choice is critical for the long-term viability and appeal of the "game" feature, moving beyond a single hardcoded game to a system capable of hosting multiple, dynamically loaded interactive experiences, aligning perfectly with the vision of a versatile "custom OS." Furthermore, storing game assets on the larger SD card rather than in the microcontroller's limited internal flash memory is a significant advantage for resource management.

6. Development Workflow and Tools

A successful bare-metal embedded project, particularly one as complex as a custom OS, relies heavily on a meticulously configured development toolchain and a deep understanding of memory management.

6.1. Toolchain Setup (ARM-GCC, Linker, Debugger)

The recommended toolchain for cross-compilation is ARM-GCC, a free and open-source compiler suite specifically designed for ARM processors. The linker plays a crucial role in this process, combining the compiled object files and placing them into the microcontroller's memory according to the directives specified in the linker script. The linker script (

.ld file) is essentially the "OS" for memory layout in a bare-metal environment, dictating the precise location of every byte of code and data. Errors in the linker script can lead to difficult-to-diagnose runtime issues, such as incorrect vector table addresses or stack/heap overlaps.

Debugging capabilities are paramount for low-level embedded development. The ST-Link V2 programmer/debugger is a standard tool for connecting to the STM32F103C8T6 via SWD (Serial Wire Debug) or JTAG interfaces.

st-util can serve as a GDB server, bridging the hardware debugger to the arm-none-eabi-gdb command-line debugger. This setup allows for powerful debugging features, including setting breakpoints, inspecting registers, examining memory contents, and single-stepping through assembly code. Integrated Development Environments (IDEs) such as STM32CubeIDE or Visual Studio Code with the PlatformIO extension can provide a more streamlined and integrated development experience, offering graphical interfaces for configuration, building, and debugging.

The success of a bare-metal project is profoundly dependent on a well-configured toolchain and a thorough understanding of the linker's function in memory mapping. In the absence of an RTOS or higher-level OS managing memory, the developer directly assumes the role of the memory manager. This means that debugging capabilities, particularly hardware-assisted debugging via JTAG/SWD, are indispensable for diagnosing low-level issues. Standard printf debugging is often limited or can introduce timing issues in bare-metal environments, necessitating direct hardware inspection. Therefore, proficiency with the debugger and the ability to interpret the linker map file become as critical as the coding skills themselves.

6.2. Debugging and Testing Strategies for Embedded OS

Debugging and testing for a custom embedded OS involve a systematic approach. Iterative development and testing of individual peripheral drivers are essential before their integration into the complete OS. This modular testing helps isolate and resolve issues at an early stage.

The linker map file (.map), generated during the build process, is an invaluable tool for analyzing memory usage (both Flash and RAM) and identifying potential overflows or inefficiencies. This proactive analysis is crucial for optimizing the memory footprint of the custom OS, ensuring it fits within the tight memory constraints of the STM32F103C8T6 and remains stable.

Memory usage analysis using the linker map is not merely a post-build check but an iterative optimization step. As new features are incorporated, their impact on RAM and Flash consumption must be continuously monitored. The linker map provides a detailed breakdown of the size of the .text, .data, and .bss sections, as well as the allocated stack and heap space. This allows for early detection of potential memory exhaustion. When a new feature, such as Long File Name (LFN) support in FatFs or a new game, is added, its memory impact is immediately visible in the map file. This information is vital for making informed decisions about whether a feature is feasible or requires optimization. If memory becomes constrained, the map file helps pinpoint the largest memory consumers, enabling targeted optimization efforts, such as reducing buffer sizes, employing more compact data structures, or applying specific compiler optimization flags. This proactive approach to memory analysis is central to ensuring the OS remains lean, efficient, and stable throughout its development lifecycle.

For basic debugging and logging messages, a serial debugging interface (e.g., via UART) can be implemented. This requires a minimal UART driver and a custom 

__io_putchar function to redirect printf output to the serial port.

6.3. Code Optimization for Performance and Memory Footprint

Code optimization is a continuous process throughout the development of a custom embedded OS. Compiler optimization flags play a significant role; for instance, using -Os (optimize for size) with ARM-GCC can drastically reduce the generated binary size.

Beyond compiler optimizations, efficient C programming practices are paramount. This includes avoiding unnecessary global variables, utilizing appropriate fixed-width integer data types (e.g., uint8_t instead of int where memory is critical), and optimizing loops and algorithms for both speed and memory efficiency.

As discussed in Section 2.3, employing custom memory allocators, specifically memory pools, can prevent heap fragmentation and provide deterministic allocation times, which is vital for the real-time characteristics of the OS. These custom allocators ensure that memory is managed predictably, mitigating the risks associated with standard dynamic memory allocation in a non-MMU environment.

7. Conclusion

This report has detailed the comprehensive design and implementation strategy for a custom operating system on the STM32F103C8T6 microcontroller. The architectural choices, driven by the microcontroller's resource constraints and the specific functional requirements, underscore a bare-metal approach augmented with cooperative multitasking.

7.1. Summary of Custom OS Capabilities

The custom OS successfully integrates core components essential for a functional embedded system. It employs a cooperative multitasking paradigm, managed by a SysTick-driven scheduler and optimized context switching via PendSV, to ensure responsiveness and efficient CPU utilization. Robust memory management strategies, including careful linker script partitioning and the consideration of custom memory pools, address the tight Flash and SRAM constraints.

A sophisticated multi-stage bootloader has been designed, enabling the OS to reside in internal flash while also supporting the crucial capability of booting from an SD card if the OS image size exceeds the internal flash capacity. This mechanism effectively incorporates In-Application Programming (IAP) for flexible firmware updates.

Peripheral drivers have been meticulously implemented for the SD card (utilizing SPI communication and FatFs for file system access), 5 tactile buttons (with robust software debouncing and interrupt-driven handling), a 128x64 I2C LCD (with a lean driver and graphics primitives), and a piezo speaker (driven by PWM with DMA for efficient audio streaming). The file navigation system leverages the FatFs integration to provide intuitive browsing of audio and game files. A data-driven approach for basic game execution allows for dynamic content loading from the SD card, transforming the microcontroller into a versatile platform for interchangeable entertainment.

7.2. Future Enhancements and Expansion Possibilities

The foundation laid by this custom OS offers numerous avenues for future enhancements and expansion. More sophisticated task scheduling algorithms, such as priority-based cooperative scheduling or a full-fledged preemptive RTOS (like FreeRTOS, if resource overhead can be managed), could be explored to enhance real-time determinism. For audio, integrating support for more advanced file formats (e.g., MP3 playback) would necessitate the addition of external audio DACs and dedicated decoding libraries, as the current PWM output has inherent fidelity limitations. Supporting more complex game types would require optimizing graphics routines, potentially leveraging hardware acceleration features of more advanced microcontrollers, or exploring advanced rendering techniques tailored for monochrome displays.

Adding network connectivity (e.g., Ethernet, Wi-Fi) would open possibilities for over-the-air (OTA) firmware updates, remote control, and networked gaming. Implementing a simple command-line interface (CLI) over UART could provide advanced debugging capabilities and system control. Furthermore, integrating comprehensive power management features would be crucial for extending battery life in portable applications, allowing the OS to intelligently manage power states of peripherals and the CPU. The modular architecture established in this design facilitates these future enhancements, allowing for incremental development and adaptation to evolving requirements.
