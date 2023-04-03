/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 4: Cryptography True
* Random Number Generation Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
 * Macros
 ********************************************************************************/
/* Macro for the maximum value of the random number generated in bits */
#define ASCII_7BIT_MASK                 (0x7F)

#define PASSWORD_LENGTH                 (8u)
#define ASCII_VISIBLE_CHARACTER_START   (33u)
#define ASCII_RETURN_CARRIAGE           (0x0D)

#define SCREEN_HEADER "\r\n__________________________________________________"\
                      "____________________________\r\n*\tPSoC 4 Cryptography: "\
                      "True Random Number Generation\r\n*\r\n*\tThis code example "\
                      "demonstrates generating a One-Time Password (OTP)\r\n*\tusing the"\
                      " True Random Number generation feature of PSoC 4\r\n*\t"\
                      "cryptography block\r\n*\r\n*\tUART Terminal Settings\tBaud Rate:"\
                      "115200 bps 8N1 \r\n*"\
                      "\r\n__________________________________________________"\
                      "____________________________\r\n\n"

#define SCREEN_HEADER1 "\r\n================================================="\
                       "=============================\r\n"

/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
#define CLEAR_SCREEN         "\x1b[2J\x1b[;H"

#define MAX_TRNG_BIT_SIZE     (32UL)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void generate_password();
uint8_t check_range(uint8_t value);

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* UART configuration structure */
cy_stc_scb_uart_context_t uart_context;

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *   1. Initializes the BSP.
 *   2. Initialize and enable UART block.
 *   3. Generate random number and convert it to a alpha-numeric password.
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Enable the Crypto block. */
    result = Cy_Crypto_Enable(CRYPTO);

    /* Crypto init failed. Stop program execution */
    if (result != CY_CRYPTO_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize and enable UART block */
    result = Cy_SCB_UART_Init(UART_HW, &UART_config, &uart_context);

    /* SCB UART init failed. Stop program execution */
    if (result != CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(UART_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    Cy_SCB_UART_PutString(UART_HW, CLEAR_SCREEN);

    Cy_SCB_UART_PutString(UART_HW, SCREEN_HEADER);
    Cy_SCB_UART_PutString(UART_HW, "Press Enter key to generate OTP\r\n\n");

    for (;;)
    {
        /* Check if 'Enter' key was pressed */
        while (Cy_SCB_UART_Get(UART_HW) != ASCII_RETURN_CARRIAGE)
        {
        };
        /* Generate a password of 8 characters in length */
        generate_password();
    }
}

/*******************************************************************************
 * Function Name: generate_password
 ********************************************************************************
 * Summary:
 *  This function generates a 8 character long password.
 *
 * Parameters:
 *  None
 *
 * Return
 *  void
 *
 *******************************************************************************/
void generate_password()
{
    int8_t index;
    uint8_t temp_value = 0;
    uint8_t bit_position;
    cy_en_crypto_status_t crypto_status;

    /* Generated random number */
    uint32_t rnd_num = 0;

    /* Array to hold the generated password. Array size is inclusive of
       string NULL terminating character */
    uint8_t password[PASSWORD_LENGTH + 1] = {0};

    for (index = 0; index < PASSWORD_LENGTH;)
    {
        /* Generate a random 32 bit number*/
        crypto_status = Cy_Crypto_Trng(CRYPTO, MAX_TRNG_BIT_SIZE, &rnd_num);

        if (crypto_status != CY_CRYPTO_SUCCESS)
        {
            CY_ASSERT(0);
        }

        for (bit_position = 0; bit_position < 32; bit_position += 8)
        {
            /* extract byte from the bit position offset 0, 8, 16, and 24. */
            temp_value = ((rnd_num >> bit_position) & ASCII_7BIT_MASK);
            temp_value = check_range(temp_value);
            password[index++] = temp_value;
        }
    }

    /* Terminate the password with end of string character */
    password[index] = '\0';

    /* Display the generated password on the UART Terminal */
    Cy_SCB_UART_PutString(UART_HW, "\nOne-Time Password:\r\n");
    Cy_SCB_UART_PutString(UART_HW, (char *)password);
    Cy_SCB_UART_PutString(UART_HW, "\r\n\nPress Enter key to generate new OTP\r\n");
    Cy_SCB_UART_PutString(UART_HW, SCREEN_HEADER1);
}

/*******************************************************************************
 * Function Name: check_range
 ********************************************************************************
 * Summary:
 *  This function checks if the generated random number is in the range
 *  of alpha-numeric, special characters ASCII codes. If not, convert to
 *  that range.
 *
 * Parameters:
 *  uint8_t value - generated random number
 *
 * Return
 *  Updated value of the random number
 *
 *******************************************************************************/
uint8_t check_range(uint8_t value)
{
    if (value < ASCII_VISIBLE_CHARACTER_START)
    {
        value += ASCII_VISIBLE_CHARACTER_START;
    }
    return value;
}

/* [] END OF FILE */
