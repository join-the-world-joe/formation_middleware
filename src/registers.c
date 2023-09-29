/**
* @file     registers.c
* @brief    This file defines all the registers that are used in data model
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     26/09/2018
*/

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>

#include "channel.h"
#include "log_controller.h"
#include "tcp.h"
#include "registers.h"
#include "modbus_tcp.h"
#include "debug.h"
#include <arpa/inet.h>
#include "platform_endian.h"


int register_init(Register *reg, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param; 

    if(!reg || !_log_controller_ctx || !_log_controller_ioctl_param) {
        printf("register_init, incorrect parameters!\n");
        return -1;
    }

    reg->ops.init = register_init;
    reg->ops.deinit = register_deinit;

    reg->_log_controller_ctx = _log_controller_ctx;
    reg->_log_controller_ioctl_param = _log_controller_ioctl_param;

    reg->ops.read_coils = read_coils;  /* For function code 1 */
    reg->ops.read_discrete_inputs = read_discrete_inputs; /* For function code 2 */
    reg->ops.read_holding_registers = read_holding_registers; /* For function code 3 */
    reg->ops.read_input_registers = read_input_registers; /* For function code 4 */
    reg->ops.write_single_coil = write_single_coil; /* For function code 5 */
    reg->ops.write_single_register = write_single_register; /* For function code 6 */
    reg->ops.write_multiple_registers = write_multiple_registers; /* For function code 16 */

    return ret;
}


int register_deinit(Register *reg, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param;

    if(!reg || !_log_controller_ctx || !_log_controller_ioctl_param) {
        printf("register_deinit, incorrect parameters!\n");
        return -1;
    }

    // do nothing here !!!

    return ret;
}


/**
* @fn               read_coils
* @param[in]        APU
*                   Protocol Data Unit
* @param[in/out]    output_pdu
*                   Protocol Data Unit
* @param[in/out]    output_length
*                   length of output_pdu
* @return           =0, success; >0, modbus exception code
* @brief            For function code 0x01
*/
int read_coils(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length) 
{
    Register *_register = NULL;
    int ret = 0, i, mod, j, byte_offset, remain, shift;
    uint8_t *coils, unit_identifier, condiction1, condiction2;
    uint16_t starting_address = 0, quantity_of_outputs = 0, nb, reg_offset, nb_reg;
    uint16_t starting_address_hi, starting_address_lo, quantity_of_outputs_hi, quantity_of_outputs_lo;

    if(!PDU || !reg || !output_pdu || !output_length) {
        printf("read_coils incorrect parameters\n");
        return -1;
    }

    _register = reg;
    unit_identifier = reg->cur_unit_identifier;

    starting_address_hi = PDU[1] << 8; // Starting Address Hi
    starting_address_lo = PDU[2];      // Starting Address Lo
   
    quantity_of_outputs_hi =  PDU[3] << 8;  // Quantity of Outputs Hi
    quantity_of_outputs_lo = PDU[4];  // Quantity of Outputs Lo

    starting_address = starting_address_hi | starting_address_lo;
    quantity_of_outputs = quantity_of_outputs_hi | quantity_of_outputs_lo;

    condiction1 = starting_address >= (NB_COILS_REGISTERS);
    condiction2 = (starting_address + quantity_of_outputs) > NB_COILS_REGISTERS ;
    
    if(condiction1 || condiction2)
        return MODBUS_EXCEPTION_CODE02;

    nb = quantity_of_outputs / 8;
    mod = quantity_of_outputs % 8;

    if(mod > 0) 
        nb++;
 
    coils = _register->data_model[unit_identifier].coils;

    nb_reg = 0;
    
    //function code and byte count
    output_pdu[0] = MODBUS_FC_READ_COILS;
    output_pdu[1] = nb;

    //How many bytes we have to response to client
    // what are the contents that we will send to client
    // how to construct the contents 
    for(i=0, byte_offset = 2; i<nb; i++, byte_offset++) {

        output_pdu[byte_offset] = 0;

        reg_offset = i*8 + starting_address; 

        //printf("nb_reg = %d\n", nb_reg);

        if(quantity_of_outputs - nb_reg >= 8) {
            remain = 8;
            shift = 0;
        } else {
            remain = (quantity_of_outputs - nb_reg); 
            //shift = 8 - remain; // shift from zero to 7
            shift = 0;
            //printf("remain %d shift %d\n", remain, shift);
        }
         
        for(j=reg_offset; j<reg_offset+remain; j++, shift++) {

            nb_reg++; 

            if(nb_reg <= quantity_of_outputs) {
                if(coils[j] == 1) 
                    output_pdu[byte_offset] |= 1<<shift;
            }
        }
    }

    *output_length = nb + 2;

    return ret;
}


/**
* @fn               read_discrete_inputs
* @param[in]        APU
*                   Protocol Data Unit
* @param[in/out]    output_pdu
*                   Protocol Data Unit
* @param[in/out]    output_length
*                   length of output_pdu
* @return           =0, success; >0, modbus exception code
* @brief            For function code 0x02
*/
int read_discrete_inputs(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length)
{
    Register *_register = NULL;
    int ret = 0, i, j, k, mod, byte_offset;
    uint8_t nb, unit_identifier, *discretes_input, reg_val, condiction1, condiction2, remain, shift;
    uint16_t starting_address_hi, starting_address_lo, quantity_of_inputs_hi, quantity_of_inputs_lo;
    uint16_t starting_address, quantity_of_inputs, reg_offset, nb_reg;
    
    if(!PDU || !reg || !output_pdu || !output_length) {
        printf("read_discrete_inputs incorrect parameters\n");
        return -1;
    }

    _register = reg;
    unit_identifier = reg->cur_unit_identifier;

    starting_address_hi = PDU[1] << 8;  // Starting Address Hi
    starting_address_lo = PDU[2];       // Starting Address Lo

    quantity_of_inputs_hi = PDU[3] << 8;// Quantity of Inputs Hi
    quantity_of_inputs_lo = PDU[4];     // Quantity of Inputs Lo

    starting_address = starting_address_hi | starting_address_lo;
    quantity_of_inputs = quantity_of_inputs_hi | quantity_of_inputs_lo;

    //printf("starting_address = %hu, quantity_of_outputs = %hu\n", starting_address, quantity_of_inputs);

    condiction1 = starting_address >= (NB_DISCRETES_INPUT_REGISTERS);
    condiction2 = (starting_address + quantity_of_inputs) > NB_DISCRETES_INPUT_REGISTERS ;
        
    if(condiction1 || condiction2)
        return MODBUS_EXCEPTION_CODE02;

    nb = quantity_of_inputs / 8;
    mod = quantity_of_inputs % 8;

    if(mod > 0) 
        nb++;

    discretes_input = _register->data_model[unit_identifier].discretes_input;

    nb_reg = 0;
    
    //function code and byte count
    output_pdu[0] = MODBUS_FC_READ_DISCRETE_INPUTS;
    output_pdu[1] = nb;

    //How many bytes we have to response to client
    // what are the contents that we will send to client
    // how to construct the contents 
    for(i=0, byte_offset = 2; i<nb; i++, byte_offset++) {

        output_pdu[byte_offset] = 0;

        reg_offset = i*8 + starting_address; 

        if(quantity_of_inputs - nb_reg >= 8) {
            remain = 8;
            shift = 0;
        } else {
            remain = (quantity_of_inputs - nb_reg); 
            shift = 8 - remain; // shift from zero to 7
        }
         
        for(j=reg_offset; j<reg_offset+remain; j++, shift++) {

            nb_reg++; 

            if(nb_reg <= quantity_of_inputs) {
                if(discretes_input[j] == 1) 
                    output_pdu[byte_offset] |= 1<<shift;
            }
        }
    }

    *output_length = nb + 2;

    return ret;
}


/**
* @fn               read_holding_registers
* @param[in]        APU
*                   Protocol Data Unit
* @param[in/out]    output_pdu
*                   Protocol Data Unit
* @param[in/out]    output_length
*                   length of output_pdu
* @return           =0, success; >0, modbus exception code
* @brief            For function code 0x03
*/
int read_holding_registers(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length)
{
    Register *_register = NULL;
    int ret = 0, i, j, k, mod, byte_offset;
    uint8_t nb, unit_identifier, condiction1, condiction2, endian;
    uint16_t *holding_registers;
    uint16_t starting_address_hi, starting_address_lo, nb_of_registers_hi, nb_of_registers_lo;
    uint16_t starting_address, nb_of_registers, reg_val;

    if(!PDU || !reg || !output_pdu || !output_length) {
        printf("read_holding_register incorrect parameters\n");
        return -1;
    }

    _register = reg;
    unit_identifier = reg->cur_unit_identifier;

    starting_address_hi = PDU[1] << 8;  // Starting Address Hi
    starting_address_lo = PDU[2];       // Starting Address Lo

    nb_of_registers_hi = PDU[3] << 8;   // No. of Registers Hi
    nb_of_registers_lo = PDU[4];        // No. of Registers Lo

    starting_address = starting_address_hi | starting_address_lo;
    nb_of_registers = nb_of_registers_hi | nb_of_registers_lo;

    condiction1 = starting_address >= (NB_HOLDING_REGISTERS);
    condiction2 = (starting_address + nb_of_registers) > NB_HOLDING_REGISTERS ;
        
    if(condiction1 || condiction2)
        return MODBUS_EXCEPTION_CODE02;

    nb = nb_of_registers;

    endian = _register->platform_endian;

    holding_registers = _register->data_model[unit_identifier].holding_registers;
    
    //function code and byte count
    output_pdu[0] = MODBUS_FC_READ_HOLDING_REGISTERS;
    output_pdu[1] = nb*2;

    for(i=0; i<nb; i++) {
        reg_val = holding_registers[starting_address + i];
        if(_register->platform_endian == __LITTLE_ENDIAN__ ) {
            output_pdu[i*2+2] = (reg_val & 0xFF00)>>8;
            output_pdu[i*2+2 + 1] = (reg_val & 0xFF);
        } else {
            output_pdu[i*2+2] = reg_val;
        }
    }

    *output_length = 2*nb + 2;
 
    return ret;
}


/**
* @fn               read_input_registers
* @param[in]        APU
*                   Protocol Data Unit
* @param[in/out]    output_pdu
*                   Protocol Data Unit
* @param[in/out]    output_length
*                   length of output_pdu
* @return           =0, success; >0, modbus exception code
* @brief            For function code 0x04
*/
int read_input_registers(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length)
{
    Register *_register = NULL;
    int ret = 0, i, j, k, mod, byte_offset;
    uint8_t nb, unit_identifier, condiction1, condiction2, endian;
    uint16_t *input_registers;
    uint16_t starting_address_hi, starting_address_lo, quantity_of_input_reg_hi, quantity_of_input_reg_lo;
    uint16_t starting_address, quantity_of_input_reg, reg_offset, nb_reg, reg_val;

    if(!PDU || !reg || !output_pdu || !output_length) {
        printf("read_input_registers incorrect parameters\n");
        return -1;
    }

    _register = reg;
    unit_identifier = reg->cur_unit_identifier;

    starting_address_hi = PDU[1] << 8;  // Starting Address Hi
    starting_address_lo = PDU[2];       // Starting Address Lo

    quantity_of_input_reg_hi = PDU[3] << 8; // Quantity of Input Reg. Hi
    quantity_of_input_reg_lo = PDU[4];      // Quantity of Input Reg. Lo

    starting_address = starting_address_hi | starting_address_lo;
    quantity_of_input_reg = quantity_of_input_reg_hi | quantity_of_input_reg_lo;

    condiction1 = starting_address >= (NB_INPUT_REGISTERS);
    condiction2 = (starting_address + quantity_of_input_reg) > NB_INPUT_REGISTERS ;
        
    if(condiction1 || condiction2)
        return MODBUS_EXCEPTION_CODE02;

    nb = quantity_of_input_reg;

    //printf("nb = %d\n", nb);

    endian = _register->platform_endian;

    input_registers = reg->data_model[unit_identifier].input_registers;

    //function code and byte count
    output_pdu[0] = MODBUS_FC_READ_INPUT_REGISTERS;
    output_pdu[1] = nb*2;

    for(i=0; i<nb; i++) {

        reg_val = input_registers[starting_address + i];

        if(endian == __LITTLE_ENDIAN__ ) {
            output_pdu[i*2+2] = (reg_val & 0xFF00)>>8;
            output_pdu[i*2+2 + 1] = (reg_val & 0xFF);
        } else {
            output_pdu[i*2+2] = reg_val;
        }
    }

    *output_length = 2*nb + 2;

    return ret;
}


/**
* @fn               write_single_coil
* @param[in]        APU
*                   Protocol Data Unit
* @param[in/out]    output_pdu
*                   Protocol Data Unit
* @param[in/out]    output_length
*                   length of output_pdu
* @return           =0, success; >0, modbus exception code
* @brief            For function code 0x05
*/
int write_single_coil(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length) 
{
    Register *_register = NULL;
    int ret = 0, i, j, k, mod, byte_offset;
    uint8_t nb, unit_identifier, condiction1, condiction2, endian;
    uint8_t *coils;
    uint16_t output_address_hi, output_address_lo, output_value_hi, output_value_lo;
    uint16_t output_address, output_value, reg_offset, nb_reg, reg_val;

    if(!PDU || !reg || !output_pdu || !output_length) {
        printf("write_single_coil incorrect parameters\n");
        return -1;
    }

    _register = reg;
    unit_identifier = reg->cur_unit_identifier;

    output_address_hi = PDU[1] << 8;
    output_address_lo = PDU[2];

    output_value_hi = PDU[3] << 8;
    output_value_lo = PDU[4];

    output_address = output_address_hi | output_address_lo;
    output_value = output_value_hi | output_value_lo;

    condiction1 = output_address >= NB_COILS_REGISTERS;
    condiction2 = (output_value != __ON__ && output_value != __OFF__);

    printf("output_address_hi 0x%x lo 0x%x\n", output_address_hi, output_address_lo);
    printf("write_single_coil output_address 0x%x\n", output_address);

    if(condiction1) 
        return MODBUS_EXCEPTION_CODE02;

    if(condiction2)
        return MODBUS_EXCEPTION_CODE03;
 
    endian = _register->platform_endian;

    coils = _register->data_model[unit_identifier].coils;

    //function code and byte count
    output_pdu[0] = MODBUS_FC_WRITE_SINGLE_COIL;

    if(output_value == __OFF__) {
        // turn off
        coils[output_address] = COILS_OFF;
    } else if(output_value == __ON__) {
        // turn on
        coils[output_address] = COILS_ON;
    }

    if(endian == __LITTLE_ENDIAN__) {
        // Little-Endian
        output_pdu[1] = (output_address & 0xFF00)>>8;
        output_pdu[2] = (output_address & 0xFF);
        if(output_value == __ON__) {
            output_pdu[3] = (__ON__ & 0xFF00)>>8;
            output_pdu[4] = (__ON__ & 0xFF);
        } else {
            output_pdu[3] = 0;
            output_pdu[4] = 0;
        }
    } else {
        // Big-Endian
        output_pdu[1] = output_address;
        output_pdu[3] = output_value;
    }

    *output_length = 5;

    return ret;
}


/**
* @fn               write_single_register
* @param[in]        APU
*                   Protocol Data Unit
* @param[in/out]    output_pdu
*                   Protocol Data Unit
* @param[in/out]    output_length
*                   length of output_pdu
* @return           =0, success; >0, modbus exception code
* @brief            For function code 0x06
*/
int write_single_register(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length)
{
    
    uint8_t nb, unit_identifier, condiction1;
    uint16_t *holding_registers;
    int ret = 0, i, j, k, mod, byte_offset;
    uint16_t register_address_hi, register_address_lo, register_value_hi, register_value_lo;
    uint16_t register_address, register_value, reg_offset, nb_reg, reg_val;

    if(!PDU || !reg || !output_pdu || !output_length) {
        printf("write_single_coil incorrect parameters\n");
        return -1;
    }

    unit_identifier = reg->cur_unit_identifier;

    register_address_hi = PDU[1] << 8;
    register_address_lo = PDU[2];

    register_value_hi = PDU[3] << 8;
    register_value_lo = PDU[4];

    register_address = register_address_hi | register_address_lo;
    register_value = register_value_hi | register_value_lo;

    condiction1 = register_address >= (NB_HOLDING_REGISTERS);
    if(condiction1)
        return MODBUS_EXCEPTION_CODE02;

    holding_registers = reg->data_model[unit_identifier].holding_registers;

    printf("register_address = 0x%x, register_value = 0x%x\n", register_address, register_value);

    // function code
    output_pdu[0] = MODBUS_FC_WRITE_SINGLE_REGISTER;

    holding_registers[register_address] = register_value;

    output_pdu[1] = PDU[1];
    output_pdu[2] = PDU[2];
    output_pdu[3] = PDU[3];
    output_pdu[4] = PDU[4];

    *output_length = 5;

    return ret;
}


/**
* @fn               write_multiple_registers
* @param[in]        APU
*                   Protocol Data Unit
* @param[in/out]    output_pdu
*                   Protocol Data Unit
* @param[in/out]    output_length
*                   length of output_pdu
* @return           =0, success; >0, modbus exception code
* @brief            For function code 0x10
*/
int write_multiple_registers(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length)
{
    Register *_register = NULL;
    uint16_t *holding_registers;
    int ret = 0, i, j, k, mod, byte_offset;
    uint8_t nb, unit_identifier, byte_count;
    uint16_t register_address, register_value;
    uint16_t register_value_hi, register_value_lo;
    uint16_t starting_address, quantity_of_registers, reg_offset, nb_reg, reg_val;
    uint16_t starting_address_hi, starting_address_lo, quantity_of_registers_hi, quantity_of_registers_lo;
    
    if(!PDU || !reg || !output_pdu || !output_length) {
        printf("write_multiple_registers incorrect parameters\n");
        return -1;
    }

    _register = reg;
    unit_identifier = reg->cur_unit_identifier;

    starting_address_hi = PDU[1] << 8;
    starting_address_lo = PDU[2];

    quantity_of_registers_hi = PDU[3] << 8;
    quantity_of_registers_lo = PDU[4];

    byte_count = PDU[5];

    starting_address = starting_address_hi | starting_address_lo;
    quantity_of_registers = quantity_of_registers_hi | quantity_of_registers_lo;

    holding_registers = _register->data_model[unit_identifier].holding_registers;

    for(i=0; i<byte_count/2; i++) {

        register_address = starting_address + i;

        register_value_hi = PDU[6 + i*2];
        register_value_lo = PDU[6 + i*2 + 1];
        register_value = (register_value_hi << 8) | register_value_lo;

        holding_registers[register_address] = register_value;

    }

    output_pdu[0] = PDU[0];
    output_pdu[1] = PDU[1];
    output_pdu[2] = PDU[2];
    output_pdu[3] = PDU[3];
    output_pdu[4] = PDU[4];

    *output_length = 5; 

    return ret;

}

void copy_4paired_register_region(_4paired_register_region *dest, _4paired_register_region *src) 
{
    *(dest->word1) = *(src->word1);
    *(dest->word2) = *(src->word2);
    *(dest->word3) = *(src->word3);
    *(dest->word4) = *(src->word4);
}

void update_4paired_register_region(_4paired_register_region *dest, uint64_t value) 
{
    *(dest->word1) = value & 0xFFFF;
    *(dest->word2) = (value >> 16) & 0xFFFF;
    *(dest->word3) = (value >> 32) & 0xFFFF;
    *(dest->word4) = (value >> 48) & 0xFFFF;
}

uint64_t combine_4paired_register_region(_4paired_register_region *src)
{
    uint64_t temp = 0;

    temp |=  (uint64_t)(*(src->word1));
    temp |= ((uint64_t)(*(src->word2))) << 16;
    temp |= ((uint64_t)(*(src->word3))) << 32;
    temp |= ((uint64_t)(*(src->word4))) << 48;

    return temp;
}

int compare_4paired_register_region(_4paired_register_region *region1, _4paired_register_region *region2)
{
    uint16_t temp1, temp2;

    temp1 = *(region1->word1);
    temp2 = *(region2->word1);
    if(temp1 != temp2) {
        return 1;
    }

    temp1 = *(region1->word2);
    temp2 = *(region2->word2);
    if(temp1 != temp2) {
        return 1;
    }

    temp1 = *(region1->word3);
    temp2 = *(region2->word3);
    if(temp1 != temp2) {
        return 1;
    }

    temp1 = *(region1->word4);
    temp2 = *(region2->word4);
    if(temp1 != temp2) {
        return 1;
    }

    return 0;
}

void copy_2paired_register_region(_2paired_register_region *dest, _2paired_register_region *src) 
{
    *(dest->word1) = *(src->word1);
    *(dest->word2) = *(src->word2);
}

void update_2paired_register_region(_2paired_register_region *dest, uint32_t value) 
{
    *(dest->word1) = value & 0xFFFF;
    *(dest->word2) = (value >> 16) & 0xFFFF;
}

uint32_t combine_2paired_register_region(_2paired_register_region *src)
{
    uint32_t temp = 0;

    temp |=  (uint32_t)(*(src->word1));
    temp |= ((uint32_t)(*(src->word2))) << 16;

    return temp;
}

int compare_2paired_register_region(_2paired_register_region *region1, _2paired_register_region *region2)
{
    uint16_t temp1, temp2;

    temp1 = *(region1->word1);
    temp2 = *(region2->word1);
    if(temp1 != temp2) {
        return 1;
    }

    temp1 = *(region1->word2);
    temp2 = *(region2->word2);
    if(temp1 != temp2) {
        return 1;
    }

    return 0;
}


#if 0

void main(void)
{
    uint64_t number = 0x88FFFFFFFFFFFFFF;
    uint64_t temp = 0;
    uint16_t reg[4] = {0}, temp_reg[4];
    _4paired_register_region region, temp_region;

    region.word1 = &reg[0];
    region.word2 = &reg[1];
    region.word3 = &reg[2];
    region.word4 = &reg[3];

    temp_region.word1 = &temp_reg[0];
    temp_region.word2 = &temp_reg[1];
    temp_region.word3 = &temp_reg[2];
    temp_region.word4 = &temp_reg[3];

    update_4paired_register_region(&region, number);

    printf("reg[0] = 0x%x\n", reg[0]);
    printf("reg[1] = 0x%x\n", reg[1]);
    printf("reg[2] = 0x%x\n", reg[2]);
    printf("reg[3] = 0x%x\n", reg[3]);

    temp = combine_4paired_register_region(&region);
    printf("temp %lu\n", temp);

    copy_4paired_register_region(&temp_region, &region);

    temp = combine_4paired_register_region(&temp_region);
    printf("temp %lu\n", temp);
}


#endif

int do_save_data_model()
{
    return 0;
}

int do_reload_data_model()
{
    return 0;
}

int clear_historical_data_model()
{
    return 0;
}

