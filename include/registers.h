/**
* @file     registers.h
* @brief    This file defines all the registers that are used in data model
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     26/09/2018
*/

#ifndef __REGISTER_H__
#define __REGISTER_H__

#define COILS_ON    1
#define COILS_OFF   0

#define __ON__      0xFF00
#define __OFF__     0x0000

#define NB_DISCRETES_INPUT_REGISTERS 10
#define NB_COILS_REGISTERS           10
#define NB_INPUT_REGISTERS           10
#define NB_HOLDING_REGISTERS         65500

typedef struct _Data_Model{
    
    uint8_t  discretes_input[NB_DISCRETES_INPUT_REGISTERS]; /* Read-Only */
    uint8_t  coils[NB_COILS_REGISTERS];  /* Read-Write */
    uint16_t input_registers[NB_INPUT_REGISTERS]; /* Read-Only */
    uint16_t holding_registers[NB_HOLDING_REGISTERS]; /* Read-Write */

    pthread_mutex_t *data_model_mutex;

}Data_Model;

typedef struct __4paired_register_region {
    uint16_t *word1;
    uint16_t *word2;
    uint16_t *word3;
    uint16_t *word4;
} _4paired_register_region;

typedef struct __2paired_register_region {
    uint16_t *word1;
    uint16_t *word2;
} _2paired_register_region;


void copy_4paired_register_region(_4paired_register_region *dest, _4paired_register_region *src);
void update_4paired_register_region(_4paired_register_region *dest, uint64_t value);
uint64_t combine_4paired_register_region(_4paired_register_region *src);
int compare_4paired_register_region(_4paired_register_region *region1, _4paired_register_region *region2);

void copy_2paired_register_region(_2paired_register_region *dest, _2paired_register_region *src);
void update_2paired_register_region(_2paired_register_region *dest, uint32_t value);
uint32_t combine_2paired_register_region(_2paired_register_region *src);
int compare_2paired_register_region(_2paired_register_region *region1, _2paired_register_region *region2);


struct __register;

typedef struct _register_ops {

    int (*init)(struct __register *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
    int (*deinit)(struct __register *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);

    int (*read_coils)(struct __register *, uint8_t *, uint8_t *, uint16_t *);
    int (*read_discrete_inputs)(struct __register *, uint8_t *, uint8_t *, uint16_t *);
    int (*read_holding_registers)(struct __register *, uint8_t *, uint8_t *, uint16_t *);
    int (*read_input_registers)(struct __register *, uint8_t *, uint8_t *, uint16_t *);
    int (*write_single_coil)(struct __register *, uint8_t *, uint8_t *, uint16_t *);
    int (*write_single_register)(struct __register *, uint8_t *, uint8_t *, uint16_t *);
    int (*write_multiple_registers)(struct __register *, uint8_t *, uint8_t *, uint16_t *);

} register_ops;

typedef struct _step_info {
    struct {
        uint16_t *begin;
        uint16_t *number;
        uint16_t *count;
    } cycle;

    struct {
        uint16_t *status;
        _4paired_register_region current_region;
        _4paired_register_region voltage_region;
        _4paired_register_region power_region;
        _4paired_register_region sampling_interval_region;
        _4paired_register_region resistance_region;
    } common;

    struct {
        
        _4paired_register_region last_logic_holding_time_region;
        _4paired_register_region last_logic_voltage_region;
        _4paired_register_region last_logic_current_region;
        _4paired_register_region last_logic_capacity_region;
        _4paired_register_region last_logic_energy_region;
        _4paired_register_region last_logic_temperature_region;

        uint16_t *operator;
        uint16_t *step_index;
        uint16_t *behavior;
        _4paired_register_region operand2_region;
        uint16_t *next; /* next step id */

    } goingto;

    struct {
        // power
        // jump_time
        // uppper voltage
    } cpc;

    struct {
        // power
        // jump_time
        // lower voltage
    } cpd;

    struct {
        // resistance
        // jump_time
        // lower voltage
    } crd;

} step_info;

typedef struct _step_time_parameters {
    _4paired_register_region established_time_region;
    _4paired_register_region abolished_time_region;
} step_time_parameters;

typedef struct _jump_condiction_parameters {
    uint16_t *enable;
    _4paired_register_region time_region;
    _4paired_register_region voltage_region;
    _4paired_register_region current_region;
    _4paired_register_region capacity_region;
    _4paired_register_region energy_region;
} jump_condiction_parameters;  

typedef struct _exception_handling {
    uint16_t *exception_enable1;
    uint16_t *exception_enable2;
    uint16_t *exception_enable3;
} exception_handling;

typedef struct _tolerances {
    _4paired_register_region common_node_voltage_tolerance_region;  // in 0.1 mV
    _4paired_register_region constant_voltage_stage_confirmation_tolerance_region; /* in mV */
    _4paired_register_region constant_voltage_stage_current_drop_confirmation_tolerance_region; /* in permillage */
} tolerances;

typedef struct _formation_step_parameters {
    uint16_t *step_definition;
    uint16_t *step_type;
    jump_condiction_parameters jump;
    step_time_parameters timing;
    step_info info;
    struct {
        _4paired_register_region accumulated_logic_capacity_region;
        _4paired_register_region accumulated_step_logic_capacity_region;
    } temp;
} formation_step_parameters;

typedef struct _calibration_parameters {

    uint16_t *enable;
    uint16_t *current_source_level;
    uint16_t *kb_save_enable;
    _4paired_register_region calibration_current_region; /* User defined */
    _4paired_register_region calibration_voltage_region; /* User defined */
    _4paired_register_region calibration_sample_current_region; /* From AD */
    _4paired_register_region calibration_sample_voltage_region; /* From AD */

} calibration_parameters;

typedef struct _step_parameter_pointers {
    uint16_t *nb_1_655;         /*  1x655 */
    uint16_t *nb_656_1310;      /*  2x655 */
    uint16_t *nb_1311_1965;     /*  3x655 */
    uint16_t *nb_1966_2620;     /*  4x655 */
    uint16_t *nb_2621_3275;     /*  5x655 */
    uint16_t *nb_3276_3930;     /*  6x655 */
    uint16_t *nb_3931_4585;     /*  7x655 */
    uint16_t *nb_4586_5240;     /*  8x655 */
    uint16_t *nb_5241_5895;     /*  9x655 */
    uint16_t *nb_5896_6550;     /* 10x655 */
    uint16_t *nb_6551_7205;     /* 11x655 */
    uint16_t *nb_7206_7860;     /* 12x655 */
} step_parameter_pointers;

typedef struct _logical_channel_register {
    uint16_t *status;
    uint16_t *mode;
    uint16_t *action;
    uint16_t *binding;
    uint16_t *nb_binding;
    uint16_t *exception_code;
    uint16_t *exception_message;
    uint16_t *step_pointer;
    uint16_t *step_quantity;

    struct {

        _4paired_register_region holding_time_reference_region;
        _4paired_register_region holding_time_region;
        _4paired_register_region left_boundary_time_region;
        _4paired_register_region right_boundary_time_region;

        _4paired_register_region last_logic_holding_time_region;
        _4paired_register_region last_logic_voltage_region;
        _4paired_register_region last_logic_current_region;
        _4paired_register_region last_accumulated_step_logic_capacity_region;
        _4paired_register_region last_logic_energy_region;
        _4paired_register_region last_logic_temperature_region;

        _4paired_register_region last_sampling_record_time_region;
        _4paired_register_region jump_time_reference_region;
        _4paired_register_region ready_left_boundary_time_region;
        _4paired_register_region ready_right_boundary_time_region;

        _4paired_register_region last_accumulated_logic_capacity_sampling_time_region;

    } temp;

    // for timming
    _4paired_register_region maximum_source_current_region;

    _4paired_register_region logic_voltage_region;
    _4paired_register_region logic_current_region;
    
    _4paired_register_region accumulated_logic_capacity_region;
    _4paired_register_region accumulated_step_logic_capacity_region;
    
    _4paired_register_region logic_energy_region;

    struct {
        _4paired_register_region step_turnaround_delay_region;
        uint16_t *calc_accumulated_capacity_time_interval;
    } timing;
    
    tolerances tolerance;
    formation_step_parameters step_param;
    calibration_parameters calibration_param;
    step_parameter_pointers step_param_pointers;

} logical_channel_register;

typedef struct _physical_channel_register {
    uint16_t *status;
    uint16_t *ready;
    _4paired_register_region current_region;
    _4paired_register_region voltage_region; /*Sign Magnitude Representation*/
    uint16_t *current_source_level; /* 1 --- level1, 2 --- level2, 3 --- level3 */
    _4paired_register_region source_current_level_1_region;
    _4paired_register_region source_current_level_2_region;
    _4paired_register_region source_current_level_3_region;

    /* digital to analog converter */
    _4paired_register_region vi_region;
    _4paired_register_region vu_region;
    _4paired_register_region vu_bias_region;

    /* Io Expander */
    uint16_t *kz_h;
    uint16_t *kz_m;
    uint16_t *kz_l;
    uint16_t *jc;
    uint16_t *cf;
    uint16_t *ck;
    uint16_t *cled;
    uint16_t *fled;

    _4paired_register_region bv_region;
    _4paired_register_region cv_region;

    uint16_t *bell;
    uint16_t *rdry;
    uint16_t *led_ready;
    uint16_t *led_run;
    uint16_t *led_error;
    uint16_t *ac_loss;

    uint16_t *bound_relation;

    struct {
        uint16_t *c_vi_k_level_1;
        uint16_t *c_vi_k_level_2;
        uint16_t *c_vi_k_level_3;
        uint16_t *d_vi_k_level_1;
        uint16_t *d_vi_k_level_2;
        uint16_t *d_vi_k_level_3;
        _4paired_register_region c_vi_b_level1_region;
        _4paired_register_region c_vi_b_level2_region;
        _4paired_register_region c_vi_b_level3_region;
        _4paired_register_region d_vi_b_level1_region;
        _4paired_register_region d_vi_b_level2_region;
        _4paired_register_region d_vi_b_level3_region;

        uint16_t *c_cv_k_level_1;
        uint16_t *c_cv_k_level_2;
        uint16_t *c_cv_k_level_3;
        uint16_t *d_cv_k_level_1;
        uint16_t *d_cv_k_level_2;
        uint16_t *d_cv_k_level_3;
        _4paired_register_region c_cv_b_level1_region;
        _4paired_register_region c_cv_b_level2_region;
        _4paired_register_region c_cv_b_level3_region;
        _4paired_register_region d_cv_b_level1_region;
        _4paired_register_region d_cv_b_level2_region;
        _4paired_register_region d_cv_b_level3_region;

        uint16_t *bv_k;
        uint16_t *vu_k;
        _4paired_register_region bv_b_region;
        _4paired_register_region vu_b_region;
    } kb;

    struct {
        _4paired_register_region voltage_region;
        _4paired_register_region current_region;
    } sample;

    struct {
        _4paired_register_region last_vi_region;
        _4paired_register_region last_vu_region;

        uint16_t *last_kz_h;
        uint16_t *last_kz_m;
        uint16_t *last_kz_l;
        uint16_t *last_jc;
        uint16_t *last_cf;
        uint16_t *last_ck;
        uint16_t *last_cled;
        uint16_t *last_fled;
        uint16_t *last_bell;
        uint16_t *last_rdry;
        uint16_t *last_led_ready;
        uint16_t *last_led_run;
        uint16_t *last_led_error;
        uint16_t *last_ac_loss;

        _4paired_register_region last_unit_converted_bv_region;
        _4paired_register_region last_unit_converted_cv_region;

        _4paired_register_region last_current_region; /*Sign Magnitude Representation*/
        _4paired_register_region last_voltage_region; /*Sign Magnitude Representation*/

        uint16_t *last_current_source_level;

        _4paired_register_region unit_converted_bv_region;
        _4paired_register_region unit_converted_cv_region;

    } temp;

    // for misc
    _2paired_register_region device_host;
    uint16_t *device_host_save_enable;
} physical_channel_register;

typedef struct __register {
    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param; 
    pthread_mutex_t *log_mutex;
    uint8_t platform_endian;
    uint8_t cur_unit_identifier;
    Data_Model data_model[NB_LOGIC_CHANNEL + NB_STEP_PARAMETER_CHANNELS];
    register_ops ops;
    logical_channel_register logical_channel[NB_LOGIC_CHANNEL];
    physical_channel_register physical_channel[NB_PHYSICAL_CHANNEL];
} Register;

int register_init(Register *reg, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
int register_deinit(Register *reg, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
int read_coils(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length);
int read_discrete_inputs(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length);
int read_holding_registers(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length);
int read_input_registers(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length);
int write_single_coil(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length);
int write_single_register(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length);
int write_multiple_registers(Register *reg, uint8_t *PDU, uint8_t *output_pdu, uint16_t *output_length);


#endif  //__REGISTER_H__


