#include "BaseControl_Define.h"
#include "hw_intf.h"
#include "stepmotor.h"
#include "encoder.h"

#define STEPMOTOR_PWM_DUTY  				50

#define ENCODER_COUNTER_MODE_UP  			0
#define ENCODER_COUNTER_MODE_DOWN  			1

encoder_handle_t encoder_left_handle = NULL;
encoder_handle_t encoder_right_handle = NULL;
stepmotor_handle_t motor_left_handle = NULL;
stepmotor_handle_t motor_right_handle = NULL;

err_code_t periph_motor_init(void)
{
	err_code_t err_ret;

	/* Initialize left motor */
	motor_left_handle = stepmotor_init();
	if (motor_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	stepmotor_cfg_t leftmotor_cfg = {
		.dir = 0,
		.duty = 0,
		.freq_hz = 0,
		.set_pwm_duty = hw_intf_leftmotor_set_pwm_duty,
		.set_pwm_freq = hw_intf_leftmotor_set_pwm_freq,
		.start_pwm = hw_intf_leftmotor_start,
		.stop_pwm = hw_intf_leftmotor_stop,
		.set_dir = hw_intf_leftmotor_set_dir,
	};
	err_ret = stepmotor_set_config(motor_left_handle, leftmotor_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = stepmotor_config(motor_left_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	/* Initialize right motor */
	motor_right_handle = stepmotor_init();
	if (motor_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	stepmotor_cfg_t rightmotor_cfg = {
		.dir = 0,
		.duty = 0,
		.freq_hz = 0,
		.set_pwm_duty = hw_intf_rightmotor_set_pwm_duty,
		.set_pwm_freq = hw_intf_rightmotor_set_pwm_freq,
		.start_pwm = hw_intf_rightmotor_start,
		.stop_pwm = hw_intf_rightmotor_stop,
		.set_dir = hw_intf_rightmotor_set_dir
	};
	err_ret = stepmotor_set_config(motor_right_handle, rightmotor_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = stepmotor_config(motor_right_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	stepmotor_set_pwm_freq(motor_left_handle, 0);
	stepmotor_set_pwm_duty(motor_left_handle, STEPMOTOR_PWM_DUTY);
	stepmotor_set_dir(motor_left_handle, MOTORLEFT_DIR_FORWARD);
	stepmotor_start(motor_left_handle);

	stepmotor_set_pwm_freq(motor_right_handle, 0);
	stepmotor_set_pwm_duty(motor_right_handle, STEPMOTOR_PWM_DUTY);
	stepmotor_set_dir(motor_right_handle, MOTORRIGHT_DIR_FORWARD);
	stepmotor_start(motor_right_handle);

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_left_start(void)
{
	if (motor_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err;

	err = stepmotor_start(motor_left_handle);
	if (err != ERR_CODE_SUCCESS)
	{
		return err;
	}

	return ERR_CODE_SUCCESS;
}
err_code_t periph_motor_left_stop(void)
{
	if (motor_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err;

	err = stepmotor_stop(motor_left_handle);
	if (err != ERR_CODE_SUCCESS)
	{
		return err;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_left_set_speed(float speed)
{
	if (motor_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	if (speed < 0)
	{
		stepmotor_set_dir(motor_left_handle, MOTORLEFT_DIR_BACKWARD);
		encoder_set_mode(encoder_left_handle, ENCODER_COUNTER_MODE_DOWN);
		stepmotor_set_pwm_freq(motor_left_handle, (uint32_t)(-speed * VEL2FREQ));
		stepmotor_set_pwm_duty(motor_left_handle, STEPMOTOR_PWM_DUTY);
	}
	else
	{
		stepmotor_set_dir(motor_left_handle, MOTORLEFT_DIR_FORWARD);
		encoder_set_mode(encoder_left_handle, ENCODER_COUNTER_MODE_UP);
		stepmotor_set_pwm_freq(motor_left_handle, (uint32_t)(speed * VEL2FREQ));
		stepmotor_set_pwm_duty(motor_left_handle, STEPMOTOR_PWM_DUTY);
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_right_start(void)
{
	if (motor_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err;

	err = stepmotor_start(motor_right_handle);
	if (err != ERR_CODE_SUCCESS)
	{
		return err;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_right_stop(void)
{
	if (motor_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err;

	err = stepmotor_stop(motor_right_handle);
	if (err != ERR_CODE_SUCCESS)
	{
		return err;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_right_set_speed(float speed)
{
	if (motor_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	if (speed < 0)
	{
		stepmotor_set_dir(motor_right_handle, MOTORLEFT_DIR_BACKWARD);
		encoder_set_mode(encoder_right_handle, ENCODER_COUNTER_MODE_DOWN);
		stepmotor_set_pwm_freq(motor_right_handle, (uint32_t)(-speed * VEL2FREQ));
		stepmotor_set_pwm_duty(motor_right_handle, STEPMOTOR_PWM_DUTY);
	}
	else
	{
		stepmotor_set_dir(motor_right_handle, MOTORLEFT_DIR_FORWARD);
		encoder_set_mode(encoder_right_handle, ENCODER_COUNTER_MODE_UP);
		stepmotor_set_pwm_freq(motor_right_handle, (uint32_t)(speed * VEL2FREQ));
		stepmotor_set_pwm_duty(motor_right_handle, STEPMOTOR_PWM_DUTY);
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_encoder_init(void)
{
	err_code_t err_ret;

	/* Initialize left encoder */
	encoder_left_handle = encoder_init();
	if (encoder_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	encoder_cfg_t left_encoder_cfg = {
		.max_reload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV,
		.start = hw_intf_left_encoder_start,
		.stop = hw_intf_left_encoder_stop,
		.set_counter = hw_intf_left_encoder_set_counter,
		.get_counter = hw_intf_left_encoder_get_counter,
		.set_mode = hw_intf_left_encoder_set_mode,
	};
	err_ret = encoder_set_config(encoder_left_handle, left_encoder_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = encoder_config(encoder_left_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	/* Initialize right encoder */
	encoder_right_handle = encoder_init();
	if (encoder_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	encoder_cfg_t right_encoder_cfg = {
		.max_reload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV,
		.start = hw_intf_right_encoder_start,
		.stop = hw_intf_right_encoder_stop,
		.set_counter = hw_intf_right_encoder_set_counter,
		.get_counter = hw_intf_right_encoder_get_counter,
		.set_mode = hw_intf_right_encoder_set_mode
	};
	err_ret = encoder_set_config(encoder_right_handle, right_encoder_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = encoder_config(encoder_right_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	encoder_set_mode(encoder_left_handle, ENCODER_COUNTER_MODE_UP);
	encoder_set_mode(encoder_right_handle, ENCODER_COUNTER_MODE_UP);

	encoder_start(encoder_left_handle);
	encoder_start(encoder_right_handle);

	return ERR_CODE_SUCCESS;
}

err_code_t periph_encoder_left_get_tick(int32_t *tick)
{
	if (encoder_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint32_t temp;

	encoder_get_value(encoder_left_handle, &temp);
	encoder_set_value(encoder_left_handle, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);

	*tick = temp - NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2;

	return ERR_CODE_SUCCESS;
}

err_code_t periph_encoder_right_get_tick(int32_t *tick)
{
	if (encoder_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint32_t temp;

	encoder_get_value(encoder_right_handle, &temp);
	encoder_set_value(encoder_right_handle, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);

	*tick = temp - NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2;

	return ERR_CODE_SUCCESS;
}
