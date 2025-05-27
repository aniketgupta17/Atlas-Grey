/**
 * @file rtc.h
 * @brief RTC header file for time management in Zephyr-based systems
 * 
 * Author: Abhinav  
 * Student ID: 48244178
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>

extern const struct device *const rtc;

 int set_date_time(const struct device *rtc);
 int get_date_time(const struct device *rtc);


