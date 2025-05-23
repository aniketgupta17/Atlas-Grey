#include<rtc.h>
 
const struct device *const rtc = DEVICE_DT_GET(DT_ALIAS(rtc));
 
int set_date_time(const struct device *rtc)
 {
   int ret = 0;
   struct rtc_time tm = {
     .tm_year = 2025 - 1900,
     .tm_mon = 11 - 1,
     .tm_mday = 17,
     .tm_hour = 4,
     .tm_min = 19,
     .tm_sec = 0,
   };
 
   ret = rtc_set_time(rtc, &tm);
   if (ret < 0) {
     printk("Cannot write date time: %d\n", ret);
     return ret;
   }
   return ret;
 }
 
int get_date_time(const struct device *rtc)
 {
   int ret = 0;
   struct rtc_time tm;
 
   ret = rtc_get_time(rtc, &tm);
   if (ret < 0) {
     printk("Cannot read date time: %d\n", ret);
     return ret;
   }
 
   printk("RTC date and time: %04d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900,
          tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
 
   return ret;
 }
 

 
