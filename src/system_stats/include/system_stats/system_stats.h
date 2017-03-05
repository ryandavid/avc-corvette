#ifndef SYSTEM_STATS_H
#define SYSTEM_STATS_H


std::string get_hostname();
void get_sys_load(double* load_1min, double* load_5min, double* load_15min);
int64_t get_total_mem();
int64_t get_mem_used();

#endif  // SYSTEM_STATS_H