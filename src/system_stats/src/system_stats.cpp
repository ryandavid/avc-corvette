#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <sys/sysctl.h>

#ifdef __APPLE__
#include <mach/vm_statistics.h>
#include <mach/mach_types.h>
#include <mach/mach_init.h>
#include <mach/mach_host.h>
#else
#include "sys/types.h"
#include "sys/sysinfo.h"
#include <stdlib.h>
#endif // __APPLE__

#ifdef WITH_CUDA
extern "C"
{
    #include <nvml.h>
}
#endif

#include "system_stats/system_stats.h"

#ifdef __APPLE__
std::string get_hostname() {
    size_t hostname_len = 128;
    char hostname[hostname_len];
    sysctlbyname("kern.hostname", &hostname, &hostname_len, NULL, 0);

    return std::string(hostname);
}

void get_sys_load(double* load_1min, double* load_5min, double* load_15min) {
    struct loadavg loadinfo;
    int mib[2];

    mib[0] = CTL_VM;
    mib[1] = VM_LOADAVG;

    size_t size = sizeof(loadinfo);
    int ret = sysctl(mib, 2, &loadinfo, &size, NULL, 0);

    *load_1min = (double)loadinfo.ldavg[0] / loadinfo.fscale;
    *load_5min = (double)loadinfo.ldavg[1] / loadinfo.fscale;
    *load_15min = (double)loadinfo.ldavg[2] / loadinfo.fscale;
}

int64_t get_total_mem() {
    int64_t total_mem;
    int mib[2];

    mib[0] = CTL_HW;
    mib[1] = HW_MEMSIZE;

    size_t size = sizeof(int64_t);
    int ret = sysctl(mib, 2, &total_mem, &size, NULL, 0);

    return total_mem;
}


// http://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
int64_t get_mem_used() {
    vm_size_t page_size;
    mach_port_t mach_port;
    mach_msg_type_number_t count;
    vm_statistics64_data_t vm_stats;

    mach_port = mach_host_self();
    count = sizeof(vm_stats) / sizeof(natural_t);
    if (KERN_SUCCESS == host_page_size(mach_port, &page_size) &&
        KERN_SUCCESS == host_statistics64(mach_port, HOST_VM_INFO,
                                        (host_info64_t)&vm_stats, &count)) {
        return ((int64_t)vm_stats.active_count +
               (int64_t)vm_stats.inactive_count +
               (int64_t)vm_stats.wire_count) *  (int64_t)page_size;
    }

    return 0;
}

#else
std::string get_hostname() {
    size_t length = 128;
    char scratch[length];

    gethostname(&scratch[0], length);
    return std::string(scratch);
}

void get_sys_load(double* load_1min, double* load_5min, double* load_15min) {
    int num_elements = 3;
    double averages[3];

    if (getloadavg(&averages[0], num_elements) != -1) {
        *load_1min = averages[0];
        *load_5min = averages[1];
        *load_15min = averages[2];
    } else {
        *load_1min = 0;
        *load_5min = 0;
        *load_15min = 0;
    }
}

// http://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
int64_t get_total_mem() {
    struct sysinfo memInfo;

    sysinfo (&memInfo);
    long long total_phys_mem = memInfo.totalram;

    return total_phys_mem * memInfo.mem_unit;
}

int64_t get_mem_used() {
    struct sysinfo memInfo;

    sysinfo (&memInfo);
    long long phys_mem_used = memInfo.totalram - memInfo.freeram;
    //Multiply in next statement to avoid int overflow on right hand side...
    return phys_mem_used * memInfo.mem_unit;
}

#endif  // __APPLE__

