#ifndef SLIMENRF_SYSTEM
#define SLIMENRF_SYSTEM

#define STORED_TRACKERS 1
#define STORED_ADDR_0 2

void sys_write(uint16_t id, void *ptr, const void *data, size_t len);
void sys_read(uint16_t id, void *data, size_t len);

#endif