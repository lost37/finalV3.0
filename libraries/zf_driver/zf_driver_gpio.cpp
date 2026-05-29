#include "zf_driver_gpio.h"
#include "zf_driver_file.h"

void gpio_set_level(const char *path, uint8 dat)
{
    dat = dat + 0x30;

    file_write_dat(path, dat);
}



uint8 gpio_get_level(const char *path)
{
    uint8 dat;

    file_read_dat(path, &dat);

    if (dat == '0')
        return 0;
    if (dat == '1')
        return 1;

    return dat;
}
