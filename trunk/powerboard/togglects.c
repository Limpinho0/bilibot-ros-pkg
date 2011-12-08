
/* hello-ftdi.c: flash LED connected between CTS and GND.
   This example uses the libftdi API.
   Minimal error checking; written for brevity, not durability. */

#include <stdio.h>
#include <ftdi.h>

#define LED 0x08  /* CTS (brown wire on FTDI cable) */


int main()
{
    unsigned char c = 0;
    struct ftdi_context ftdic;

    /* Initialize context for subsequent function calls */
    ftdi_init(&ftdic);

    /* Open FTDI device based on FT232R vendor & product IDs */
    if(ftdi_usb_open(&ftdic, 0x0403, 0x6001) < 0) {
        puts("Can't open device");
        return 1;
    }

    /* Enable bitbang mode with a single output line */
//     ftdi_enable_bitbang(&ftdic, LED);
    if (ftdi_set_bitmode(&ftdic, LED, BITMODE_BITBANG) < 0)
        puts( "Can't enable bitbang");

    /* Endless loop: invert LED state, write output, pause 1 second */
//     for(;;) {
        ftdi_write_data(&ftdic, &c, 1); //low
        sleep(1);
        c ^= LED;
        ftdi_write_data(&ftdic, &c, 1);//high
        sleep(1);
        c ^= LED;
        ftdi_write_data(&ftdic, &c, 1);//low
//     }
//     ftdi_set_bitmode(&ftdic, LED, 0x00);
    if(ftdi_disable_bitbang(&ftdic) < 0)
      puts("Can't disable bitbang");
      
//     if(ftdi_usb_reset(&ftdic) < 0)
//       puts("Can't reset device");
    if(ftdi_usb_close(&ftdic)<0)
      puts("Can't close device");
    ftdi_deinit(&ftdic);
}


// In order to prevent the requirement that the libusb (and in turn libftdi)
// be run as root simply add the following line to your /etc/init.d/rc file:
// 
// chmod o+w -R /dev/bus/usb