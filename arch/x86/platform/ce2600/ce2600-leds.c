#include <linux/leds.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)

static struct gpio_led ce2600_leds[] = {
	[0] = {
	       .gpio = 50,
	       .name = "ce2600:ds:orange",
	       .default_trigger = "none",
	       .active_low = 1,
	       },
	[1] = {
	       .gpio = 51,
	       .name = "ce2600:power:green",
	       .default_trigger = "default-on",
	       .active_low = 1,
	       },
	[2] = {
	       .gpio = 58,
	       .name = "ce2600:online:orange",
	       .default_trigger = "activity",
	       .active_low = 1,
	       },
	[3] = {
	       .gpio = 61,
	       .name = "ce2600:us:orange",
	       .default_trigger = "mmc0",
	       .active_low = 1,
	       },
	[4] = {
	       .gpio = 69,
	       .name = "ce2600:ds:green",
	       .default_trigger = "heartbeat",
	       .active_low = 1,
	       },
	[5] = {
	       .gpio = 98,
	       .name = "ce2600:us:green",
	       .default_trigger = "none",
	       .active_low = 1,
	       },
	[6] = {
	       .gpio = 116,
	       .name = "ce2600:online:green",
	       .default_trigger = "none",
	       .active_low = 1,
	       },
};
/*
GPIO 50 = DS orange (inverted)
GPIO 51 = Power (inverted)
GPIO 58 = OnLine orange (inverted)
GPIO 61 = US orange (inverted)
GPIO 69 = DS green (inverted)
GPIO 98 = US green (inverted)
GPIO 116 = OnLine green (inverted)
*/

static struct gpio_led_platform_data ce2600_led_pdata = {
	.num_leds = ARRAY_SIZE(ce2600_leds),
	.leds = ce2600_leds,
};

static struct platform_device ce2600_led_device = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &ce2600_led_pdata,
		},
};

static int __init ce2600_init_led(void)
{
    printk("ce2600: registering %d leds\n", ce2600_led_pdata.num_leds);
	return platform_device_register(&ce2600_led_device);
}
#else
static inline int ce2600_init_led(void)
{
    return -1;
}
#endif


#if defined(CONFIG_KEYBOARD_GPIO_POLLED)

static struct gpio_keys_button ce2600_buttons[] = {
	[0] = {
	       .code	   = KEY_OPTION,
	       .gpio	   = 59,
	       .desc	   = "WiFi button",
	       .active_low     = 0,
	       },
	[1] = {
	       .code	   = KEY_SELECT,
	       .gpio	   = 59,
	       .desc	   = "WPS button",
	       .active_low     = 0,
	       },
};

static struct gpio_keys_platform_data ce2600_button_data = {
	.buttons	= ce2600_buttons,
	.nbuttons       = ARRAY_SIZE(ce2600_buttons),
};

static struct platform_device ce2600_button_device = {
	.name	   = "gpio-keys-polled",
	.id	     = -1,
	.num_resources  = 0,
	.dev	    = {
		.platform_data  = &ce2600_button_data,
	},
};

static int __init ce2600_init_buttons(void)
{
    printk("ce2600: registering %d buttons\n", ce2600_button_data.nbuttons);
	return platform_device_register(&ce2600_button_device);
}
#else
static inline int ce2600_init_buttons(void)
{
    return -1;
}
#endif

extern int intelce_detected(void);

static int __init ce2600_leds_init(void)
{
	if (!intelce_detected())
		return -ENODEV;

	ce2600_init_buttons();
	return ce2600_init_led();
}

static void __exit ce2600_leds_exit(void)
{
    platform_device_unregister(&ce2600_led_device);
}

module_init(ce2600_leds_init);
module_exit(ce2600_leds_exit);

MODULE_AUTHOR("Santiago Hormazabal <santiagohssl@gmail.com>");
MODULE_DESCRIPTION("LEDs for CE2600 modems");
MODULE_LICENSE("GPL v2");
