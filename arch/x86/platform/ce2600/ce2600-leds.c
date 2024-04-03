#include <linux/leds.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)

static struct gpio_led ce2600_leds[] = {
	[0] = {
	       .gpio = 116,
	       .name = "ce2600-leds-116",
	       .default_trigger = "default-on",
	       .active_low = 1,
	       },
};

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

extern int intelce_detected(void);

static int __init ce2600_leds_init(void)
{
    if (!intelce_detected())
        return -ENODEV;
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
