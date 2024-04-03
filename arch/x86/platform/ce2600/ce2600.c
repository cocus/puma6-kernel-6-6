// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel ce2600  platform specific setup code
 *
 * (C) Copyright 2010 Intel Corporation
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/reboot.h>
#include <linux/serial_reg.h>
#include <linux/serial_8250.h>

#include <asm/prom.h>
#include <asm/setup.h>
#include <asm/i8259.h>
#include <asm/io.h>
#include <asm/io_apic.h>
#include <asm/processor.h>

#ifdef CONFIG_SERIAL_8250

static unsigned int mem_serial_in(struct uart_port *p, int offset)
{
	offset = offset << p->regshift;
	return readl(p->membase + offset);
}

/*
 * The UART Tx interrupts are not set under some conditions and therefore serial
 * transmission hangs. This is a silicon issue and has not been root caused. The
 * workaround for this silicon issue checks UART_LSR_THRE bit and UART_LSR_TEMT
 * bit of LSR register in interrupt handler to see whether at least one of these
 * two bits is set, if so then process the transmit request. If this workaround
 * is not applied, then the serial transmission may hang. This workaround is for
 * errata number 9 in Errata - B step.
*/

static unsigned int ce2600_mem_serial_in(struct uart_port *p, int offset)
{
	unsigned int ret, ier, lsr;

	if (offset == UART_IIR) {
		offset = offset << p->regshift;
		ret = readl(p->membase + offset);
		if (ret & UART_IIR_NO_INT) {
			/* see if the TX interrupt should have really set */
			ier = mem_serial_in(p, UART_IER);
			/* see if the UART's XMIT interrupt is enabled */
			if (ier & UART_IER_THRI) {
				lsr = mem_serial_in(p, UART_LSR);
				/* now check to see if the UART should be
				   generating an interrupt (but isn't) */
				if (lsr & (UART_LSR_THRE | UART_LSR_TEMT))
					ret &= ~UART_IIR_NO_INT;
			}
		}
	} else
		ret =  mem_serial_in(p, offset);
	return ret;
}

static void ce2600_mem_serial_out(struct uart_port *p, int offset, int value)
{
	offset = offset << p->regshift;
	writel(value, p->membase + offset);
}

static void ce2600_serial_fixup(int port, struct uart_port *up,
	u32 *capabilities)
{
#ifdef CONFIG_EARLY_PRINTK
	/*
	 * Over ride the legacy port configuration that comes from
	 * asm/serial.h. Using the ioport driver then switching to the
	 * PCI memmaped driver hangs the IOAPIC
	 */
	if (up->iotype !=  UPIO_MEM32) {
		up->uartclk  = 14745600;
		up->mapbase = 0xdffe0200;
		set_fixmap_nocache(FIX_EARLYCON_MEM_BASE,
				up->mapbase & PAGE_MASK);
		up->membase =
			(void __iomem *)__fix_to_virt(FIX_EARLYCON_MEM_BASE);
		up->membase += up->mapbase & ~PAGE_MASK;
		up->mapbase += port * 0x100;
		up->membase += port * 0x100;
		up->iotype   = UPIO_MEM32;
		up->regshift = 2;
		up->irq = 4;
	}
#endif
	up->iobase = 0;
	up->serial_in = ce2600_mem_serial_in;
	up->serial_out = ce2600_mem_serial_out;

	*capabilities |= (1 << 12);
}

static __init void serial_fixup(void)
{
	serial8250_set_isa_configurator(ce2600_serial_fixup);
}

#else
static inline void serial_fixup(void) {};
#endif

enum ce2600_board_type {
	CE_BOARD_TYPE_HP_BOARD_TYPE = 0,
	CE_BOARD_TYPE_HP_MG_BOARD_TYPE,
	CE_BOARD_TYPE_FM_BOARD_TYPE,
	CE_BOARD_TYPE_CAT_ISLAND_BOARD_TYPE,
	CE_BOARD_TYPE_GS_BOARD_TYPE,
	CE_BOARD_TYPE_CR_BOARD_TYPE,

	CE_BOARD_TYPE_MAX = CE_BOARD_TYPE_CR_BOARD_TYPE
};

static const char* _ce2600_board_types_strings[CE_BOARD_TYPE_MAX] = {
	"Harbor Park",
	"Harbor Park - MG",
	"Falcon Mine",
	"Cat Island",
	"Golden Spring",
	"Cat River(CR)",
};

static unsigned int _intel_ce_board_type = -1;
static unsigned int _intel_ce_nic_phy_mode = -1;
static unsigned int _intel_ce_board_rev = -1;
static unsigned int _intel_ce_flash_layout_table = -1;

static unsigned int _intel_ce_init = 0;

int intelce_get_board_type(unsigned int *board)
{
	if (board) {
		*board = _intel_ce_board_type;
	}
	return 0;
}
EXPORT_SYMBOL(intelce_get_board_type);

int intelce_set_board_type(unsigned int board)
{
	if (board > CE_BOARD_TYPE_MAX)
		return -1;
	_intel_ce_board_type = board;
	printk("ce2600: board type set to 0x%x (%s)\n",
		_intel_ce_board_type,
		_ce2600_board_types_strings[_intel_ce_board_type]);
	return 0;
}
EXPORT_SYMBOL(intelce_set_board_type);

int intelce_detected(void)
{
	return _intel_ce_init;
}
EXPORT_SYMBOL(intelce_detected);


/* ce2600 specific setup data parsing. Happens AFTER x86_ce2600_early_setup.
 */
void __init x86_ce2600_set_setup_data(u32 type, const void *buf, size_t len)
{
	/* only proceed if the type of loader is exactly 0x10 (only thing set by CEFDK) */
	if (boot_params.hdr.type_of_loader != 0x10)
		return;

	switch (type) {
		case SETUP_INTEL_CE_BOARD_TYPE:
			if (*((u32*)buf) > CE_BOARD_TYPE_MAX)
				return;
			_intel_ce_board_type = *((u32*)buf);
			printk("ce2600 setup data: board type 0x%x (%s)\n",
				_intel_ce_board_type,
				_ce2600_board_types_strings[_intel_ce_board_type]);
			break;
		case SETUP_INTEL_CE_NIC_PHY_MODE:
			_intel_ce_nic_phy_mode = *((u32*)buf);
			printk("ce2600 setup data: nic phy mode 0x%x\n",
				_intel_ce_nic_phy_mode);
			break;
		case SETUP_INTEL_CE_BOARD_REV:
			_intel_ce_board_rev = *((u32*)buf);
			printk("ce2600 setup data: board rev 0x%x\n",
				_intel_ce_board_rev);
			break;
		case SETUP_INTEL_CE_FLASH_LAYOUT_TABLE:
			_intel_ce_flash_layout_table = *((u32*)buf);
			printk("ce2600 setup data: layout table 0x%x\n",
				_intel_ce_flash_layout_table);
			break;
		default:
			/* shouldn't happen */
			return;
	}

	if (_intel_ce_init == 0) {
		x86_platform.legacy.devices.pnpbios = 0;
		x86_platform.legacy.rtc = 0;
		x86_platform.legacy.i8042 = X86_LEGACY_I8042_PLATFORM_ABSENT;
		/*
		* By default, the reboot method is ACPI which is supported by the
		* ce2600 bootloader CEFDK using FADT.ResetReg Address and ResetValue
		* the bootloader will however issue a system power off instead of
		* reboot. By using BOOT_KBD we ensure proper system reboot as
		* expected.
		*/
		reboot_type = BOOT_KBD;

		serial_fixup();



		_intel_ce_init = 1;
	}
}
