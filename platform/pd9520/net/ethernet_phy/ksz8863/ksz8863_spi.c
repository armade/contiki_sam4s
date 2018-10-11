
#include "compiler.h"
#include "gmac.h"
#include "ethernet_phy.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>


/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond
enum switch_reg {

	ChipID0 = 0,
	ChipID1,
	// Global Registers: 0-15
	GlobalControl0,
	GlobalControl1,
	GlobalControl2,
	GlobalControl3,
	GlobalControl4,
	GlobalControl5,
	GlobalControl6,
	GlobalControl7,
	GlobalControl8,
	GlobalControl9,
	GlobalControl10,
	GlobalControl11,
	GlobalControl12,
	GlobalControl13,

	// Port Registers: 16-95
	Port1Control0 = 16,
	Port1Control1,
	Port1Control2,
	Port1Control3,
	Port1Control4,
	Port1Control5,
	Port1Control6,
	Port1Control7,
	Port1Control8,
	Port1Control9,
	Port1Control10,
	Port1Control11,
	Port1Control12,
	Port1Control13,
	Port1Status0,
	Port1Status1,

	Port2Control0 = 32,
	Port2Control1,
	Port2Control2,
	Port2Control3,
	Port2Control4,
	Port2Control5,
	Port2Control6,
	Port2Control7,
	Port2Control8,
	Port2Control9,
	Port2Control10,
	Port2Control11,
	Port2Control12,
	Port2Control13,
	Port2Status0,
	Port2Status1,

	Port3Control0 = 48,
	Port3Control1,
	Port3Control2,
	Port3Control3,
	Port3Control4,
	Port3Control5,
	Port3Control6,
	Port3Control7,
	Port3Control8,
	Port3Control9,
	Reservednotappliedtoport3, // 58-62
	Port3Status1 = 63,

	// ksz8863 specific
	Reset = 67,

	TOSPriorityControlRegister0 = 96,
	TOSPriorityControlRegister1,
	TOSPriorityControlRegister2,
	TOSPriorityControlRegister3,
	TOSPriorityControlRegister4,
	TOSPriorityControlRegister5,
	TOSPriorityControlRegister6,
	TOSPriorityControlRegister7,
	TOSPriorityControlRegister8,
	TOSPriorityControlRegister9,
	TOSPriorityControlRegister10,
	TOSPriorityControlRegister11,
	TOSPriorityControlRegister12,
	TOSPriorityControlRegister13,
	TOSPriorityControlRegister14,
	TOSPriorityControlRegister15,

	MAC_address0 = 112,
	MAC_address1,
	MAC_address2,
	MAC_address3,
	MAC_address4,
	MAC_address5,
	MAC_address6,

	IndirectAccessControl0 = 121,
	IndirectAccessControl1,
	IndirectDataRegister8,
	IndirectDataRegister7,
	IndirectDataRegister6,
	IndirectDataRegister5,
	IndirectDataRegister4,
	IndirectDataRegister3,
	IndirectDataRegister2,
	IndirectDataRegister1,
	IndirectDataRegister0,

	Port1TxqSplitForQ0 = 175,
	Port1TxqSplitForQ1,
	Port1TxqSplitForQ2,
	Port1TxqSplitForQ3,
	Port2TxqSplitForQ0,
	Port2TxqSplitForQ1,
	Port2TxqSplitForQ2,
	Port2TxqSplitForQ3,
	Port3TxqSplitForQ0,
	Port3TxqSplitForQ1,
	Port3TxqSplitForQ2,
	Port3TxqSplitForQ3,

	InterruptEnable = 187,
	LinkChangeInterrupt,

	InsertSrcPvid = 194,
	PowerManagementLedMode,

	RegisterSize = 198,
};

#define FIXED_RECORD				0xfe
#define END_OF_RECORD               0xff
#define F_R							FIXED_RECORD
#define EOR							END_OF_RECORD

#define ID1_CHIPID_MASK                0x0f
#define ID1_CHIPID_SHIFT            0x04
#define ID1_REVISION_MASK            0x07
#define ID1_REVISION_SHIFT            0x01
#define ID1_STOP_SWITCH                0x00    // start the switch
#define ID1_START_SWITCH            0x01    // start the switch

#define KSZ8863_FAMILY_ID            0x88
#define KSZ8863_CHIP_ID                0x03
#define KSZ8863_PHYID1                0x0022
#define KSZ8863_PHYID2                0x1430
#define KSZ8863_PORTS_REGS_OFFSET    16

#define KSZ8863_PHY1_DFLT_ADDR        1
#define KSZ8863_PHY2_DFLT_ADDR        2

#define KSZ8863_PHY1_ADDR            1
#define KSZ8863_PHY2_ADDR            2

#define KSZ8863_MDIO_MIN            MII_BMCR
#define KSZ8863_MDIO_MAX            MII_ANLPAR

#define KSZ8863_CMD_WRITE            0x02U
#define KSZ8863_CMD_READ            0x03U

#define KSZ8863_RESET_DELAY            10 // usec

// max switch port support logical Network interface
#define KSZ8863_MAX_ETH                2
#define KSZ8863_MIN_ETH                1

// The KSZ8863 Switch as a single PHY
int ks8872_total_phy = 2;

/*-----------------------------------------------------------------------------------------*/
// How to translate a SPI register
typedef struct {
    uint8_t mdio_bit;
    uint8_t spi_reg; // case FIXED_RECORD: Fixed value (in spi_bit)
    uint8_t spi_bit;
} mdio2spibittrad;

static mdio2spibittrad bmcr_trads[] = {
    {14, 29, 0},     // Loopback
    {13, 28, 6},     // Force 100
    {12, 28, 7},     // Auto negotiation enable
    {11, 29, 3},     // Power down
    {9, 29, 5},     // Restart auto negotiation
    {8, 28, 5},     // Force full duplex
    {5, 31, 7},     // MDI/MDI-x mode
    {4, 29, 1},     // Force MDI
    {3, 29, 2},     // Disable MDI
    {2, 29, 4},     // Disable far-end fault detection
    {1, 29, 6},     // Disable transmit
    {0, 29, 7},     // Disable LED
    {EOR, }
};
static mdio2spibittrad bmsr_trads[] = {
    {14, F_R, 1},     // 100 full capable
    {13, F_R, 1},     // 100 half capable
    {12, F_R, 1},     // 10 full capable
    {11, F_R, 1},     // 10 half capable
    {5, 30, 6},     // Auto-negotiation complete
    {4, 31, 0},     // Far-end fault detect
    {3, 28, 7},     // Auto-negotiation capable
    {2, 30, 5},        // Link up
    {EOR, }
};
static mdio2spibittrad anar_trads[] = {
    {10, 28, 4},     // Pause
    {8, 28, 3},     // Adv 100 full
    {7, 28, 2},     // Adv 100 half
    {6, 28, 1},     // Adv 10 full
    {5, 28, 0},     // Adv 10 half
    {0, F_R, 1},     // 802 3
    {EOR, }
};
static mdio2spibittrad anlpar_trads[] = {
    {10, 30, 4},     // Pause
    {8, 30, 3},     // Link partner adv 100 full
    {7, 30, 2},     // Link partner adv 100 half
    {6, 30, 1},     // Link partner adv 10 full
    {5, 30, 0},     // Link partner adv 10 half
    {EOR, }
};

static mdio2spibittrad *mdio2spibittrads[] = {
    [MII_BMCR] = bmcr_trads,
    [MII_BMSR] = bmsr_trads,
    [MII_ANAR] = anar_trads,
    [MII_ANLPAR] = anlpar_trads
};
/*-----------------------------------------------------------------------------------------*/
// SPI registers to read to fill out the MDIO register
// (NB: for PORT1 -  if PORT2 the register is added with 16)
static uint8_t bmcr_spiregs[] = {31, 28, 29, END_OF_RECORD};
static uint8_t bmsr_spiregs[] = {31, 30, 28, FIXED_RECORD, END_OF_RECORD};
static uint8_t anar_spiregs[] = {28, FIXED_RECORD, END_OF_RECORD};
static uint8_t anlpar_spiregs[] = {30, END_OF_RECORD};

static uint8_t *mdioreg2spiregs[] = {
    [MII_BMCR] = bmcr_spiregs,
    [MII_BMSR] = bmsr_spiregs,
    [MII_ANAR] = anar_spiregs,
    [MII_ANLPAR] = anlpar_spiregs
};

//#define KSZ8863_DBG

#ifdef KSZ8863_DBG
#define dbg printf
static char *mdioreg2str[] = {
    [MII_BMCR] =      "BMCR",         // Basic control register
    [MII_BMSR] =      "BMSR",         // Basic status register
    [MII_PHYSID1] =   "PHYSID1",     // Physical identifier I
    [MII_PHYSID2] =   "PHYSID2",     // Physical identifier II
    [MII_ADVERTISE] = "ADVERTISE",     // Advertisement register
    [MII_LPA] =       "LPA"         // Link partner ability register
};
#else
#define dbg(...)
#endif

int ethsw_read(uint8_t *p, uint8_t a, uint32_t n);
int ethsw_write(uint8_t a, uint8_t *p, uint32_t n);
void spi3_delay(void);

void udelay(int delay_us)
{
	int delay = delay_us*10;
	int i;

	for(i=0;i<delay;i++)
		spi3_delay();
}

static int KSZ8863_read(uint8_t *buf, uint8_t offset, uint8_t count)
{
	return ethsw_read(buf, offset, count);
}

static int KSZ8863_write(uint8_t *buf, uint8_t offset, uint8_t count)
{
	return ethsw_write(offset, buf, count);
}

static inline int KSZ8863_read_reg(uint8_t addr, uint8_t *buf)
{
    return ethsw_read(buf, addr, 1);
}

static inline int KSZ8863_write_reg(uint8_t addr, uint8_t val)
{
	uint8_t buf = val;

    return ethsw_write(addr, &buf, 1);
}

static inline int KSZ8863_stop(void)
{
    return KSZ8863_write_reg(ChipID1, ID1_STOP_SWITCH);
}

static inline int KSZ8863_start(void)
{
    return KSZ8863_write_reg(ChipID1, ID1_START_SWITCH);
}

int KSZ8863_reset(void)
{
    int err;

    err = KSZ8863_stop();
    if (err)    return err;

    udelay(KSZ8863_RESET_DELAY);

    err = KSZ8863_write_reg(Reset, 4); // Software reset
    if (err)    return err;

    return KSZ8863_start();
}

int KSZ8863_setup_MAC_addr(const unsigned char *addr)
{
    unsigned char *MAC_addr = (unsigned char *)addr;

    return KSZ8863_write(MAC_addr, MAC_address0, 6);
}

 int KSZ8863_ack_interrupt(void)
{
    uint8_t spi_val;

    KSZ8863_read(&spi_val, LinkChangeInterrupt, 1);

    spi_val |= 1 << (0);
    spi_val |= 1 << (1);

    return KSZ8863_write(&spi_val, LinkChangeInterrupt, 1);
}

int KSZ8863_config_intr(void)
{
    uint8_t spi_val;

    KSZ8863_read(&spi_val, InterruptEnable, 1);

    spi_val |= 1 << (0);
    spi_val |= 1 << (1);

    return KSZ8863_write(&spi_val, InterruptEnable, 1);
}

/* ------------------------------------------------------------------------ */

static uint16_t spi2mdiobits(int phy_id, uint8_t spi_val, uint8_t spireg, uint8_t mdio_reg, uint16_t initmdioval)
{
    mdio2spibittrad *trads;
    int i,bit;
    uint16_t ret = initmdioval;

    trads = mdio2spibittrads[mdio_reg];
    for (i = 0; trads[i].mdio_bit != END_OF_RECORD; i++)
        if (trads[i].spi_reg == spireg) {
            if (spireg == FIXED_RECORD) {
                ret &= ~(1 << trads[i].mdio_bit);
                ret |= trads[i].spi_bit << trads[i].mdio_bit;
            } else {
                bit = !!(spi_val & (1 << trads[i].spi_bit));
                ret &= ~(1 << trads[i].mdio_bit);
                ret |= bit << trads[i].mdio_bit;
            }
        }

    return ret;
}

static uint8_t mdio2spibits(int phy_id, uint16_t mdio_val, uint8_t spireg, uint8_t mdio_reg, uint8_t initspival)
{
    mdio2spibittrad *trads;
    int i;
    uint8_t ret = initspival;

    trads = mdio2spibittrads[mdio_reg];
    for (i = 0; trads[i].mdio_bit != END_OF_RECORD; i++)
        if (trads[i].spi_reg == spireg) {
            int bit = !!(mdio_val & (1 << trads[i].mdio_bit));
            ret &= ~(1 << trads[i].spi_bit);
            ret |= bit << trads[i].spi_bit;
        }

    return ret;
}

static int get_spi2mdioreg(int phy_id, uint8_t mdio_reg, uint16_t *mdio_val)
{
    uint8_t spi_val=0;
    uint8_t *spi_regs;
    int i, ret = 0;

    if (mdio_reg < KSZ8863_MDIO_MIN || mdio_reg > KSZ8863_MDIO_MAX) {
        printf( "%s: register out of bonds (%02x)\n", __func__, mdio_reg);
        return -1;
    }

    if (phy_id != KSZ8863_PHY1_DFLT_ADDR && phy_id != KSZ8863_PHY2_DFLT_ADDR) {
        printf( "%s: phy addr out of bonds (%02x)\n", __func__, phy_id);
        return -1;
    }

    dbg("%s: %s READ\n", __func__, mdioreg2str[mdio_reg]);

    if (mdio_reg == MII_PHYID1) {
        *mdio_val = (uint16_t)KSZ8863_PHYID1;
        return 0;
    } else if (mdio_reg == MII_PHYID2) {
        *mdio_val = (uint16_t)KSZ8863_PHYID2;
        return 0;
    }

    spi_regs = mdioreg2spiregs[mdio_reg];
    *mdio_val = 0;
    for (i = 0; spi_regs[i] != END_OF_RECORD; i++) {
        ret = KSZ8863_read(&spi_val, spi_regs[i] + (phy_id - 1) * KSZ8863_PORTS_REGS_OFFSET, 1);
        if (ret < 0)
            break;
        *mdio_val |= spi2mdiobits(phy_id, spi_val, spi_regs[i], mdio_reg, 0);
    }

    return ret;
}

static int set_mdio2spireg(int phy_id, uint8_t mdio_reg, uint16_t mdio_val)
{
    uint8_t spi_val;
    uint8_t *spi_regs;
    int i;
    int ret = 0;

    if (mdio_reg < KSZ8863_MDIO_MIN || mdio_reg > KSZ8863_MDIO_MAX) {
        printf("register out of bonds (%02x)\n", mdio_reg);
        return -1;
    }

    if (phy_id != KSZ8863_PHY1_DFLT_ADDR && phy_id != KSZ8863_PHY2_DFLT_ADDR) {
    	printf("phy addr out of bonds (%02x)\n", mdio_reg);
        return -1;
    }

    spi_regs = mdioreg2spiregs[mdio_reg];
    for (i = 0; spi_regs[i] != END_OF_RECORD; i++) {
        if (spi_regs[i] == FIXED_RECORD)
            continue;

        ret = KSZ8863_read(&spi_val, spi_regs[i] + (phy_id - 1) * KSZ8863_PORTS_REGS_OFFSET, 1);
        if (ret < 0)
            break;

        spi_val = mdio2spibits(phy_id, mdio_val, spi_regs[i], mdio_reg, spi_val);
        ret = KSZ8863_write(&spi_val, spi_regs[i]+ (phy_id - 1) * KSZ8863_PORTS_REGS_OFFSET, 1);
        if (ret < 0)
            break;
    }

    return ret;
}
/**
 * \brief Wait PHY operation to be completed.
 *
 * \param p_gmac HW controller address.
 * \param ul_retry The retry times.
 *
 * Return GMAC_OK if the operation is completed successfully.
 */
/*static uint8_t gmac_phy_wait(Gmac* p_gmac, const uint32_t ul_retry)
{
	volatile uint32_t ul_retry_count = 0;

	while (!gmac_is_phy_idle(p_gmac)) {
		ul_retry_count++;

		if (ul_retry_count >= ul_retry) {
			return GMAC_TIMEOUT;
		}
	}
	return GMAC_OK;
}*/

/**
 * \brief Read the PHY register.
 *
 * \param p_gmac   Pointer to the GMAC instance.
 * \param uc_phy_address PHY address.
 * \param uc_address Register address.
 * \param p_value Pointer to a 32-bit location to store read data.
 *
 * \Return GMAC_OK if successfully, GMAC_TIMEOUT if timeout.
 */
uint8_t gmac_phy_read(Gmac* p_gmac, uint8_t uc_phy_address, uint8_t uc_address,
		uint32_t* p_value)
{
	int phy_id = uc_phy_address;
	int regnum = uc_address;
	uint16_t mdio_val;

	// KSZ8863 doesn't support broadcast PHY address
	if (phy_id == 0)
		return GMAC_TIMEOUT;

	// The Switch is represented as single PHY
	if ((regnum == MII_PHYID1) || (regnum == MII_PHYID2)){
		*p_value = get_spi2mdioreg(phy_id, regnum, &mdio_val) < 0 ? -1 : mdio_val;
		return GMAC_OK;
	}

	// read phy 1 link status first
	get_spi2mdioreg(phy_id, MII_BMSR, &mdio_val);

	if(regnum == MII_PCR1)
	{
		// return phy 1 register if phy 1 is linkup, otherwise, return phy 2 register status
		if ((mdio_val & MII_LINK_STATUS) == MII_LINK_STATUS){
			KSZ8863_read((void *)&mdio_val, regnum, 1);
			*p_value = mdio_val;
			return GMAC_OK;
		}
		else{
			KSZ8863_read((void *)&mdio_val, 0x2E, 1);
			*p_value = mdio_val;
			return GMAC_OK;
		}
	}

	// Is it going to read phy link status?
	if (regnum == MII_BMSR) {
		// Yes, return link down if both Switch phy are link down, otherwise, linkup
		if ((mdio_val & MII_LINK_STATUS) == MII_LINK_STATUS){
			*p_value = mdio_val;
			return GMAC_OK;
		}
		else{
			get_spi2mdioreg((phy_id+1), regnum, &mdio_val);
			*p_value = mdio_val;
			return GMAC_OK;
		}
	} else {
		// No, return phy 1 register if phy 1 is linkup, otherwise, return phy 2 register status
		if ((mdio_val & MII_LINK_STATUS) == MII_LINK_STATUS){
			get_spi2mdioreg(phy_id, regnum, &mdio_val);
			*p_value = mdio_val;
			return GMAC_OK;
		}
		else{
			get_spi2mdioreg((phy_id+1), regnum, &mdio_val);
			*p_value = mdio_val;
			return GMAC_OK;
		}
	}
	return GMAC_OK;
}

/**
 * \brief Write the PHY register.
 *
 * \param p_gmac   Pointer to the GMAC instance.
 * \param uc_phy_address PHY Address.
 * \param uc_address Register Address.
 * \param ul_value Data to write, actually 16-bit data.
 *
 * \Return GMAC_OK if successfully, GMAC_TIMEOUT if timeout.
 */
uint8_t gmac_phy_write(Gmac* p_gmac, uint8_t uc_phy_address,
		uint8_t uc_address, uint32_t ul_value)
{
	int phy_id = uc_phy_address;
	int regnum = uc_address;
	int phy_addr, i;

	// KSZ8863 doesn't support broadcast PHY address
	if (phy_id == 0)
		return GMAC_TIMEOUT;

	for (i=0; i<ks8872_total_phy; i++) {
		phy_addr = phy_id + i;
		set_mdio2spireg(phy_addr, regnum, ul_value);
	}

	return GMAC_OK;
}

/*=============================================================================================================*/

/*#define SPI3_MISO_BIT 23
#define SPI3_MOSI_BIT 24
#define SPI3_SCK_BIT 25
#define SPI3_CS1_BIT 26
#define SPI3_CS2_BIT 28
#define SPI3_CS3_BIT 29*/
#define SPI3PORT PIOD
/*static Pin spi3[6]={ { 1<<SPI3_MISO_BIT , PIOD, ID_PIOD, PIO_INPUT   , PIO_DEFAULT},
                     { 1<<SPI3_MOSI_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT},
                     { 1<<SPI3_SCK_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT},
                     { 1<<SPI3_CS1_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT},
                     { 1<<SPI3_CS2_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT},
                     { 1<<SPI3_CS3_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT} };*/

#define SPI3_CS1	PIO_PD26
#define SPI3_SCK	PIO_PD25
#define SPI3_MOSI	PIO_PD24
#define SPI3_MISO	PIO_PD23

static unsigned char spi3_selected;

void spi3_init(void)
{
	pio_set_input(PIOD, SPI3_MISO, PIO_PULLUP);
	pio_set_output(PIOD, SPI3_MOSI, 1,  false, true);
	pio_set_output(PIOD, SPI3_SCK, 1,  false, true);
	pio_set_output(PIOD, SPI3_CS1, 1,  false, true);
}

void spi3_delay(void) //min 100ns. Det er 15.
{
	int i;
	for (i=0; i<4; ++i) {
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
}

void spi3_unselect(void)
{
	if (spi3_selected) {
		SPI3PORT->PIO_SODR = SPI3_CS1;
		spi3_selected=0;
		spi3_delay(); //idle
	}
}

void spi3_select_ethsw(void) //ud paa falling edge, ind paa rising edge
{
	spi3_unselect();
	SPI3PORT->PIO_CODR = SPI3_CS1;
	spi3_delay();
	spi3_selected=1;
}

unsigned spi3_ethsw_int(uint32_t d)
{
	uint32_t out=0;
	uint32_t n=8;
	while (n) {
		--n;
		if (d&(1<<n))
			SPI3PORT->PIO_SODR = SPI3_MOSI;
		else
			SPI3PORT->PIO_CODR = SPI3_MOSI;
		SPI3PORT->PIO_CODR = SPI3_SCK;
		spi3_delay();
		//sample before rising edge
		if (SPI3PORT->PIO_PDSR&SPI3_MISO) out|=1<<n;
		SPI3PORT->PIO_SODR = SPI3_SCK;
		spi3_delay();
	}
	return out;
}

int ethsw_write(uint8_t a, uint8_t *p, uint32_t n)
{
	spi3_select_ethsw();
	spi3_ethsw_int(2);
	spi3_ethsw_int(a);
	while (n) {
		spi3_ethsw_int(*p++);
		--n;
	}
	spi3_unselect();

	return 0;
}

int ethsw_read(uint8_t *p, uint8_t a, uint32_t n)
{
	spi3_select_ethsw();
	spi3_ethsw_int(3);
	spi3_ethsw_int(a);
	while (n) {
		*p++=spi3_ethsw_int(0);
		--n;
	}
	spi3_unselect();

	return 0;
}

//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
