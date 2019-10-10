/*
 * An emulated PCI device to serve as the interface of PV DMA.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "sysemu/sysemu.h"
#include "qemu/error-report.h"

#define PVDMA_PCI_DEVICE_ID   0xabcd

#define TYPE_PCI_PVDMA_DEVICE "pvdma-pci"
#define PVDMA(obj)        OBJECT_CHECK(PvDmaState, obj, TYPE_PCI_PVDMA_DEVICE)

typedef struct PvDmaMmioInfo{
    uint64_t regs[4];
} PvDmaMmioInfo;

typedef struct PvDmaState {
    PCIDevice pci_dev;
    PvDmaMmioInfo mmio;
    MemoryRegion mmio_region;
} PvDmaState;

bool has_pvdma = false;

static uint64_t pvdma_mmio_read(void *opaque, hwaddr addr, uint32_t size)
{
    PvDmaState *pvdma = opaque;

    if (addr >= sizeof(pvdma->mmio) ||
        (addr + size - 1) > sizeof(pvdma->mmio) ||
        (addr % 8) != 0 ||
	(addr / 8) >= 4) {
        error_report("%s(): invalid addr/size: 0x%lx/0x%x\n",
                     __FUNCTION__, addr, size);
        return (uint64_t)-1;;
    }

    return pvdma->mmio.regs[addr/8];
}

static void pvdma_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    PvDmaState *pvdma = opaque;

    if (addr >= sizeof(pvdma->mmio) ||
        (addr + size - 1) > sizeof(pvdma->mmio) ||
        (addr % 8) != 0) {
        error_report("%s(): invalid addr/size: 0x%lx/0x%x\n",
                     __FUNCTION__, addr, size);
        return;
    }

    pvdma->mmio.regs[addr/8] = val;

    return;
}

static const MemoryRegionOps pvdma_mmio_ops = {
    .read = pvdma_mmio_read,
    .write = pvdma_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};


static void pvdma_pci_realize(PCIDevice *pdev, Error **errp)
{
    PvDmaState *pvdma = PVDMA(pdev);
    uint8_t type = PCI_BASE_ADDRESS_SPACE_MEMORY |
                   PCI_BASE_ADDRESS_MEM_TYPE_64 |
                   PCI_BASE_ADDRESS_MEM_PREFETCH;

    has_pvdma = true;

    memset(&pvdma->mmio, 0, sizeof(pvdma->mmio));

    memory_region_init_io(&pvdma->mmio_region, OBJECT(pvdma), &pvdma_mmio_ops, pvdma,
                    "pvdma_info_region", sizeof(pvdma->mmio));
    pci_register_bar(pdev, 0, type, &pvdma->mmio_region);
}

static void pvdma_reset(DeviceState *dev)
{
}

static void pvdma_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pvdma_pci_realize;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = PVDMA_PCI_DEVICE_ID;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;
    dc->reset = pvdma_reset;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static TypeInfo pvdma_pci_info = {
    .name = "pvdma-pci",
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PvDmaState),
    .class_init = pvdma_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
     },
};

static void pvdma_pci_register_types(void)
{
    type_register_static(&pvdma_pci_info);
}

type_init(pvdma_pci_register_types)
