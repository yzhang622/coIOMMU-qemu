/*
 * An emulated PCI device to serve as the interface of PV DMA.
 */

#include <sys/ioctl.h>
#include <linux/vfio.h>
#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bus.h"
#include "hw/i386/pc.h"
#include "sysemu/sysemu.h"
#include "qemu/error-report.h"

#define PVDMA_PCI_DEVICE_ID   0xabcd

#define TYPE_PCI_PVDMA_DEVICE "pvdma-pci"
#define PVDMA(obj)        OBJECT_CHECK(PvDmaState, obj, TYPE_PCI_PVDMA_DEVICE)
typedef struct PinnedPageQueue {
    unsigned long count;
    QemuMutex page_list_lock;
    QTAILQ_HEAD(, PinnedPage) page_list;
} PinnedPageQueue;
#define PAGE_SHIFT_4K          12
#define PAGE_SIZE_4K           (1 << PAGE_SHIFT_4K)

typedef struct PvDmaMmioInfo {
    uint64_t regs[4];
      struct PvDmaInfo {
          uint64_t ipt_gpa;
          uint64_t cmd;
          uint64_t ipt_level;
          uint64_t gfn_bdf;
      } info;
} PvDmaMmioInfo;

#define IPTE_MAP_CNT_MASK	0xFFFF
#define IPTE_MAP_CNT_MAX	0xFF
#define IPTE_PINNED_FLAG	16
#define IPTE_MAPPED_FLAG	17
#define IPTE_ACCESSED_FLAG	18
#define IPTE_WRITEABLE_FLAG	19
//#define IPTE_DIRTY_FLAG	20

typedef struct {
	int ipte;
} IptLeafEntry;

typedef struct {
	uint64_t present:1;
	uint64_t reserve:11;
	uint64_t pfn:52;
} IptParentEntry;

#define pvdma_set_flag(flag, iptep) \
	set_bit_atomic(flag, (unsigned long*)iptep)

#define pvdma_clear_flag(flag, iptep) \
	clear_bit_atomic(flag, (unsigned long*)iptep)

#define pvdma_test_flag(flag, iptep) \
	test_bit(flag, (unsigned long*)iptep)

typedef struct {
	void *ipt_root;
	unsigned int ipt_level;
} PvDmaIpt;

typedef struct PvDmaState {
    PCIDevice pci_dev;
    PvDmaMmioInfo mmio;
    void *device_bitmap;
    MemoryRegion mmio_region;
    MemoryRegion device_info_region;
    QemuMutex mutex;
    PvDmaIpt ipt;
} PvDmaState;

bool has_pvdma = false;

#define MAX_NUM_DEVICES (1<<16)

static void pvdma_init_device_bitmap_under_bus(PCIBus * bus, void * device_bitmap)
{
    PCIDevice *pdev;
    int devfn;
    uint16_t  bdf;

    for(devfn = 0; devfn < ARRAY_SIZE(bus->devices); devfn++) {
        pdev = bus->devices[devfn];

        if (!pdev || pdev->vfio_container_fd <= 0) {
            bdf = PCI_BUILD_BDF(pci_bus_num(bus), devfn);
            set_bit_atomic(bdf, device_bitmap);
        }
    }
}

static void pvdma_init_device_bitmap(void* device_bitmap)
{
    pci_for_all_buses(pvdma_init_device_bitmap_under_bus, device_bitmap);
}

static bool pvdma_get_vaddr(uint64_t gpa, void **vaddr, bool* read_only)
{
    MemoryRegion *mr;
    hwaddr xlat;
    hwaddr len = PAGE_SIZE_4K;
    //TODO: should add param to notify qemu is gpa shall be writeable.
    bool writable = true;

    mr = address_space_translate(&address_space_memory,
                                 gpa,
                                 &xlat, &len, writable,
                                 MEMTXATTRS_UNSPECIFIED);
    if (!memory_region_is_ram(mr))
        return false;

    *vaddr = memory_region_get_ram_ptr(mr) + xlat;
    *read_only = !writable || mr->readonly;

    return true;
}

#define PVDMA_UPPER_LEVEL_STRIDE	(9)
#define PVDMA_UPPER_LEVEL_MASK		(((uint64_t)1 << PVDMA_UPPER_LEVEL_STRIDE) - 1)
#define PVDMA_PT_LEVEL_STRIDE		(10)
#define PVDMA_PT_LEVEL_MASK		(((uint64_t)1 << PVDMA_PT_LEVEL_STRIDE) - 1)

static inline unsigned int pvdma_level_to_offset(unsigned long pfn, int level)
{
	unsigned int offset;

	if (level == 1)
		return (pfn) & PVDMA_PT_LEVEL_MASK;

	offset = PVDMA_PT_LEVEL_STRIDE + (level - 2) * PVDMA_UPPER_LEVEL_STRIDE;

	return (pfn >> offset) & PVDMA_UPPER_LEVEL_MASK;;
}

static IptLeafEntry * pfn_to_ipt_pte(PvDmaState *s, unsigned long pfn)
{
    void *pt;
    unsigned int index;
    IptParentEntry *parent_pte;
    IptLeafEntry *leaf_pte;
    unsigned int level = s->ipt.ipt_level;
    bool read_only;
    unsigned long gpa;
    unsigned int target_level = 1;

    pt = s->ipt.ipt_root;

    while (level != target_level) {
        index = pvdma_level_to_offset(pfn, level);
        parent_pte = (IptParentEntry *)pt + index;

        if (!parent_pte->present) {
            break;
        } else {
            gpa = (parent_pte->pfn << 12);
            if (!pvdma_get_vaddr(gpa, (void **)&pt, &read_only)) {
                error_report("get vaddr failed for gpa: 0x%lx.\n", gpa);
                break;
            }
        }
        level--;
    }

    if (level > target_level) {
        error_report("IPT absent at level %d for pfn 0x%lx .\n", level, pfn);
        return NULL;
    }

	index = pvdma_level_to_offset(pfn, target_level);
	leaf_pte = (IptLeafEntry *)pt + index;

	return leaf_pte;
}

static PCIDevice* pvdma_get_device(uint16_t bdf, bool *is_assigned)
{
    uint8_t bus_num = bdf >> 8;
    uint8_t devfn = bdf  & 0xff;
    MachineState *ms = MACHINE(qdev_get_machine());
    PCMachineState *pcms = PC_MACHINE(ms);
    PCIBus *bus = pcms->bus;
    PCIDevice* pdev;

    pdev = pci_find_device(bus, bus_num, devfn);

    if (!pdev || pdev->vfio_container_fd <= 0){
        error_report("guest device(%02x:%02x.%02x) is not an assigned one.\n",
                     bus_num, (devfn & 0xf8)>>3, (devfn & 0x07));
        *is_assigned = false;
    } else {
        *is_assigned = true;
    }

    return pdev;
}
static int pvdma_pin_page(PvDmaState *pvdma, uint64_t gfn, PCIDevice *pdev)
{
    uint64_t vaddr;
    bool read_only;
    uint64_t iova = (gfn << PAGE_SHIFT_4K);
    struct vfio_iommu_type1_dma_map map = {
        .argsz = sizeof(map),
        .flags = VFIO_DMA_MAP_FLAG_READ,
        .iova = iova,
        .size = PAGE_SIZE_4K,
        };
    int ret;
    IptLeafEntry *leaf_entry;

    qemu_mutex_lock(&pvdma->mutex);

    leaf_entry = pfn_to_ipt_pte(pvdma, gfn);
    if (leaf_entry == NULL)
        return -1;

    if (pvdma_test_flag(IPTE_PINNED_FLAG, leaf_entry)) {
        qemu_mutex_unlock(&pvdma->mutex);
        return 0;
    }

    if (!pvdma_get_vaddr(iova, (void **)&vaddr, &read_only)) {
        error_report("get vaddr failed for gpa: 0x%lx.\n", iova);
        qemu_mutex_unlock(&pvdma->mutex);
        return -1;
    }

    if (read_only && pvdma_test_flag(IPTE_WRITEABLE_FLAG, leaf_entry)) {
        error_report("write a ready only addr: 0x%lx.\n", iova);
        qemu_mutex_unlock(&pvdma->mutex);
        return -1;
    }

    if (!read_only && pvdma_test_flag(IPTE_WRITEABLE_FLAG, leaf_entry))
        map.flags |= VFIO_DMA_MAP_FLAG_WRITE;

    map.vaddr = vaddr;
    ret = ioctl(pdev->vfio_container_fd, VFIO_IOMMU_MAP_DMA, &map);

    if (ret == 0) {
        pvdma_set_flag(IPTE_PINNED_FLAG, leaf_entry);
    } else {
        error_report("VFIO_MAP_DMA failed, ret=%d, errno=%d.\n", ret, errno);
        if ((errno == 17) &&
            (!pvdma_test_flag(IPTE_PINNED_FLAG, leaf_entry))) {
            error_report("gfn=0x%lx.\n", gfn);
            pvdma_set_flag(IPTE_PINNED_FLAG, leaf_entry);
        }
    }

    qemu_mutex_unlock(&pvdma->mutex);

    return ret;
}

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
    bool read_only;
    bool ret;
    void **vaddr = NULL;
    bool is_assigned;
    PCIDevice *pdev = NULL;
    uint16_t  bdf;
    uint64_t gfn_bdf = 0;

    if (addr >= sizeof(pvdma->mmio) ||
        (addr + size - 1) > sizeof(pvdma->mmio) ||
        (addr % 8) != 0) {
        error_report("%s(): invalid addr/size: 0x%lx/0x%x\n",
                     __FUNCTION__, addr, size);
        return;
    }

    if (addr == 0x0) {
        pvdma->mmio.regs[addr/8] = val;
        vaddr = &pvdma->ipt.ipt_root;
        ret = pvdma_get_vaddr(val, vaddr, &read_only);
        if (!ret) {
            error_report("%s(): address is not mapped in qemu: gpa=0x%lx.\n",
                         __FUNCTION__, val);
        } else if (read_only) {
            error_report("%s(): address is readonly: gpa=0x%lx.\n",
                        __FUNCTION__, val);
        }
    } else if (addr == 0x08) {
        pvdma_init_device_bitmap(pvdma->device_bitmap);
    } else if (addr == 0x10) {
        pvdma->ipt.ipt_level = val;
    } else if (addr == 0x18) {
        gfn_bdf = val;
        bdf = gfn_bdf & 0xffff;
        pdev = pvdma_get_device(bdf, &is_assigned);
        if (!pdev || !is_assigned) {
            set_bit_atomic(bdf, pvdma->device_bitmap);
        } else {
            pvdma_pin_page(pvdma, gfn_bdf >> 16, pdev);
        }

    }

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

    memory_region_init_rom(&pvdma->device_info_region,
                           OBJECT(pdev),
                           "pvdma_device_bitmap",
                           (1<<16)/8,
                           NULL);
    pvdma->device_bitmap = memory_region_get_ram_ptr(&pvdma->device_info_region);
    pci_register_bar(pdev, 2, type, &pvdma->device_info_region);

    qemu_mutex_init(&pvdma->mutex);
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
