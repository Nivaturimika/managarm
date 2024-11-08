#include <nic/amdpcnet/amdpcnet.hpp>
#include <arch/dma_pool.hpp>
#include <arch/dma_structs.hpp>
#ifdef __x86_64__
#include <arch/io_space.hpp>
#endif
#include <arch/mem_space.hpp>
#include <arch/register.hpp>
#include <arch/variable.hpp>
#include <async/recurring-event.hpp>
#include <async/oneshot-event.hpp>
#include <helix/ipc.hpp>
#include <helix/timer.hpp>
#include <helix/memory.hpp>
#include <protocols/hw/client.hpp>
#include <netserver/nic.hpp>
#include <core/queue.hpp>
#include <queue>
#include <string.h>

// debug options
namespace {
constexpr bool logDriverStuff = true;
}

namespace {

// AMD PCNET has two register sets, indexed by the same RAP32/RAP16 register
// - RDP is for CSRs
// - BDP is for BCRs
//
// Simple as that!
inline constexpr arch::scalar_register<uint32_t> pcnet_mac_1(0x0);
inline constexpr arch::scalar_register<uint32_t> pcnet_mac_2(0x4);

inline constexpr arch::scalar_register<uint16_t> pcnet16_rdp(0x10);
inline constexpr arch::scalar_register<uint16_t> pcnet16_rap(0x12);
inline constexpr arch::scalar_register<uint16_t> pcnet16_rst(0x14);
inline constexpr arch::scalar_register<uint16_t> pcnet16_bdp(0x16);

inline constexpr arch::scalar_register<uint32_t> pcnet32_rdp(0x10);
inline constexpr arch::scalar_register<uint32_t> pcnet32_rap(0x14);
inline constexpr arch::scalar_register<uint32_t> pcnet32_rst(0x18);
inline constexpr arch::scalar_register<uint32_t> pcnet32_bdp(0x1c);

struct Request {
	Request(size_t size) : index(0, size) { };
	QueueIndex index;
	async::oneshot_event event;
	arch::dma_buffer_view frame;
};

struct [[gnu::packed]] Descriptor {
	uint32_t addr;
	uint16_t length;
	uint8_t owned;
	uint8_t data[9];
};
static_assert(sizeof(Descriptor) == 16);

struct [[gnu::packed]] InitializerDescriptor {
	uint16_t mode;
	uint8_t rx_len;
	uint8_t tx_len;
	//4
	uint8_t mac[6];
	uint16_t reserved_0;
	//12
	uint8_t ladr[8];
	//20
	uint32_t rx_paddr;
	//24
	uint32_t tx_paddr;
};
static_assert(sizeof(InitializerDescriptor) == 28);

struct AmdPcNetNic;

template<bool IsTransmit>
struct AmdPcNetQueue {
	static const uint32_t descriptor_count = IsTransmit ? 8 : 32;

	std::queue<std::shared_ptr<Request>> requests;
	arch::dma_array<Descriptor> descriptors;
	QueueIndex next_index;
	std::vector<arch::dma_buffer> buffers;
	//
	AmdPcNetQueue() : next_index(0, descriptor_count) {

	}
	void init(AmdPcNetNic& nic);
};

struct AmdPcNetNic : nic::Link {
	AmdPcNetNic(protocols::hw::Device dev);
	virtual async::result<size_t> receive(arch::dma_buffer_view) override;
	virtual async::result<void> send(const arch::dma_buffer_view) override;
	virtual ~AmdPcNetNic() override = default;

	async::result<void> init();
	async::detached processIrqs();

	arch::contiguous_pool dmaPool_;
	protocols::hw::Device _device;
	helix::UniqueDescriptor _irq;
	helix::Mapping _mmio_mapping;
	arch::mem_space _mmio;
	arch::dma_buffer _initializer;

	AmdPcNetQueue<true> _tx;
	AmdPcNetQueue<false> _rx;
protected:
	uint32_t csr_read(uint32_t n);
	void csr_write(uint32_t n, uint32_t m);
};

template<bool IsTransmit>
void AmdPcNetQueue<IsTransmit>::init(AmdPcNetNic& nic) {
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: setting " << descriptor_count << " buffers of " << (IsTransmit ? "TX" : "RX") << std::endl;
	}
	descriptors = arch::dma_array<Descriptor>(nic.dmaPool(), descriptor_count);
	for(uint32_t i = 0; i < descriptor_count; ++i) {
		auto buf = arch::dma_buffer(nic.dmaPool(), 2048);
		memset(buf.data(), 0, buf.size());
		uintptr_t addr = helix_ng::ptrToPhysical(buf.data());
		buffers.push_back(std::move(buf));
		//
		descriptors[i].addr = uint32_t(addr);
		uint16_t len = uint16_t(1520);
		len &= 0x0fff;
		len |= 0xf000;
		descriptors[i].length = uint16_t(len);
		descriptors[i].owned = IsTransmit ? 0x00 : 0x80;
	}
}

uint32_t AmdPcNetNic::csr_read(uint32_t n) {
	_mmio.store(pcnet32_rap, n); //read
	return _mmio.load(pcnet32_rdp);
}
void AmdPcNetNic::csr_write(uint32_t n, uint32_t m) {
	_mmio.store(pcnet32_rap, n); //write
	_mmio.store(pcnet32_rdp, m);
}

async::result<void> AmdPcNetNic::init() {
	// Setup for PCI - then select the first MMIO bar
	_irq = co_await _device.accessIrq();
	co_await _device.enableBusmaster();
	auto info = co_await _device.getPciInfo();
	size_t bar_index = 0;
	while(true) {
		assert(bar_index < 6 && "drivers/amdpcnet: unable to locate MMIO BAR!");
		if(info.barInfo[bar_index].ioType == protocols::hw::IoType::kIoTypeMemory) {
			break;
		}
		bar_index++;
	}
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: selected pci bar " << bar_index << std::endl;
	}
	auto &barInfo = info.barInfo[bar_index];
	assert(barInfo.ioType == protocols::hw::IoType::kIoTypeMemory);
	auto bar = co_await _device.accessBar(bar_index);
	_mmio_mapping = {bar, barInfo.offset, barInfo.length};
	_mmio = _mmio_mapping.get();

	// Now we are ready to start sending shit
	// Reset the fucker (in both modes, 16-bit and 32-bit)
	// Cleverly, RST is on +0x18 for 32-bit mode, so reading from there causes
	// no (important) side effects! wohoo thanks pcnet i love you pcnet
	_mmio.load(pcnet32_rst);
	_mmio.load(pcnet16_rst);
	co_await helix::sleepFor(1'000'000); // sleep for 1 microsecond
	_mmio.store(pcnet32_rdp, 0); // 32-bit mode

	//
	// SWSTYLE read and writeback to csr 58
	auto csr58 = (csr_read(58) & 0xff00) | 0x02;
	csr_write(58, csr58);
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: initialized the fucker w/a csr58 = " << csr58 << std::endl;
	}

	//
	// ASEL select automatically coaxial shit idfk im not a TV
	_mmio.store(pcnet32_rap, 2); //read
	auto bcr2 = _mmio.load(pcnet32_bdp);
	bcr2 |= 0x02;
	_mmio.store(pcnet32_rap, 2); //write
	_mmio.store(pcnet32_bdp, bcr2);

	auto mac_lower = _mmio.load(pcnet_mac_1);
	auto mac_higher = _mmio.load(pcnet_mac_2);
	mac_[0] = (mac_lower >> 0) & 0xFF;
	mac_[1] = (mac_lower >> 8) & 0xFF;
	mac_[2] = (mac_lower >> 16) & 0xFF;
	mac_[3] = (mac_lower >> 24) & 0xFF;
	mac_[4] = (mac_higher >> 0) & 0xFF;
	mac_[5] = (mac_higher >> 8) & 0xFF;
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: MAC " << mac_ << std::endl;
	}

	//
	// TRANSMIT
	_tx.init(*this);

	//
	// RECEIVE
	_rx.init(*this);

	//
	// INITIALIZE
	_initializer = arch::dma_buffer(dmaPool(), sizeof(InitializerDescriptor));
	auto* init = (InitializerDescriptor*)_initializer.data();
	init->mode = 0;
	init->reserved_0 = 0;
	init->ladr[0] = 0;
	init->ladr[1] = 0;
	init->ladr[2] = 0;
	init->ladr[3] = 0;
	init->ladr[4] = 0;
	init->ladr[5] = 0;
	init->ladr[6] = 0;
	init->ladr[7] = 0;
	//
	init->mac[0] = mac_[0];
	init->mac[1] = mac_[1];
	init->mac[2] = mac_[2];
	init->mac[3] = mac_[3];
	init->mac[4] = mac_[4];
	init->mac[5] = mac_[5];
	//
	init->rx_len = 5 << 4;
	init->tx_len = 3 << 4;
	//
	init->tx_paddr = helix_ng::ptrToPhysical(_tx.descriptors.data());
	init->rx_paddr = helix_ng::ptrToPhysical(_rx.descriptors.data());

	// CSR1 and CSR2 hold the initialization structure!
	uintptr_t init_addr = helix_ng::ptrToPhysical(_initializer.data());
	_mmio.store(pcnet32_rap, 1); //write
	_mmio.store(pcnet32_rdp, uint16_t(init_addr));
	_mmio.store(pcnet32_rap, 2); //write
	_mmio.store(pcnet32_rdp, uint16_t(init_addr >> 16));

	//
	// Unset bits 10,9,8 so we get RX,TX and INIT irqs (respectively)
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: step-unset-irqs" << std::endl;
	}
	auto csr3 = csr_read(3) & ~((1 << 12) | (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));
	csr_write(3, csr3);

	//
	// Automatically pad ethernet packets
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: step-automatically-pad-ethernet-packets" << std::endl;
	}
	auto csr4 = csr_read(4) | (1 << 11);
	csr_write(4, csr4);

	//
	// Initialize step
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: step-initialize" << std::endl;
	}
	csr_write(0, (1 << 0) | (1 << 6));

	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: 2434234" << std::endl;
	}

	processIrqs();
}

AmdPcNetNic::AmdPcNetNic(protocols::hw::Device device)
	: nic::Link(1500, &dmaPool_), _device { std::move(device) }
{
	promiscuous_ = true;
	all_multicast_ = true;
	multicast_ = true;
	broadcast_ = true;
	l1_up_ = true;
	async::run(this->init(), helix::currentDispatcher);
}

async::result<size_t> AmdPcNetNic::receive(arch::dma_buffer_view frame) {
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: receive() -> " << frame.size() << std::endl;
	}
	auto req = std::make_shared<Request>(_rx.descriptor_count);
	req->frame = frame;
	req->index = _rx.next_index;
	++_rx.next_index;
	co_await req->event.wait();
	co_return frame.size();
}

async::result<void> AmdPcNetNic::send(const arch::dma_buffer_view frame) {
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: send() -> " << frame.size() << std::endl;
	}

	auto req = std::make_shared<Request>(_tx.descriptor_count);
	req->frame = frame;

	memcpy(_tx.buffers[_tx.next_index].data(), req->frame.data(), 1520); //nov the pcnet will steal our dma
	_tx.descriptors[_tx.next_index].owned |= 0x02; //START OF SPLIT PACKET
	_tx.descriptors[_tx.next_index].owned |= 0x01; //END OF PACKET
	_tx.descriptors[_tx.next_index].owned |= 0x80; //give it to the device!

	req->index = _tx.next_index;
	++_tx.next_index;
	co_await req->event.wait();
}

async::detached AmdPcNetNic::processIrqs() {
	co_await _device.enableBusIrq();
	if(logDriverStuff) {
		std::cout << "drivers/amdpcnet: irqs enabled!" << std::endl;
	}

	// TODO: The kick here should not be required.
	HEL_CHECK(helAcknowledgeIrq(_irq.getHandle(), kHelAckKick, 0));
	uint64_t sequence = 0;
	while(true) {
		auto await = co_await helix_ng::awaitEvent(_irq, sequence);
		HEL_CHECK(await.error());
		sequence = await.sequence();

		__sync_synchronize();

		if(logDriverStuff) {
			std::cout << "drivers/amdpcnet: got the fucking irq shit #" << sequence << "!!!" << std::endl;
		}

		auto const csr0 = csr_read(0);
		uint32_t new_csr0 = 0;

		// Handle receives
		if((csr0 & (1 << 10)) != 0) { // RINT -- completed receive frame
			if(logDriverStuff) {
				std::cout << "drivers/amdpcnet: IRQ-RINT" << std::endl;
			}
			while(!_rx.requests.empty()) {
				auto req = _rx.requests.front();
				auto i = req->index;
				//
				if((_rx.descriptors[i].data[7] & 0x80) != 0) {
					if(logDriverStuff) {
						std::cout << "drivers/amdpcnet: breaking RX loop @ " << i << " because it's not owned" << std::endl;
					}
					break;
				}
				//
				if(logDriverStuff) {
					std::cout << "drivers/amdpcnet: RX request @ " << i << "!!!" << std::endl;
				}
				memcpy(req->frame.data(), _rx.buffers[i].data(), 1520); //steal - erm... "borrow" the data
				req->frame = req->frame.subview(0, 1520);
				_rx.descriptors[i].data[7] = 0x80; //give device back the buffer
				//
				req->event.raise();
				_rx.requests.pop();
			}
			new_csr0 |= (1 << 10); //mask off RINT
		}

		// Handle transmits
		if((csr0 & (1 << 9)) != 0) { // TINT -- completed transmit frame
			if(logDriverStuff) {
				std::cout << "drivers/amdpcnet: IRQ-TINT" << std::endl;
			}
			while(!_tx.requests.empty()) {
				auto req = _tx.requests.front();
				auto i = req->index;
				// TODO: this may be bad, since it could potentially lead
				// to the tx buffer being filled before being flushed intsead
				// of splitting the workload because pcnet loves to fill
				// this buffer from top to bottom sometimes but then
				// sometimes it may want to fill it from bottom to top
				// who the fuck knows whats goig on inside the pcnet but
				// the certain thing is that if all tx descriptors are filled
				// except the first one, yknow the pcnet was plotting against my
				// code this whole time

				// Skip owned
				if((_tx.descriptors[i].data[7] & 0x80) == 0) {
					break;
				}
				//
				if(logDriverStuff) {
					std::cout << "drivers/amdpcnet: TX request @ " << i << "!!!" << std::endl;
				}
				// claim the buffer back from the evil pcnet card
				_tx.descriptors[i].data[7] = 0x00;
				//
				req->event.raise();
				_tx.requests.pop();
			}
			new_csr0 |= (1 << 9); //mask off TINT
		}

		if((csr0 & (1 << 8)) != 0) { //IDON -- initialization complete
			// Start da card!
			if(logDriverStuff) {
				std::cout << "drivers/amdpcnet: IRQ-IDON" << std::endl;
			}
			new_csr0 &= ~((1 << 0) | (1 << 2)); //clear stop and init bit
			new_csr0 |= (1 << 1); //set start bit
			new_csr0 |= (1 << 8); //mask off IDON
		}
		new_csr0 |= (1 << 6); //set interrupt enable
		csr_write(0, new_csr0);

		if(new_csr0 != csr0 && logDriverStuff) {
			std::cout << "drivers/amdpcnet: CSR0(old)" << (void*)(uintptr_t)csr0 << " != (new)" << (void*)(uintptr_t)new_csr0 << " !!!" << std::endl;
		}

		//auto csr4 = csr_read(4) | 0x022A;
		//csr_write(4, csr4);

		HEL_CHECK(helAcknowledgeIrq(_irq.getHandle(), kHelAckAcknowledge, sequence));
	}
}
} // namespace

namespace nic::amdpcnet {
std::shared_ptr<nic::Link> makeShared(protocols::hw::Device device) {
	return std::make_shared<AmdPcNetNic>(std::move(device));
}
} // namespace nic::amdpcnet
