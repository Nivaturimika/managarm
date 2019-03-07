#ifndef THOR_ARCH_X86_PAGING_HPP
#define THOR_ARCH_X86_PAGING_HPP

#include <atomic>
#include <frg/list.hpp>
#include <frigg/algorithm.hpp>
#include <frigg/smart_ptr.hpp>
#include "../../generic/types.hpp"

namespace thor {

void initializePhysicalAccess();

enum {
	kPageSize = 0x1000,
	kPageShift = 12
};

struct PageAccessor {
	friend void swap(PageAccessor &a, PageAccessor &b) {
		using frigg::swap;
		swap(a._pointer, b._pointer);
	}

	PageAccessor()
	: _pointer{nullptr} { }

	PageAccessor(PhysicalAddr physical) {
		assert(!(physical & (kPageSize - 1)));
		assert(physical < 0x4000'0000'0000);
		_pointer = reinterpret_cast<void *>(0xFFFF'8000'0000'0000 + physical);
	}

	PageAccessor(const PageAccessor &) = delete;
	
	PageAccessor(PageAccessor &&other)
	: PageAccessor{} {
		swap(*this, other);
	}

	~PageAccessor() { }
	
	PageAccessor &operator= (PageAccessor other) {
		swap(*this, other);
		return *this;
	}

	void *get() {
		return _pointer;
	}

private:
	void *_pointer;
};

struct ShootNode {
	friend struct PageSpace;
	friend struct PageBinding;

	VirtualAddr address;
	size_t size;

	void (*shotDown)(ShootNode *node);

private:
	uint64_t _sequence;

	std::atomic<unsigned int> _bindingsToShoot;

	frg::default_list_hook<ShootNode> _queueNode;
};

struct PageSpace;
struct PageBinding;

static constexpr int maxPcidCount = 8;

// Per-CPU context for paging.
struct PageContext {
	friend struct PageBinding;

	PageContext();

	PageContext(const PageContext &) = delete;
	
	PageContext &operator= (const PageContext &) = delete;

private:
	// Timestamp for the LRU mechansim of PCIDs.
	uint64_t _nextStamp;

	// Current primary binding (i.e. the currently active PCID).
	PageBinding *_primaryBinding;
};

struct PageBinding {
	PageBinding();

	PageBinding(const PageBinding &) = delete;
	
	PageBinding &operator= (const PageBinding &) = delete;

	frigg::SharedPtr<PageSpace> boundSpace() {
		return _boundSpace.grab();
	}

	void setupPcid(int pcid) {
		assert(!_pcid);
		_pcid = pcid;
	}

	uint64_t primaryStamp() {
		return _primaryStamp;
	}

	void makePrimary();

	void rebind(frigg::SharedPtr<PageSpace> space);

	void shootdown();

private:
	int _pcid;

	// TODO: Once we can use libsmarter in the kernel, we should make this a shared_ptr
	//       to the PageSpace (that does *not* prevent the PageSpace from becoming
	//       "unactivatable".
	frigg::WeakPtr<PageSpace> _boundSpace;

	bool _wasRebound;

	uint64_t _primaryStamp;

	uint64_t _alreadyShotSequence;
};

struct PageSpace {
	static void activate(frigg::SharedPtr<PageSpace> space);

	friend struct PageBinding;

	PageSpace(PhysicalAddr root_table);

	PhysicalAddr rootTable() {
		return _rootTable;
	}

	void submitShootdown(ShootNode *node);

private:
	PhysicalAddr _rootTable;

	frigg::TicketLock _mutex;
	
	unsigned int _numBindings;

	uint64_t _shootSequence;

	frg::intrusive_list<
		ShootNode,
		frg::locate_member<
			ShootNode,
			frg::default_list_hook<ShootNode>,
			&ShootNode::_queueNode
		>
	> _shootQueue;
};

namespace page_mode {
	static constexpr uint32_t remap = 1;
}

enum class PageMode {
	null,
	normal,
	remap
};

namespace page_access {
	static constexpr uint32_t write = 1;
	static constexpr uint32_t execute = 2;
}

enum class CachingMode {
	null,
	uncached,
	writeCombine,
	writeThrough,
	writeBack,
};

struct KernelPageSpace : PageSpace {
public:
	static void initialize(PhysicalAddr pml4_address);

	static KernelPageSpace &global();

	// TODO: This should be private.
	explicit KernelPageSpace(PhysicalAddr pml4_address);
	
	KernelPageSpace(const KernelPageSpace &) = delete;
	
	KernelPageSpace &operator= (const KernelPageSpace &) = delete;

	void mapSingle4k(VirtualAddr pointer, PhysicalAddr physical,
			uint32_t flags, CachingMode caching_mode);
	PhysicalAddr unmapSingle4k(VirtualAddr pointer);

private:
	frigg::TicketLock _mutex;
};

struct ClientPageSpace : PageSpace {
public:
	ClientPageSpace();
	
	ClientPageSpace(const ClientPageSpace &) = delete;
	
	~ClientPageSpace();

	ClientPageSpace &operator= (const ClientPageSpace &) = delete;

	void mapSingle4k(VirtualAddr pointer, PhysicalAddr physical, bool user_access,
			uint32_t flags, CachingMode caching_mode);
	void unmapRange(VirtualAddr pointer, size_t size, PageMode mode);
	bool isMapped(VirtualAddr pointer);

private:
	frigg::TicketLock _mutex;
};

void invalidatePage(const void *address);

} // namespace thor

#endif // THOR_ARCH_X86_PAGING_HPP
