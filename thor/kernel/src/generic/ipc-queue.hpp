#ifndef THOR_GENERIC_IPCQUEUE_HPP
#define THOR_GENERIC_IPCQUEUE_HPP

#include <frg/list.hpp>
#include "accessors.hpp"
#include "cancel.hpp"
#include "kernel_heap.hpp"
#include "../arch/x86/ints.hpp"

namespace thor {

// NOTE: The following structs mirror the Hel{Queue,Element} structs.
// They must be kept in sync!

static const int kHeadMask = 0xFFFFFF;
static const int kHeadWaiters = (1 << 24);

struct QueueStruct {
	int headFutex;
	unsigned int elementLimit;
	unsigned int sizeShift;
	char padding[4];
	int indexQueue[];
};

static const int kProgressMask = 0xFFFFFF;
static const int kProgressWaiters = (1 << 24);
static const int kProgressDone = (1 << 25);

struct ChunkStruct {
	int progressFutex;
	char padding[4];
	char buffer[];
};

struct ElementStruct {
	unsigned int length;
	unsigned int reserved;
	void *context;
};

struct QueueSource {
	void *pointer;
	size_t size;
	const QueueSource *link;
};

struct QueueNode {
	friend struct UserQueue;

	QueueNode()
	: _context{0}, _source{nullptr} { }

	// Users of UserQueue::submit() have to set this up first.
	void setupContext(uintptr_t context) {
		_context = context;
	}
	void setupSource(QueueSource *source) {
		_source = source;
	}

	virtual void complete();
	
private:
	uintptr_t _context;
	const QueueSource *_source;

	frg::default_list_hook<QueueNode> _queueNode;
};

struct UserQueue : CancelRegistry, private FutexNode {
private:
	using Address = uintptr_t;

	using NodeList = frg::intrusive_list<
		QueueNode,
		frg::locate_member<
			QueueNode,
			frg::default_list_hook<QueueNode>,
			&QueueNode::_queueNode
		>
	>;

	using Mutex = frigg::TicketLock;

	struct Chunk {
		Chunk()
		: pointer{nullptr} { }

		Chunk(frigg::SharedPtr<AddressSpace> space_, void *pointer_)
		: space{frigg::move(space_)}, pointer{pointer_}, bufferSize{4096} { }

		// Pointer (+ address space) to queue chunk struct.
		frigg::SharedPtr<AddressSpace> space;
		void *pointer;

		// Size of the chunk's buffer.
		size_t bufferSize;
	};

public:
	UserQueue(frigg::SharedPtr<AddressSpace> space, void *pointer);

	UserQueue(const UserQueue &) = delete;

	UserQueue &operator= (const UserQueue &) = delete;

	void setupChunk(size_t index, frigg::SharedPtr<AddressSpace> space, void *pointer);

	void submit(QueueNode *node);

private:
	void onWake() override;

	void _progress();
	void _advanceChunk();
	void _retireChunk();
	bool _waitHeadFutex();
	void _wakeProgressFutex(bool done);

private:	
	Mutex _mutex;

	// Pointer (+ address space) to queue head struct.
	frigg::SharedPtr<AddressSpace> _space;
	void *_pointer;
	
	int _sizeShift;

	AcquireNode _acquireNode;

	// Accessors for the queue header.
	ForeignSpaceAccessor _queuePin;
	DirectSpaceAccessor<QueueStruct> _queueAccessor;

	bool _waitInFutex;

	// Points to the chunk that we're currently writing.
	Chunk *_currentChunk;

	// Accessors for the current chunk.
	ForeignSpaceAccessor _chunkPin;
	DirectSpaceAccessor<ChunkStruct> _chunkAccessor;

	// Progress into the current chunk.
	int _currentProgress;

	// Index into the queue that we're currently processing.
	int _nextIndex;

	frigg::Vector<Chunk, KernelAlloc> _chunks;

	frg::intrusive_list<
		QueueNode,
		frg::locate_member<
			QueueNode,
			frg::default_list_hook<QueueNode>,
			&QueueNode::_queueNode
		>
	> _nodeQueue;
};

} // namespace thor

#endif // THOR_GENERIC_IPCQUEUE_HPP
