// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

#include "emu.h"
#include "emumem_mud.h"
#include "emumem_hea.h"
#include "emumem_heu.h"
#include "emumem_heun.h"
#include "emumem_hedr.h"

template<int HighBits, int Width, int AddrShift, int Endian> handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::handler_entry_read_dispatch(address_space *space, const handler_entry::range &init, handler_entry_read<Width, AddrShift, Endian> *handler) : handler_entry_read<Width, AddrShift, Endian>(space, handler_entry::F_DISPATCH)
{
	if (!handler)
		handler = space->get_unmap_r<Width, AddrShift, Endian>();
	handler->ref(COUNT);
	for(unsigned int i=0; i != COUNT; i++) {
		m_dispatch[i] = handler;
		m_ranges[i] = init;
	}
}

template<int HighBits, int Width, int AddrShift, int Endian> handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::~handler_entry_read_dispatch()
{
	for(unsigned int i=0; i != COUNT; i++)
		m_dispatch[i]->unref();
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::enumerate_references(handler_entry::reflist &refs) const
{
	for(unsigned int i=0; i != COUNT; i++)
		refs.add(m_dispatch[i]);
}

template<int HighBits, int Width, int AddrShift, int Endian> typename handler_entry_size<Width>::uX handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::read(offs_t offset, uX mem_mask)
{
	return m_dispatch[(offset >> LowBits) & BITMASK]->read(offset, mem_mask);
}

template<int HighBits, int Width, int AddrShift, int Endian> void *handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::get_ptr(offs_t offset) const
{
	return m_dispatch[(offset >> LowBits) & BITMASK]->get_ptr(offset);
}

template<int HighBits, int Width, int AddrShift, int Endian> std::string handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::name() const
{
	return "dispatch";
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::lookup(offs_t address, offs_t &start, offs_t &end, handler_entry_read<Width, AddrShift, Endian> *&handler) const
{
	offs_t slot = (address >> LowBits) & BITMASK;
	auto h = m_dispatch[slot];
	if(h->is_dispatch())
		h->lookup(address, start, end, handler);
	else {
		start = m_ranges[slot].start;
		end = m_ranges[slot].end;
		handler = h;
	}
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::range_cut_before(offs_t address, int start)
{
	while(--start >= 0) {
		if(LowBits > -AddrShift && m_dispatch[start]->is_dispatch()) {
			static_cast<handler_entry_read_dispatch<LowBits, Width, AddrShift, Endian> *>(m_dispatch[start])->range_cut_before(address);
			break;
		}
		if(m_ranges[start].end <= address)
			break;
		m_ranges[start].end = address;
	}
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::range_cut_after(offs_t address, int start)
{
	while(++start < COUNT) {
		if(LowBits > -AddrShift && m_dispatch[start]->is_dispatch()) {
			static_cast<handler_entry_read_dispatch<LowBits, Width, AddrShift, Endian> *>(m_dispatch[start])->range_cut_after(address);
			break;
		}
		if(m_ranges[start].start >= address)
			break;
		m_ranges[start].start = address;
	}
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::populate_nomirror_subdispatch(offs_t entry, offs_t start, offs_t end, offs_t ostart, offs_t oend, handler_entry_read<Width, AddrShift, Endian> *handler)
{
	auto cur = m_dispatch[entry];
	if(cur->is_dispatch())
		cur->populate_nomirror(start, end, ostart, oend, handler);
	else {
		auto subdispatch = new handler_entry_read_dispatch<LowBits, Width, AddrShift, Endian>(handler_entry::m_space, m_ranges[entry], cur);
		cur->unref();
		m_dispatch[entry] = subdispatch;
		subdispatch->populate_nomirror(start, end, ostart, oend, handler);
	}
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::populate_nomirror(offs_t start, offs_t end, offs_t ostart, offs_t oend, handler_entry_read<Width, AddrShift, Endian> *handler)
{
	offs_t start_entry = start >> LowBits;
	offs_t end_entry = end >> LowBits;
	range_cut_before(ostart-1, start_entry);
	range_cut_after(oend+1, end_entry);

	if(LowBits <= -AddrShift) {
		handler->ref(end_entry - start_entry);
		for(offs_t ent = start_entry; ent <= end_entry; ent++) {
			m_dispatch[ent]->unref();
			m_dispatch[ent] = handler;
			m_ranges[ent].set(ostart, oend);
		}

	} else if(start_entry == end_entry) {
		if(!(start & LOWMASK) && (end & LOWMASK) == LOWMASK) {
			m_dispatch[start_entry]->unref();
			m_dispatch[start_entry] = handler;
			m_ranges[start_entry].set(ostart, oend);
		} else {
			populate_nomirror_subdispatch(start_entry, start & LOWMASK, end & LOWMASK, ostart, oend, handler);
		}

	} else {
		if(start & LOWMASK) {
			populate_nomirror_subdispatch(start_entry, start & LOWMASK, LOWMASK, ostart, oend, handler);
			start_entry++;
		}
		if((end & LOWMASK) != LOWMASK) {
			populate_nomirror_subdispatch(end_entry, 0, end & LOWMASK, ostart, oend, handler);
			end_entry--;
		}

		if(start_entry <= end_entry) {
			handler->ref(end_entry - start_entry);
			for(offs_t ent = start_entry; ent <= end_entry; ent++) {
				m_dispatch[ent]->unref();
				m_dispatch[ent] = handler;
				m_ranges[ent].set(ostart, oend);
			}
		}
	}
}
template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::populate_mirror_subdispatch(offs_t entry, offs_t start, offs_t end, offs_t ostart, offs_t oend, offs_t mirror, handler_entry_read<Width, AddrShift, Endian> *handler)
{
	auto cur = m_dispatch[entry];
	if(cur->is_dispatch())
		cur->populate_mirror(start, end, ostart, oend, mirror, handler);
	else {
		auto subdispatch = new handler_entry_read_dispatch<LowBits, Width, AddrShift, Endian>(handler_entry::m_space, m_ranges[entry], cur);
		cur->unref();
		m_dispatch[entry] = subdispatch;
		subdispatch->populate_mirror(start, end, ostart, oend, mirror, handler);
	}
}


template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::populate_mirror(offs_t start, offs_t end, offs_t ostart, offs_t oend, offs_t mirror, handler_entry_read<Width, AddrShift, Endian> *handler)
{
	offs_t hmirror = mirror & HIGHMASK;
	offs_t lmirror = mirror & LOWMASK;

	if(lmirror) {
		// If lmirror is non-zero, then each mirror instance is a single entry
		offs_t add = 1 + ~hmirror;
		offs_t offset = 0;
		offs_t base_entry = start >> LowBits;
		start &= LOWMASK;
		end &= LOWMASK;
		do {
			populate_mirror_subdispatch(base_entry | (offset >> LowBits), start, end, ostart | offset, oend | offset, lmirror, handler);
			offset = (offset + add) & hmirror;
		} while(offset);
	} else {
		// If lmirror is zero, call the nomirror version as needed
		offs_t add = 1 + ~hmirror;
		offs_t offset = 0;
		do {
			if(offset)
				handler->ref();
			populate_nomirror(start | offset, end | offset, ostart | offset, oend | offset, handler);
			offset = (offset + add) & hmirror;
		} while(offset);
	}
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::mismatched_patch(const memory_units_descriptor<Width, AddrShift, Endian> &descriptor, u8 rkey, std::vector<mapping> &mappings, handler_entry_read<Width, AddrShift, Endian> *&target)
{
	u8 ukey = descriptor.rkey_to_ukey(rkey);
	handler_entry_read<Width, AddrShift, Endian> *original = target->is_units() ? target : nullptr;
	handler_entry_read<Width, AddrShift, Endian> *replacement = nullptr;
	for(const auto &p : mappings)
		if(p.ukey == ukey && p.original == original) {
			replacement = p.patched;
			break;
		}
	if(!replacement) {
		if(original)
			replacement = new handler_entry_read_units<Width, AddrShift, Endian>(descriptor, ukey, static_cast<handler_entry_read_units<Width, AddrShift, Endian> *>(original));
		else
			replacement = new handler_entry_read_units<Width, AddrShift, Endian>(descriptor, ukey, inh::m_space);

		mappings.emplace_back(mapping{ original, replacement, ukey });
	} else
		replacement->ref();
	target->unref();
	target = replacement;
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::populate_mismatched_nomirror_subdispatch(offs_t entry, offs_t start, offs_t end, offs_t ostart, offs_t oend, const memory_units_descriptor<Width, AddrShift, Endian> &descriptor, u8 rkey, std::vector<mapping> &mappings)
{
	auto cur = m_dispatch[entry];
	if(cur->is_dispatch())
		cur->populate_mismatched_nomirror(start, end, ostart, oend, descriptor, rkey, mappings);
	else {
		auto subdispatch = new handler_entry_read_dispatch<LowBits, Width, AddrShift, Endian>(handler_entry::m_space, m_ranges[entry], cur);
		cur->unref();
		m_dispatch[entry] = subdispatch;
		subdispatch->populate_mismatched_nomirror(start, end, ostart, oend, descriptor, rkey, mappings);
	}
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::populate_mismatched_nomirror(offs_t start, offs_t end, offs_t ostart, offs_t oend, const memory_units_descriptor<Width, AddrShift, Endian> &descriptor, u8 rkey, std::vector<mapping> &mappings)
{
	offs_t start_entry = start >> LowBits;
	offs_t end_entry = end >> LowBits;
	range_cut_before(ostart-1, start_entry);
	range_cut_after(oend+1, end_entry);

	if(LowBits <= -AddrShift) {
		for(offs_t ent = start_entry; ent <= end_entry; ent++) {
			u8 rkey1 = rkey;
			if(ent != start_entry)
				rkey1 &= ~handler_entry::START;
			if(ent != end_entry)
				rkey1 &= ~handler_entry::END;
			mismatched_patch(descriptor, rkey1, mappings, m_dispatch[ent]);
			m_ranges[ent].set(ostart, oend);
		}

	} else if(start_entry == end_entry) {
		if(!(start & LOWMASK) && (end & LOWMASK) == LOWMASK) {
			mismatched_patch(descriptor, rkey, mappings, m_dispatch[start_entry]);
			m_ranges[start_entry].set(ostart, oend);
		} else
			populate_mismatched_nomirror_subdispatch(start_entry, start & LOWMASK, end & LOWMASK, ostart, oend, descriptor, rkey, mappings);

	} else {
		if(start & LOWMASK) {
			populate_mismatched_nomirror_subdispatch(start_entry, start & LOWMASK, LOWMASK, ostart, oend, descriptor, rkey & ~handler_entry::END, mappings);
			start_entry++;
			rkey &= ~handler_entry::START;
		}
		if((end & LOWMASK) != LOWMASK) {
			populate_mismatched_nomirror_subdispatch(end_entry, 0, end & LOWMASK, ostart, oend, descriptor, rkey & ~handler_entry::START, mappings);
			end_entry--;
			rkey &= ~handler_entry::END;
		}

		if(start_entry <= end_entry) {
			for(offs_t ent = start_entry; ent <= end_entry; ent++) {
				u8 rkey1 = rkey;
				if(ent != start_entry)
					rkey1 &= ~handler_entry::START;
				if(ent != end_entry)
					rkey1 &= ~handler_entry::END;
				mismatched_patch(descriptor, rkey1, mappings, m_dispatch[ent]);
				m_ranges[ent].set(ostart, oend);
			}
		}
	}
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::populate_mismatched_mirror_subdispatch(offs_t entry, offs_t start, offs_t end, offs_t ostart, offs_t oend, offs_t mirror, const memory_units_descriptor<Width, AddrShift, Endian> &descriptor, std::vector<mapping> &mappings)
{
	auto cur = m_dispatch[entry];
	if(cur->is_dispatch())
		cur->populate_mismatched_mirror(start, end, ostart, oend, mirror, descriptor, mappings);
	else {
		auto subdispatch = new handler_entry_read_dispatch<LowBits, Width, AddrShift, Endian>(handler_entry::m_space, m_ranges[entry], cur);
		cur->unref();
		m_dispatch[entry] = subdispatch;
		subdispatch->populate_mismatched_mirror(start, end, ostart, oend, mirror, descriptor, mappings);
	}
}

template<int HighBits, int Width, int AddrShift, int Endian> void handler_entry_read_dispatch<HighBits, Width, AddrShift, Endian>::populate_mismatched_mirror(offs_t start, offs_t end, offs_t ostart, offs_t oend, offs_t mirror, const memory_units_descriptor<Width, AddrShift, Endian> &descriptor, std::vector<mapping> &mappings)
{
	offs_t hmirror = mirror & HIGHMASK;
	offs_t lmirror = mirror & LOWMASK;

	if(lmirror) {
		// If lmirror is non-zero, then each mirror instance is a single entry
		offs_t add = 1 + ~hmirror;
		offs_t offset = 0;
		offs_t base_entry = start >> LowBits;
		start &= LOWMASK;
		end &= LOWMASK;
		do {
			populate_mismatched_mirror_subdispatch(base_entry | (offset >> LowBits), start, end, ostart | offset, oend | offset, lmirror, descriptor, mappings);
			offset = (offset + add) & hmirror;
		} while(offset);
	} else {
		// If lmirror is zero, call the nomirror version as needed
		offs_t add = 1 + ~hmirror;
		offs_t offset = 0;
		do {
			populate_mismatched_nomirror(start | offset, end | offset, ostart | offset, oend | offset, descriptor, handler_entry::START|handler_entry::END, mappings);
			offset = (offset + add) & hmirror;
		} while(offset);
	}
}




template class handler_entry_read_dispatch< 1, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 1, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 2, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 2, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 3, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 0,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 0,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 0,  0, ENDIANNESS_BIG>;

template class handler_entry_read_dispatch< 1, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 1, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 2, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 2, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 3, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 1,  3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 1,  3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 1,  3, ENDIANNESS_BIG>;


template class handler_entry_read_dispatch< 1, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 1, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 2, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 2, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 3, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 1,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 1,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 1,  0, ENDIANNESS_BIG>;


template class handler_entry_read_dispatch< 1, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 1, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 2, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 2, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 3, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 1, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 1, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 1, -1, ENDIANNESS_BIG>;


template class handler_entry_read_dispatch< 2, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 2, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 3, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 2,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 2,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 2,  0, ENDIANNESS_BIG>;


template class handler_entry_read_dispatch< 2, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 2, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 3, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 2, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 2, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 2, -1, ENDIANNESS_BIG>;


template class handler_entry_read_dispatch< 2, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 2, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 3, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 2, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 2, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 2, -2, ENDIANNESS_BIG>;


template class handler_entry_read_dispatch< 3, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 3,  0, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 3,  0, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 3,  0, ENDIANNESS_BIG>;



template class handler_entry_read_dispatch< 3, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 3, -1, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 3, -1, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 3, -1, ENDIANNESS_BIG>;


template class handler_entry_read_dispatch< 3, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 3, -2, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 3, -2, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 3, -2, ENDIANNESS_BIG>;


template class handler_entry_read_dispatch< 3, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 3, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 4, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 4, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 5, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 5, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 6, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 6, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 7, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 7, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 8, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 8, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch< 9, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch< 9, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<10, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<10, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<11, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<11, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<12, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<12, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<13, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<13, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<14, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<14, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<15, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<15, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<16, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<16, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<17, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<17, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<18, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<18, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<19, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<19, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<20, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<20, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<21, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<21, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<22, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<22, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<23, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<23, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<24, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<24, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<25, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<25, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<26, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<26, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<27, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<27, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<28, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<28, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<29, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<29, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<31, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<31, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<30, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<30, 3, -3, ENDIANNESS_BIG>;
template class handler_entry_read_dispatch<32, 3, -3, ENDIANNESS_LITTLE>;
template class handler_entry_read_dispatch<32, 3, -3, ENDIANNESS_BIG>;
