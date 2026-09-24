#include "hardware.hpp"
#include "iface.hpp"
#include <fcntl.h>
#include <cmath>
#include <cassert>
#include <chrono>
#include <deque>
#include <memory>
#include <vector>
#include <algorithm>
#include <sys/mman.h>

#ifdef VERILATOR_BUILD
#include "marga_model.hpp"
extern marga_model *mm;
#endif

// variadic macro for debugging enable/disable
// #define debug_printf(...) printf(__VA_ARGS__)
#define debug_printf(...)

// Normal (non-streaming) runs must retain all RX data until the final reply,
// but must not repeatedly relocate a growing monolithic vector in the FIFO
// read path.  Data is consequently stored in independent fixed-size blocks.
// A run may use any number of blocks.
static const size_t RX_BLOCK_POINTS = 16384;

struct rx_block {
	std::unique_ptr<uint32_t[]> i{new uint32_t[RX_BLOCK_POINTS]};
	std::unique_ptr<uint32_t[]> q{new uint32_t[RX_BLOCK_POINTS]};
	size_t used = 0;
};

class rx_block_store {
public:
	void append(uint32_t i_value, uint32_t q_value) {
		if (_blocks.empty() || _blocks.back()->used == RX_BLOCK_POINTS) {
			_blocks.push_back(std::make_unique<rx_block>());
		}
		rx_block &block = *_blocks.back();
		block.i[block.used] = i_value;
		block.q[block.used] = q_value;
		++block.used;
		++_size;
	}

	size_t size() const { return _size; }
	bool empty() const { return _size == 0; }
	const std::deque<std::unique_ptr<rx_block>>& blocks() const { return _blocks; }

private:
	std::deque<std::unique_ptr<rx_block>> _blocks;
	size_t _size = 0;
};

hardware::hardware() {
	init_mem();
}

hardware::~hardware() {
}

int hardware::run_request(server_action &sa) {
	// A server may service several TCP connections, but only one ordinary
	// hardware request may own the FPGA at a time. emergency_stop() bypasses
	// this lock and interrupts run_seq through the abort epoch below.
	std::lock_guard<std::mutex> request_guard(_request_mutex);

	// See whether HW needs to be reconfigured
	size_t commands_present = sa.command_count();
	size_t commands_understood = 0;
	auto wr = sa.get_writer();
	int problems = 0;
	int status;
	mpack_start_map(wr, commands_present); // each command should fill in an element of the map

	if (commands_present == 0) {
		++problems;
		sa.add_error("no commands present or incorrectly formatted request");
	}

	// Halt and reset; returns true if the FSM has already halted (may take up to 2ms to halt the HDL if it's counting down
	auto hr = sa.get_command_and_start_reply("halt_and_reset", status);
	if (status == 1) {
		++commands_understood;
		halt_and_reset();

		auto exec = rd32(_exec);
		bool halted = (exec >> 24) == MAR_STATE_IDLE;
		mpack_write(wr, halted);
	}

	// Read directly from memory [TODO: implement]
	auto rm = sa.get_command_and_start_reply("read_mem", status);
	if (status == 1) {
		++commands_understood;
		mpack_write(wr, c_ok);
	}

	// FPGA clock config [TODO: understand this better]
	auto fcwa1 = sa.get_command_and_start_reply("fpga_clk", status);
	if (status == 1) {
		++commands_understood;
		// Enforce that all three words are present for FPGA clock configuration
		if (mpack_node_array_length(fcwa1) != 3) {
			sa.add_error("you only provided some FPGA clock control words; check you're providing all 3");
			mpack_write(wr, c_err); // error
		} else if (_emergency_latched.load(std::memory_order_acquire)) {
			sa.add_error("fpga_clk rejected while emergency stop is latched");
			mpack_write(wr, c_err);
		} else {
			std::lock_guard<std::mutex> io_guard(_io_mutex);
			if (_emergency_latched.load(std::memory_order_acquire)) {
				sa.add_error("fpga_clk rejected while emergency stop is latched");
				mpack_write(wr, c_err);
			} else {
			_slcr[2] = mpack_node_u32(mpack_node_array_at(fcwa1, 0));
			_slcr[92] = (_slcr[92] & ~mpack_node_u32(mpack_node_array_at(fcwa1, 1)) )
				| mpack_node_u32(mpack_node_array_at(fcwa1, 2));
			mpack_write(wr, c_ok); // okay
			}
		}
	} else if (status == -1) {
		// sa.add_error("Unknown MPack error from fpga_clk");
		// TODO: callback or similar
	}

	// Alter main control register
	auto ctrl = sa.get_command_and_start_reply("ctrl", status);
	if (status == 1) {
		++commands_understood;
		std::lock_guard<std::mutex> io_guard(_io_mutex);
		if (_emergency_latched.load(std::memory_order_acquire)) {
			sa.add_error("ctrl rejected while emergency stop is latched");
			mpack_write(wr, c_err);
		} else {
			wr32(_ctrl, mpack_node_u32(ctrl));
			mpack_write(wr, c_ok);
		}
	}

	// Command directly to the buffers
	auto dir = sa.get_command_and_start_reply("direct", status);
	if (status == 1) {
		++commands_understood;
		std::lock_guard<std::mutex> io_guard(_io_mutex);
		if (_emergency_latched.load(std::memory_order_acquire)) {
			sa.add_error("direct rejected while emergency stop is latched");
			mpack_write(wr, c_err);
		} else {
			wr32(_direct, mpack_node_u32(dir));
			// TODO: add sanity check for instruction/data type
			// TODO: implement higher-level direct commands, like 32b writes etc
			mpack_write(wr, c_ok);
		}
	}

	// Read one register
	auto regidx = sa.get_command_and_start_reply("regrd", status);
	if (status == 1) {
		++commands_understood;
		mpack_write(wr, rd32(_mar_base + mpack_node_u32(regidx) ));
	}

	// Read all registers
	sa.get_command_and_start_reply("regstatus", status);
	if (status == 1) {
		++commands_understood;
		mpack_start_array(wr, 7);

		mpack_write(wr, rd32(_exec));
		mpack_write(wr, rd32(_status));
		mpack_write(wr, rd32(_status_latch));
		mpack_write(wr, rd32(_buf_err));
		mpack_write(wr, rd32(_buf_full));
		mpack_write(wr, rd32(_buf_empty));
		mpack_write(wr, rd32(_rx_locs));

		mpack_finish_array(wr);
	}

	// Fill in marga execution memory
	// TODO: add an input offset too, to avoid having to overwrite everything every time
	auto mm = sa.get_command_and_start_reply("mar_mem", status);
	if (status == 1) {
		++commands_understood;
		char t[100];

		// uint32_t ro = *(uint32_t *)(GRAD_CTRL_REG_OFFSET);
		if ( mpack_node_bin_size(mm) <= MARGA_MEM_SIZE ) {
			size_t bytes_copied = hw_mpack_node_copy_data(mm, _mar_mem, MARGA_MEM_SIZE);
			sprintf(t, "mar mem data bytes copied: %zu", bytes_copied);
			sa.add_info(t);
			mpack_write(wr, c_ok);
		} else {
			sprintf(t, "too much mar mem data: %zu bytes > %d -- streaming not yet implemented", mpack_node_bin_size(mm), MARGA_MEM_SIZE);
			sa.add_error(t);
			mpack_write(wr, c_err);
		}
	}

	// Configure acquisition retry limit (how many times the server will poll the RX FIFO when waiting for data)
	auto arl = sa.get_command_and_start_reply("acq_rlim", status);
	if (status == 1) {
		++commands_understood;
		uint32_t rl = mpack_node_u32(arl);
		if (rl < 1000 or rl > 10000000) {
			sa.add_error("acquisition retry limit outside the range [1000, 10,000,000]; check your settings");
			mpack_write(wr, c_err);
		} else {
			_read_tries_limit = rl;
			mpack_write(wr, c_ok);
		}
	}

	// read all outstanding data from RX FIFOs
	auto rr = sa.get_command_and_start_reply("read_rx", status);
	if (status == 1) {
		++commands_understood;
		char t[100];
		std::vector<uint32_t> rx0_i, rx0_q, rx1_i, rx1_q;
		read_rx(rx0_i, rx0_q, rx1_i, rx1_q);

		// encode the RX replies
		unsigned rx0_elem = rx0_i.size(), rx1_elem = rx1_i.size();
		if (!rx0_elem and !rx1_elem) {
			mpack_write(wr, c_ok);
			sprintf(t, "no RX data received");
			sa.add_warning(t);
		} else {
			mpack_start_map(wr, (rx0_elem ? 2:0) + (rx1_elem ? 2:0));
			if (rx0_elem) {
				mpack_write_cstr(wr, "rx0_i");
				mpack_start_array(wr, rx0_elem);
				for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx0_q");
				mpack_start_array(wr, rx0_elem);
				for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_q[k]);
				mpack_finish_array(wr);
			}

			if (rx1_elem) {
				mpack_write_cstr(wr, "rx1_i");
				mpack_start_array(wr, rx1_elem);
				for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx1_q");
				mpack_start_array(wr, rx1_elem);
				for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_q[k]);
				mpack_finish_array(wr);
			}

			mpack_finish_map(wr);
		}
	}

	// Run a sequence
	auto runs = sa.get_command_and_start_reply("run_seq", status);
	if (status == 1) {
		++commands_understood;
		char t[100];

		// Support structured run control as well as the legacy raw binary value.
		// Legacy format is a raw bin; chunk_points == 0 keeps the single-shot reply.
		mpack_node_t rundata_node = runs;
		size_t chunk_points = 0;
		bool use_rx0 = true, use_rx1 = false;  // default: rx0 only (backward compatible)
		bool report_armed = false;
		// Only the structured protocol can explicitly rearm a latched server.
		// Legacy requests still run normally while no emergency stop is latched.
		bool rearm = false;
		std::string acquisition_id;
		std::string gradient_board_name;
		const bool structured_run = mpack_node_type(runs) == mpack_type_map;
		if (structured_run) {
			mpack_node_t data_node = mpack_node_map_cstr_optional(runs, "data");
			if (!mpack_node_is_missing(data_node) && mpack_node_type(data_node) == mpack_type_bin) {
				rundata_node = data_node;
			}
			mpack_node_t cp_node = mpack_node_map_cstr_optional(runs, "chunk_points");
			if (!mpack_node_is_missing(cp_node)) chunk_points = mpack_node_uint(cp_node);

			mpack_node_t rc_node = mpack_node_map_cstr_optional(runs, "rx_channels");
			if (!mpack_node_is_missing(rc_node) && mpack_node_type(rc_node) == mpack_type_array) {
				use_rx0 = false;
				use_rx1 = false;
				size_t rc_n = mpack_node_array_length(rc_node);
				for (size_t i = 0; i < rc_n; ++i) {
					unsigned ch = mpack_node_uint(mpack_node_array_at(rc_node, i));
					if (ch == 0) use_rx0 = true;
					else if (ch == 1) use_rx1 = true;
				}
			}

			mpack_node_t armed_node = mpack_node_map_cstr_optional(runs, "report_armed");
			if (!mpack_node_is_missing(armed_node)) report_armed = mpack_node_bool(armed_node);

			mpack_node_t rearm_node = mpack_node_map_cstr_optional(runs, "rearm");
			if (!mpack_node_is_missing(rearm_node)) rearm = mpack_node_bool(rearm_node);

			mpack_node_t grad_node = mpack_node_map_cstr_optional(runs, "grad_board");
			if (!mpack_node_is_missing(grad_node) && mpack_node_type(grad_node) == mpack_type_str) {
				gradient_board_name.assign(mpack_node_str(grad_node), mpack_node_strlen(grad_node));
			}

			mpack_node_t id_node = mpack_node_map_cstr_optional(runs, "acquisition_id");
			if (!mpack_node_is_missing(id_node) && mpack_node_type(id_node) == mpack_type_str) {
				acquisition_id.assign(mpack_node_str(id_node), mpack_node_strlen(id_node));
			}
		}
		uint64_t run_epoch = 0;
		if (!prepare_run(rearm, acquisition_id, run_epoch)) {
			mpack_start_map(wr, 2);
			mpack_write_cstr(wr, "final");
			mpack_write(wr, true);
			mpack_write_cstr(wr, "aborted");
			mpack_write(wr, true);
			mpack_finish_map(wr);
			sa.add_warning("run_seq rejected: emergency stop is latched or acquisition_id was already aborted");
		} else {
			if (!gradient_board_name.empty()) set_gradient_board(gradient_board_name);
			// A stale non-idle state is made safe before loading the new scan.
			auto exec = rd32(_exec);
			if ( (exec >> 24) != MAR_STATE_IDLE) {
				sprintf(t, "mar FSM was not idle when the run began");
				sa.add_warning(t);
				emergency_stop_result reset_result;
				{
					std::lock_guard<std::mutex> io_guard(_io_mutex);
					// A Stop may have won after prepare_run(). Never let stale-state
					// cleanup release its hardware latch with ctrl=idle.
					if (!run_aborted(run_epoch)) {
						force_safe_state(false, reset_result);
					}
				}
			}

		const size_t total_bytes_to_copy = mpack_node_bin_size(rundata_node);
		const char *rundata = (char *)mpack_node_bin_data(rundata_node);
		size_t mem_offset = 0;

		if (chunk_points > 0 || report_armed) {
			sa.start_chunks(chunk_points);
		}

		// initially fill the marga memory
		if (total_bytes_to_copy <= MARGA_MEM_SIZE) {
			hw_memcpy(_mar_mem, rundata, total_bytes_to_copy);
			mem_offset += total_bytes_to_copy;
		} else {
			// first time, wrap around back to the start again,
			// so don't need to update marga memory ptr
			hw_memcpy(_mar_mem, rundata, MARGA_MEM_SIZE);
			mem_offset += MARGA_MEM_SIZE;
		}
		// prepare main FSM control loop

		// max bytes to copy into _mar_mem at a time (would block for this long)
		const unsigned max_bytes_to_copy = 128;

		// check execution state for issues periodically
		const unsigned execution_check_interval = 20;

		// monitor buffer-low and buffer-underrun events
		unsigned mem_buffer_low = 0;
		bool mem_buffer_underrun = false;
		bool finished = false;
		bool aborted = false;
		bool armed_reported = false;

		// monitor RX-nearly-full and RX-full events
		unsigned rx_nearly_full = 0;
		bool rx_full = false;
		size_t rx_full_mem_loc = 0;

		// monitor output buffers
		uint32_t buf_full = 0, buf_err = 0;

		// monitor gradient issues
		bool ocra1_data_lost = false, ocra1_err = false, fhdo_err = false;

		// Chunked runs retain only the current unsent chunk in vectors because
		// server_action::send_intermediate_chunk consumes vector ranges.  Normal
		// runs use segmented storage so no historical RX data is relocated.
		std::vector<uint32_t> rx0_i, rx0_q, rx1_i, rx1_q;
		rx_block_store rx0_blocks, rx1_blocks;
		if (chunk_points > 0) {
			// One chunk plus a single read_rx() overshoot is all that remains in
			// memory in streaming mode.  Reserve it before the FPGA starts so the
			// first chunk cannot grow a vector in the real-time read path.
			size_t chunk_capacity = chunk_points + _max_rx_reads_per_loop;
			if (use_rx0) {
				rx0_i.reserve(chunk_capacity);
				rx0_q.reserve(chunk_capacity);
			}
			if (use_rx1) {
				rx1_i.reserve(chunk_capacity);
				rx1_q.reserve(chunk_capacity);
			}
		}
		unsigned rx_reads_per_loop = _min_rx_reads_per_loop;

		// chunked streaming state (chunk boundaries are determined by the active RX channels)
		size_t chunk_index = 0;

		auto emit_chunks = [&]() {
			if (chunk_points == 0) return;
			size_t avail = 0;
			if (use_rx0 && use_rx1) avail = std::min(rx0_i.size(), rx1_i.size());
			else if (use_rx0) avail = rx0_i.size();
			else if (use_rx1) avail = rx1_i.size();
			while (avail >= chunk_points) {
				sa.send_intermediate_chunk(rx0_i, rx0_q, rx1_i, rx1_q, 0, chunk_points, chunk_index++);
				// Keep only the small tail which arrived after the completed chunk.
				// erase() does not reduce vector capacity, so this does not cause a
				// later reallocation.  It moves at most one read_rx() overshoot in
				// normal operation, rather than all historical acquisition data.
				if (use_rx0) {
					rx0_i.erase(rx0_i.begin(), rx0_i.begin() + chunk_points);
					rx0_q.erase(rx0_q.begin(), rx0_q.begin() + chunk_points);
				}
				if (use_rx1) {
					rx1_i.erase(rx1_i.begin(), rx1_i.begin() + chunk_points);
					rx1_q.erase(rx1_q.begin(), rx1_q.begin() + chunk_points);
				}
				avail -= chunk_points;
			}
		};

		auto read_rx_to_blocks = [&](const unsigned max_reads) {
			uint32_t rxlocs = rd32(_rx_locs);
			int fifo0_locs = rxlocs & 0xffff, fifo1_locs = rxlocs >> 16;
			unsigned reads = 0;

			while (fifo0_locs > 0 || fifo1_locs > 0) {
				if (reads >= max_reads) break;

				bool read_fifo0 = false, read_fifo1 = false;
				if (fifo1_locs > 2 * fifo0_locs) {
					read_fifo1 = true;
				} else if (fifo0_locs > 2 * fifo1_locs) {
					read_fifo0 = true;
				} else {
					read_fifo0 = true;
					read_fifo1 = true;
				}

				if (read_fifo0) {
					uint32_t q = rd32(_rx0_q_data);
					uint32_t i = rd32(_rx0_i_data); // pop FIFO
					rx0_blocks.append(i, q);
					++reads;
					--fifo0_locs;
				}
				if (read_fifo1) {
					uint32_t q = rd32(_rx1_q_data);
					uint32_t i = rd32(_rx1_i_data); // pop FIFO
					rx1_blocks.append(i, q);
					++reads;
					--fifo1_locs;
				}
			}

			return std::max(fifo0_locs, fifo1_locs);
		};

		// Start only if no emergency raced with preparation/download. The
		// check and control-register write are serialized with emergency_stop().
		aborted = !start_run(run_epoch);

		// main copying and reading loop
		unsigned execution_loops = 0;
		// track program counter execution
		size_t pc_offset = 0, old_pc_hw = 0, old_pc = 0;
		while (not finished && !aborted) {
			if (run_aborted(run_epoch)) {
				aborted = true;
				break;
			}
			uint32_t exec = rd32(_exec);
			uint32_t state = exec >> 24;
			size_t pc_hw = (exec & 0xffffff) << 2; // convert to mem offset

			if (report_armed && !armed_reported && state == MAR_STATE_TRIG_FOREVER) {
				sa.send_intermediate_armed();
				armed_reported = true;
			}

			if (pc_hw + 16 < old_pc_hw) {
				// extra 16 since sometimes the PC can
				// go back a few instructions when
				// waiting/pausing
				debug_printf("PC wrapped\n");
				pc_offset += MARGA_MEM_SIZE;
			}
			if (pc_hw > old_pc_hw + MARGA_MEM_SIZE -16) {
				// PC has jumped backward due to a wrap from waiting/pausing 
				debug_printf("PC Inversely wrapped: Loop %u | Mem: 0x%zx | RxReads: %u | Exec: %u | PC_HW: 0x%zx | OLD_PC_HW: 0x%zx\n",
                   execution_loops, mem_offset, rx_reads_per_loop, exec, pc_hw, old_pc_hw);
				pc_offset -= MARGA_MEM_SIZE;
			}
			old_pc_hw = pc_hw;
			
			size_t pc = pc_hw + pc_offset;
			// unwrap pc
			debug_printf("pc_hw %zu, old_pc_hw %zu, pc %zu\n", pc_hw, old_pc_hw, pc);

			size_t total_bytes_remaining = total_bytes_to_copy - mem_offset;
			int bytes_to_copy = 0;
			if (total_bytes_remaining != 0) {
				// check whether data needs copying in this round
				//
				// -16 to keep a buffer zone of 4 un-copied
				// instructions before PC location
				bytes_to_copy = pc - mem_offset + MARGA_MEM_SIZE - 16;
				if (bytes_to_copy > (int)total_bytes_remaining) bytes_to_copy = total_bytes_remaining;
			}

			if (bytes_to_copy > 0) {
				// Monitor memory reserve during streaming
				if (pc > mem_offset) {
					// memory reserve has run dry
					bytes_to_copy = 0;
					mem_buffer_underrun = true;
					debug_printf("mem buf underrun\n");
					break;
				} else if (mem_offset - pc < MARGA_MEM_SIZE/4) {
					// memory reserve only 1/4 full
					++mem_buffer_low;
					debug_printf("mem buf low\n");
				}

				if (bytes_to_copy > (int)max_bytes_to_copy) {
					// avoid starving the RX for time
					bytes_to_copy = max_bytes_to_copy;
				}

				size_t local_mem_offset = mem_offset & MARGA_MEM_MASK;

				// check whether this copy will wrap
				if ( local_mem_offset + bytes_to_copy > MARGA_MEM_SIZE) { // wrapping: copy in two parts
					int first_bytes = MARGA_MEM_SIZE - local_mem_offset;
					int second_bytes = bytes_to_copy - first_bytes;
					hw_memcpy(_mar_mem + local_mem_offset,
					          rundata + mem_offset, first_bytes);
					mem_offset += first_bytes;
					hw_memcpy(_mar_mem, rundata + mem_offset, second_bytes);
					mem_offset += second_bytes;
				} else { // no wrapping
					debug_printf("hw_memcpy: %zu, %zu, %u\n", local_mem_offset, mem_offset, bytes_to_copy);
					auto k = (char *)hw_memcpy(_mar_mem + local_mem_offset,
					          rundata + mem_offset, bytes_to_copy);
					debug_printf("bytes copied: %zu\n", k - _mar_mem - local_mem_offset);
					mem_offset += bytes_to_copy;
				}
			}

			if (run_aborted(run_epoch)) {
				aborted = true;
				break;
			}

			// Read out RX.  The normal path stores samples in fixed blocks; the
			// chunk path keeps only its current pending chunk in vectors.
			unsigned rx_locs = chunk_points > 0
				? read_rx(rx0_i, rx0_q, rx1_i, rx1_q, rx_reads_per_loop)
				: read_rx_to_blocks(rx_reads_per_loop);

			// emit any complete RX chunks accumulated so far
			emit_chunks();

			// Simple dynamic FIFO read-out speed governor
			if (rx_locs > MARGA_RX_FIFO_SPACE - 2*_max_rx_reads_per_loop) {
				rx_full = true;
				rx_full_mem_loc = old_pc;
				// desperately try to clear the FIFOs
				rx_reads_per_loop = _max_rx_reads_per_loop;
			} else if (rx_locs > 2 * MARGA_RX_FIFO_SPACE / 3) {
				++rx_nearly_full;
				// try hard to clear the FIFOs; scale up quickly
				if (rx_reads_per_loop < _max_rx_reads_per_loop) rx_reads_per_loop = rx_reads_per_loop * 4;
			} else if (rx_locs > MARGA_RX_FIFO_SPACE / 3) {
				if (rx_reads_per_loop < _max_rx_reads_per_loop) rx_reads_per_loop = rx_reads_per_loop * 2; // scale up
			} else {
				if (rx_reads_per_loop > _min_rx_reads_per_loop) rx_reads_per_loop = rx_reads_per_loop / 2;
			}

			// periodic buffer status checking
			if (execution_loops % execution_check_interval == 0) {
				buf_full = buf_full | rd32(_buf_full);
				buf_err = buf_err | rd32(_buf_err); // don't really need or here due to break; just for consistency
				if (buf_err) break;

				uint32_t status_latch = rd32(_status_latch);
				ocra1_data_lost = ocra1_data_lost | (status_latch & 0x1);
				ocra1_err = ocra1_err | (status_latch & 0x2);
				fhdo_err = fhdo_err | (status_latch & 0x4);
			}

			if (state == MAR_STATE_HALT) {
				finished = true;
			}
			old_pc = pc;
			++execution_loops;
		}

		// Do not let a stop that races normal completion get overwritten by
		// the usual ctrl=idle write. An aborted request waits until the safe
		// state has been fully established before its final reply is sent.
		if (aborted) {
			emergency_stop_result stop_result;
			std::lock_guard<std::mutex> io_guard(_io_mutex);
			force_safe_state(true, stop_result);
		} else if (finished) {
			std::lock_guard<std::mutex> io_guard(_io_mutex);
			if (run_aborted(run_epoch)) {
				aborted = true;
				emergency_stop_result stop_result;
				force_safe_state(true, stop_result);
			} else {
				wr32(_ctrl, 0x0);
			}
		// A non-stop execution failure still receives the established reset.
		} else {
			halt_and_reset();
		}

		// Stop may arrive just after the FPGA reaches HALT but before the final
		// reply is encoded. It must still turn this request into an aborted run.
		if (!aborted && run_aborted(run_epoch)) {
			aborted = true;
			emergency_stop_result stop_result;
			std::lock_guard<std::mutex> io_guard(_io_mutex);
			force_safe_state(true, stop_result);
		}

		// final buffer and gradient status checks
		buf_full = buf_full | rd32(_buf_full);
		buf_err = buf_err | rd32(_buf_err);

		uint32_t status_latch = rd32(_status_latch);
		ocra1_data_lost = ocra1_data_lost | (status_latch & 0x1);
		ocra1_err = ocra1_err | (status_latch & 0x2);
		fhdo_err = fhdo_err | (status_latch & 0x4);

		// post-mortem reporting
		if (aborted) {
			sa.add_info("run_seq aborted by emergency stop");
		} else if (mem_buffer_underrun) {
			sprintf(t, "memory buffer underrun around byte address 0x%0zx", old_pc);
			sa.add_error(t);
		} else if (mem_buffer_low) {
			sprintf(t, "memory buffer low for %u inner loop cycles", mem_buffer_low);
			sa.add_warning(t);
		} else if (mem_offset != total_bytes_to_copy) {
			sprintf(t, "didn't copy %zu bytes before FSM halted", total_bytes_to_copy - mem_offset);
			sa.add_error(t);
		}

		if (rx_full) {
			sprintf(t, "RX FIFO/s full during sequence around byte address 0x%0zx", rx_full_mem_loc);
			sa.add_error(t);
		} else if (rx_nearly_full) {
			sprintf(t, "RX FIFO/s almost filled during sequence %u times", rx_nearly_full);
			sa.add_warning(t);
		}

		if (buf_err) {
			sprintf(t, "output buffers overflowed during sequence: 0x%08x", buf_err);
			sa.add_error(t);
		} else if (buf_full) {
			sprintf(t, "output buffers were full during sequence: 0x%08x", buf_full);
			sa.add_warning(t);
		}

		if (ocra1_data_lost) sa.add_warning("ocra1 data was lost (overwritten before being sent)");
		if (ocra1_err) sa.add_error("ocra1 gradient error; possibly missing samples");
		if (fhdo_err) sa.add_error("gpa-fhdo gradient error; possibly missing samples");

		// readout of any final data remaining at the end
		unsigned read_tries = 0;
		unsigned final_rx_read = 0;
		// while (read_tries < _halt_tries_limit) {
		// 	final_rx_read += read_rx(rx0_i, rx0_q, rx1_i, rx1_q);
		// }
		while (!aborted && read_tries < 100) {
			final_rx_read += chunk_points > 0
				? read_rx(rx0_i, rx0_q, rx1_i, rx1_q, 100)
				: read_rx_to_blocks(100);
			emit_chunks();
			++read_tries;
			// TODO why does the RX
		}
		// delete after debugging is over
		debug_printf("Final RX read: %d\n", final_rx_read);

		// TODO make sure all the buffers and RX FIFOs are empty
		unsigned buf_empty = rd32(_buf_empty);
		if ( buf_empty != MAR_BUF_ALL_EMPTY ) {
			sprintf(t, "output buffers were not empty at the end of sequence: 0x%08x", buf_empty);
			sa.add_warning(t);
		}

		// wait a little longer for gradient interfaces to become idle
		unsigned gpa_idle_tries = 0;
		bool gpas_idle = false;
		while (gpa_idle_tries < _gpa_idle_tries_limit) {
			gpas_idle = (rd32(_status) & MAR_STATUS_GPA_MASK) == 0;
			if (gpas_idle) break;
			++gpa_idle_tries;
		}
		if (not gpas_idle) {
			sprintf(t, "GPAs not idle at the end of sequence");
			sa.add_warning(t);
		}

		// halt();

		if (!aborted && run_aborted(run_epoch)) {
			aborted = true;
			emergency_stop_result stop_result;
			std::lock_guard<std::mutex> io_guard(_io_mutex);
			force_safe_state(true, stop_result);
		}

		// encode the RX replies
		size_t rx0_elem = chunk_points > 0 ? rx0_i.size() : rx0_blocks.size();
		size_t rx1_elem = chunk_points > 0 ? rx1_i.size() : rx1_blocks.size();
		if (aborted) {
			mpack_start_map(wr, 2);
			mpack_write_cstr(wr, "final");
			mpack_write(wr, true);
			mpack_write_cstr(wr, "aborted");
			mpack_write(wr, true);
			mpack_finish_map(wr);
			finish_run(run_epoch);
		} else if (chunk_points > 0) {
			// Chunked mode: intermediate chunks were flushed during the loop; emit the
			// remaining (partial) chunk as the final chunk inside the normal reply so the
			// client knows when the stream ends.
			// Completed chunks were removed from the pending vectors as soon as
			// they were sent, so every remaining point belongs in the final reply.
			size_t rx0_rem = rx0_elem;
			size_t rx1_rem = rx1_elem;

			if (!rx0_elem and !rx1_elem) {
				sprintf(t, "no RX data received");
				sa.add_warning(t);
			}

			mpack_start_map(wr, (rx0_rem ? 2 : 0) + (rx1_rem ? 2 : 0) + 3);
			if (rx0_rem) {
				mpack_write_cstr(wr, "rx0_i");
				mpack_start_array(wr, rx0_rem);
				for (size_t k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx0_q");
				mpack_start_array(wr, rx0_rem);
				for (size_t k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_q[k]);
				mpack_finish_array(wr);
			}

			if (rx1_rem) {
				mpack_write_cstr(wr, "rx1_i");
				mpack_start_array(wr, rx1_rem);
				for (size_t k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx1_q");
				mpack_start_array(wr, rx1_rem);
				for (size_t k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_q[k]);
				mpack_finish_array(wr);
			}

			mpack_write_cstr(wr, "chunk_index");
			mpack_write(wr, (uint32_t)chunk_index);
			mpack_write_cstr(wr, "final");
			mpack_write(wr, true);
			mpack_write_cstr(wr, "aborted");
			// RX encoding can be long. Commit the abort flag only after all
			// arrays have been encoded so a Stop during encoding is still seen.
			mpack_write(wr, finish_run(run_epoch));
			mpack_finish_map(wr);
		} else if (structured_run) {
			mpack_start_map(wr, (rx0_elem ? 2 : 0) + (rx1_elem ? 2 : 0) + 2);
			if (rx0_elem) {
				mpack_write_cstr(wr, "rx0_i");
				mpack_start_array(wr, rx0_elem);
				for (const auto &block_ptr : rx0_blocks.blocks()) {
					const rx_block &block = *block_ptr;
					for (size_t k = 0; k < block.used; ++k) mpack_write_int(wr, block.i[k]);
				}
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx0_q");
				mpack_start_array(wr, rx0_elem);
				for (const auto &block_ptr : rx0_blocks.blocks()) {
					const rx_block &block = *block_ptr;
					for (size_t k = 0; k < block.used; ++k) mpack_write_int(wr, block.q[k]);
				}
				mpack_finish_array(wr);
			}
			if (rx1_elem) {
				mpack_write_cstr(wr, "rx1_i");
				mpack_start_array(wr, rx1_elem);
				for (const auto &block_ptr : rx1_blocks.blocks()) {
					const rx_block &block = *block_ptr;
					for (size_t k = 0; k < block.used; ++k) mpack_write_int(wr, block.i[k]);
				}
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx1_q");
				mpack_start_array(wr, rx1_elem);
				for (const auto &block_ptr : rx1_blocks.blocks()) {
					const rx_block &block = *block_ptr;
					for (size_t k = 0; k < block.used; ++k) mpack_write_int(wr, block.q[k]);
				}
				mpack_finish_array(wr);
			}
			mpack_write_cstr(wr, "final");
			mpack_write(wr, true);
			mpack_write_cstr(wr, "aborted");
			mpack_write(wr, finish_run(run_epoch));
			mpack_finish_map(wr);
			if (!rx0_elem && !rx1_elem) sa.add_warning("no RX data received");
		} else if (!rx0_elem and !rx1_elem) {
			if (finish_run(run_epoch)) {
				mpack_start_map(wr, 2);
				mpack_write_cstr(wr, "final");
				mpack_write(wr, true);
				mpack_write_cstr(wr, "aborted");
				mpack_write(wr, true);
				mpack_finish_map(wr);
			} else {
				mpack_write(wr, c_ok);
				sprintf(t, "no RX data received");
				sa.add_warning(t);
			}
		} else {
			mpack_start_map(wr, (rx0_elem ? 2:0) + (rx1_elem ? 2:0) + 2);
			if (rx0_elem) {
				mpack_write_cstr(wr, "rx0_i");
				mpack_start_array(wr, rx0_elem);
				for (const auto &block_ptr : rx0_blocks.blocks()) {
					const rx_block &block = *block_ptr;
					for (size_t k = 0; k < block.used; ++k) mpack_write_int(wr, block.i[k]);
				}
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx0_q");
				mpack_start_array(wr, rx0_elem);
				for (const auto &block_ptr : rx0_blocks.blocks()) {
					const rx_block &block = *block_ptr;
					for (size_t k = 0; k < block.used; ++k) mpack_write_int(wr, block.q[k]);
				}
				mpack_finish_array(wr);
			}

			if (rx1_elem) {
				mpack_write_cstr(wr, "rx1_i");
				mpack_start_array(wr, rx1_elem);
				for (const auto &block_ptr : rx1_blocks.blocks()) {
					const rx_block &block = *block_ptr;
					for (size_t k = 0; k < block.used; ++k) mpack_write_int(wr, block.i[k]);
				}
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx1_q");
				mpack_start_array(wr, rx1_elem);
				for (const auto &block_ptr : rx1_blocks.blocks()) {
					const rx_block &block = *block_ptr;
					for (size_t k = 0; k < block.used; ++k) mpack_write_int(wr, block.q[k]);
				}
				mpack_finish_array(wr);
			}
			mpack_write_cstr(wr, "final");
			mpack_write(wr, true);
			mpack_write_cstr(wr, "aborted");
			mpack_write(wr, finish_run(run_epoch));

			mpack_finish_map(wr);
		}
		}
	}

	// Test client-server network throughput
	auto tln = sa.get_command_and_start_reply("test_net", status);
	if (status == 1) {
		++commands_understood;
		unsigned data_size = mpack_node_uint(tln);

		mpack_start_map(wr, 2); // Two elements in map
		mpack_write_cstr(wr, "array1");
		mpack_start_array(wr, data_size);
		for (unsigned k{0}; k < data_size; ++k) mpack_write(wr, 1.01*k); // generic, needs C11
		mpack_finish_array(wr);

		mpack_write_cstr(wr, "array2");
		mpack_start_array(wr, data_size);
		for (unsigned k{0}; k < data_size; ++k) mpack_write(wr, 1.01*(k+10)); // generic, needs C11
		mpack_finish_array(wr);

		mpack_finish_map(wr);
	} else if (status == -1) {
		// TODO: callback or similar
	}

	// Test bus throughput
	auto tbt = sa.get_command_and_start_reply("test_bus", status);
	if (status == 1) {
		++commands_understood;
		auto n_tests = mpack_node_u32(tbt);
		if (_emergency_latched.load(std::memory_order_acquire)) {
			sa.add_error("test_bus rejected while emergency stop is latched");
			mpack_write(wr, c_err);
		} else {
		std::lock_guard<std::mutex> io_guard(_io_mutex);
		if (_emergency_latched.load(std::memory_order_acquire)) {
			sa.add_error("test_bus rejected while emergency stop is latched");
			mpack_write(wr, c_err);
		} else {

		auto start_t = std::chrono::system_clock::now();
		unsigned m = 0;
		for (unsigned k = 0; k < n_tests; ++k) {
			m += k;
		}
		auto null_t = std::chrono::system_clock::now();

		for (unsigned k = 0; k < n_tests; ++k) {
			m += rd32(_status); // repeatedly read the register
		}
		auto read_t = std::chrono::system_clock::now();

		for (unsigned k = 0; k < n_tests; ++k) {
			wr32(_ctrl, k & 0xfffffffc); // repeatedly write the register, but avoid setting the lower 2 bits
		}

		wr32(_ctrl, m & 0xfffffffc); // avoid m getting optimised out of the first loop

		auto write_t = std::chrono::system_clock::now();

		// reply will contain the three differences
		int64_t null_ti = std::chrono::duration_cast<std::chrono::microseconds>(null_t - start_t).count(),
			read_ti = std::chrono::duration_cast<std::chrono::microseconds>(read_t - null_t).count(),
			write_ti = std::chrono::duration_cast<std::chrono::microseconds>(write_t - read_t).count();

		mpack_start_array(wr, 3);
		mpack_write(wr, null_ti);
		mpack_write(wr, read_ti);
		mpack_write(wr, write_ti);
		mpack_finish_array(wr);
		}
		}
	}

	// Determine if the server is running on hardware or just an emulation
	sa.get_command_and_start_reply("are_you_real", status);
	if (status == 1) {
		++commands_understood;
#ifdef __arm__
		mpack_write(wr, "hardware");
#else

#ifdef VERILATOR_BUILD
		mpack_write(wr, "simulation");
#else
		mpack_write(wr, "software");
#endif

#endif
	}

	// Final housekeeping
	mpack_finish_map(wr);

	if (commands_understood != commands_present) {
		assert(commands_understood <= commands_present && "Serious bug in logic");
		sa.add_error("not all client commands were understood");

		// Fill in remaining elements of the response map (TODO: maybe make this more sophisticated?)
		while (commands_present != 0) {
			char t[100];
			sprintf(t, "UNKNOWN%zu", commands_present);
			mpack_write_kv(wr, t, -1);
			commands_present--;
		}
	}

	return problems;
}

void hardware::init_mem() {
#ifdef __arm__
	int fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		throw hw_error("failed to access memory device - check sudo permissions and/or platform");
	}
#else
	char tempfile[100] = "/tmp/marcos_server_mem";

	int fd = open(tempfile, O_RDWR);
	if (fd < 0) {
		char errstr[1024];
		size_t filesize_KiB =  4 * END_OFFSET / EMU_PAGESIZE; // 4 because 4 KiB / page
		sprintf(errstr, "Failed to open simulated memory device.\n"\
		        "Check whether %s exists, and if not create it using:\n"\
		        "fallocate -l %ldKiB %s", tempfile, filesize_KiB, tempfile);
		throw hw_error(errstr);
	}
#endif

	// set up shared memory (please refer to the memory offset table)

	// VN: I'm not sure why in the original server, different data
	// types were used for some of these. Perhaps to allow
	// different access widths?
	_slcr = (uint32_t *) mmap(NULL, SLCR_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, SLCR_OFFSET);

	_mar_base = (uint32_t *) mmap(NULL, MARGA_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MARGA_OFFSET);

	// Map the control and status registers
	_ctrl = _mar_base + 0;
	// slv_reg1 for extension in the future
	_direct = _mar_base + 2;
	// slv_reg3 for extension in the future
	_exec = _mar_base + 4; // execution information
	_status = _mar_base + 5; // external status, ADC etc
	_status_latch = _mar_base + 6; // latched external status
	_buf_err = _mar_base + 7; // latched buffer errors
	_buf_full = _mar_base + 8; // latched full buffers
	_buf_empty = _mar_base + 9; // empty buffers
	_rx_locs = _mar_base + 10; // RX data available
	_rx0_i_data = _mar_base + 11; // RX0 i data
	_rx1_i_data = _mar_base + 12; // RX1 i data
	_rx0_q_data = _mar_base + 13; // RX0 q data
	_rx1_q_data = _mar_base + 14; // RX1 q data

	// /2 since mem is halfway in address space, /4 to convert to 32-bit instead of byte addressing
	_mar_mem = reinterpret_cast<volatile char *>(_mar_base + MARGA_SIZE/2/4);
}

void hardware::halt() {
	emergency_stop_result result;
	std::lock_guard<std::mutex> io_guard(_io_mutex);
	force_safe_state(_emergency_latched.load(std::memory_order_acquire), result);
}

void hardware::halt_and_reset() {
	// Old OCRA server comment: set FPGA clock to 143 MHz (VN: not
	// sure how this works - probably not needed any more?)
	_slcr[2] = 0xDF0D;
	_slcr[92] = (_slcr[92] & ~0x03F03F30) | 0x00100700;

	emergency_stop_result result;
	std::lock_guard<std::mutex> io_guard(_io_mutex);
	force_safe_state(_emergency_latched.load(std::memory_order_acquire), result);
}

emergency_stop_result hardware::emergency_stop(const std::string &gradient_board_name,
                                               const std::string &acquisition_id) {
	emergency_stop_result result;
	std::lock_guard<std::mutex> io_guard(_io_mutex);
	// Publishing cancellation and forcing safe MMIO are serialized with both
	// start_run() and the normal FINISHED transition. Thus a racing ctrl=1 or
	// ctrl=0 write can never occur after this stop has taken ownership.
	_emergency_latched.store(true, std::memory_order_release);
	_abort_epoch.fetch_add(1, std::memory_order_acq_rel);
	remember_aborted_acquisition(acquisition_id);
	if (!gradient_board_name.empty()) set_gradient_board(gradient_board_name);
	force_safe_state(true, result);
	return result;
}

bool hardware::prepare_run(bool rearm, const std::string &acquisition_id,
                           uint64_t &run_epoch) {
	std::lock_guard<std::mutex> io_guard(_io_mutex);
	if (acquisition_was_aborted(acquisition_id)) return false;
	run_epoch = _abort_epoch.load(std::memory_order_acquire);
	if (_emergency_latched.load(std::memory_order_acquire) && !rearm) return false;
	if (rearm) _emergency_latched.store(false, std::memory_order_release);
	if (_abort_epoch.load(std::memory_order_acquire) != run_epoch ||
	    _emergency_latched.load(std::memory_order_acquire)) return false;
	return true;
}

bool hardware::acquisition_was_aborted(const std::string &acquisition_id) const {
	if (acquisition_id.empty()) return false;
	return _aborted_acquisition_ids.find(acquisition_id) !=
	       _aborted_acquisition_ids.end();
}

void hardware::remember_aborted_acquisition(const std::string &acquisition_id) {
	if (!acquisition_id.empty()) _aborted_acquisition_ids.insert(acquisition_id);
}

bool hardware::run_aborted(uint64_t run_epoch) const {
	return _emergency_latched.load(std::memory_order_acquire) ||
	       _abort_epoch.load(std::memory_order_acquire) != run_epoch;
}

bool hardware::start_run(uint64_t run_epoch) {
	std::lock_guard<std::mutex> io_guard(_io_mutex);
	if (run_aborted(run_epoch)) return false;
	wr32(_ctrl, 0x1);
	return true;
}

bool hardware::finish_run(uint64_t run_epoch) {
	std::lock_guard<std::mutex> io_guard(_io_mutex);
	return run_aborted(run_epoch);
}

void hardware::direct_write(unsigned buffer, uint16_t value) {
	wr32(_direct, (buffer << 24) | value);
	// Allow the direct-write pipeline to advance before a dependent write.
	(void)rd32(_exec);
	(void)rd32(_exec);
}

bool hardware::wait_for_fsm_idle() {
	for (unsigned k = 0; k < _halt_tries_limit; ++k) {
		if ((rd32(_exec) >> 24) == MAR_STATE_IDLE) return true;
	}
	return false;
}

bool hardware::wait_for_buffers_empty() {
	for (unsigned k = 0; k < _halt_tries_limit; ++k) {
		if (rd32(_buf_empty) == MAR_BUF_ALL_EMPTY) return true;
	}
	return false;
}

bool hardware::wait_for_gradient_idle(uint32_t status_mask) {
	// The serializers are pipelined; do not mistake the pre-transfer idle
	// value for completion of the direct command just issued.
	(void)rd32(_status);
	(void)rd32(_status);
	for (unsigned k = 0; k < _gpa_idle_tries_limit; ++k) {
		if ((rd32(_status) & status_mask) == 0) return true;
	}
	return false;
}

void hardware::zero_digital_outputs() {
	// TX I/Q, external TX/RX gates + trigger, and both RX enables/resets.
	for (unsigned buffer = 5; buffer <= 8; ++buffer) direct_write(buffer, 0);
	direct_write(15, 0);
	direct_write(16, 0);
}

bool hardware::zero_gradients() {
	const gradient_board board = _gradient_board.load(std::memory_order_acquire);
	const uint16_t slow_spi_div = 63;

	if (board == gradient_board::ocra1 || board == gradient_board::ocra1_5761) {
		// Buffer-empty only says MaRGA has handed off the final word.  The
		// OCRA serializer may still be clocking that word, and it has no queue
		// for a broadcast received while busy.  Do not issue the zero frame
		// until the previous SPI transaction has completed.
		if (!wait_for_gradient_idle(0x00010000u)) return false;

		// Enable OCRA1, release its serializer reset, and trigger only from
		// LSB writes. The final channel word carries the broadcast bit.
		direct_write(0, (1u << 0) | (slow_spi_div << 2) | (1u << 8));
		const uint32_t zero_payload = board == gradient_board::ocra1
			? 0x00100000u : 0x00030000u;
		for (unsigned channel = 0; channel < 4; ++channel) {
			uint32_t word = zero_payload | (channel << 25);
			if (channel == 3) word |= (1u << 24);
			direct_write(2, (uint16_t)(word >> 16));
			direct_write(1, (uint16_t)word);
		}
		bool idle = wait_for_buffers_empty() && wait_for_gradient_idle(0x00010000u);
		direct_write(0, 0);
		return idle;
	}

	if (board == gradient_board::gpa_fhdo) {
		// Like OCRA, GPA-FHDO accepts a new word only while its serializer is
		// idle.  Otherwise the first channel's zero command could be dropped.
		if (!wait_for_gradient_idle(0x00020000u)) return false;

		// GPA-FHDO zero current is the midpoint DAC code (0x8000), written
		// explicitly to all four DAC input registers.
		direct_write(0, (2u << 0) | (slow_spi_div << 2));
		bool idle = true;
		for (unsigned channel = 0; channel < 4; ++channel) {
			uint32_t word = ((0x8u + channel) << 16) | 0x8000u;
			direct_write(2, (uint16_t)(word >> 16));
			direct_write(1, (uint16_t)word);
			idle = wait_for_buffers_empty() &&
			       wait_for_gradient_idle(0x00020000u) && idle;
		}
		direct_write(0, 0);
		return idle;
	}

	// Sending a command for the wrong DAC type can itself be unsafe. New
	// clients therefore identify the board in each structured run_seq.
	direct_write(0, 0);
	return false;
}

bool hardware::drain_rx_fifos() {
	for (unsigned pass = 0; pass < 8; ++pass) {
		uint32_t locs = rd32(_rx_locs);
		unsigned fifo0_locs = locs & 0xffffu;
		unsigned fifo1_locs = locs >> 16;
		if (fifo0_locs == 0 && fifo1_locs == 0) return true;
		while (fifo0_locs-- > 0) {
			(void)rd32(_rx0_q_data);
			(void)rd32(_rx0_i_data);
		}
		while (fifo1_locs-- > 0) {
			(void)rd32(_rx1_q_data);
			(void)rd32(_rx1_i_data);
		}
	}
	return rd32(_rx_locs) == 0;
}

void hardware::force_safe_state(bool keep_stop_latched, emergency_stop_result &result) {
	wr32(_ctrl, 0x2);
	result.fsm_idle = wait_for_fsm_idle();

	// Direct writes are accepted in IDLE. Write the digital zeros once as
	// early as possible, then again after every queued buffer value is gone.
	if (result.fsm_idle) zero_digital_outputs();
	bool old_buffers_empty = wait_for_buffers_empty();
	if (result.fsm_idle && old_buffers_empty) zero_digital_outputs();
	bool zero_buffers_empty = wait_for_buffers_empty();
	result.gradients_zeroed = result.fsm_idle && old_buffers_empty &&
	                          zero_buffers_empty && zero_gradients();
	if (result.fsm_idle && old_buffers_empty) zero_digital_outputs();
	bool final_buffers_empty = wait_for_buffers_empty();
	result.digital_outputs_zeroed = result.fsm_idle && old_buffers_empty &&
	                                zero_buffers_empty && final_buffers_empty;
	result.rx_fifo_empty = drain_rx_fifos();

	// ctrl=2 remains asserted for an emergency latch. The next run_seq must
	// explicitly rearm before start_run() can replace it with ctrl=1.
	wr32(_ctrl, keep_stop_latched ? 0x2 : 0x0);
}

void hardware::set_gradient_board(const std::string &name) {
	if (name == "ocra1") _gradient_board.store(gradient_board::ocra1, std::memory_order_release);
	else if (name == "ocra1_5761" || name == "ocra1-5761") _gradient_board.store(gradient_board::ocra1_5761, std::memory_order_release);
	else if (name == "gpa-fhdo" || name == "gpa_fhdo") _gradient_board.store(gradient_board::gpa_fhdo, std::memory_order_release);
	else _gradient_board.store(gradient_board::unknown, std::memory_order_release);
}

unsigned hardware::read_rx(std::vector<uint32_t> &rx0_i, std::vector<uint32_t> &rx0_q,
                           std::vector<uint32_t> &rx1_i, std::vector<uint32_t> &rx1_q,
                           const unsigned max_reads) {
	uint32_t rxlocs = rd32(_rx_locs);
	int fifo0_locs = rxlocs & 0xffff, fifo1_locs = rxlocs >> 16;

	unsigned reads = 0;

	debug_printf("initial fifo locs: %d, %d\n", fifo0_locs, fifo1_locs);

	while (fifo0_locs > 0 or fifo1_locs > 0) {
		if (reads >= max_reads) break;

		bool read_fifo0 = false, read_fifo1 = false;
		if (fifo1_locs > 2*fifo0_locs) {
			// too much data in fifo1, read it exclusively
			read_fifo1 = true;
		} else if (fifo0_locs > 2*fifo1_locs) {
			// too much data in fifo0, read it exclusively
			read_fifo0 = true;
		} else { // read both
			read_fifo0 = true;
			read_fifo1 = true;
		}

		if (read_fifo0) { // read pair of samples
			rx0_q.push_back(rd32(_rx0_q_data)); // read q first
			rx0_i.push_back(rd32(_rx0_i_data)); // pop fifo
			++reads;
			--fifo0_locs;
		}
		if (read_fifo1) { // read pair of samples
			rx1_q.push_back(rd32(_rx1_q_data)); // read q first
			rx1_i.push_back(rd32(_rx1_i_data)); // pop fifo
			++reads;
			--fifo1_locs;
		}
	}

	// Let some time pass so that the FIFO-fullness register can update
	// (this is purely to make the simulation behaviour more realistic)
#ifdef VERILATOR_BUILD
	rd32(_rx1_q_data);
#endif

	// return the FIFO closest to filling
	if (fifo0_locs > fifo1_locs) return fifo0_locs;
	else return fifo1_locs;
}

void hardware::wr32(volatile uint32_t *addr, uint32_t data) {
#ifdef VERILATOR_BUILD
	if (addr >= _mar_base && addr < _mar_base + MARGA_SIZE) {
		// do byte-address arithmetic
		auto offs_addr = reinterpret_cast<volatile char *>(addr)
			- reinterpret_cast<volatile char *>(_mar_base);
		// printf("addresses 0x%0lx, 0x%0lx\n", addr, _mar_base);
		// printf("write mar 0x%0lx, 0x%08x\n", offs_addr, data);
		mm->wr32(offs_addr, data); // convert to byte addressing
	} else {
		printf("write addr 0x%0lx, 0x%08x NOT SIMULATED\n", (size_t) addr, data);
	}
#else
	*addr = data;
#endif
}

uint32_t hardware::rd32(volatile uint32_t *addr) {
#ifdef VERILATOR_BUILD
	if (addr >= _mar_base && addr < _mar_base + MARGA_SIZE) {
		// do byte-address arithmetic
		auto offs_addr = reinterpret_cast<volatile char *>(addr)
			- reinterpret_cast<volatile char *>(_mar_base);
		// printf("read mar 0x%0lx\n", offs_addr);
		return mm->rd32(offs_addr); // convert to byte addressing
	} else {
		printf("read addr 0x%0lx NOT SIMULATED\n", (size_t) addr);
		return 0;
	}
#else
	return *addr;
#endif
}

void* hardware::hw_memcpy(volatile void *s1, const void *s2, size_t n) {
#ifdef VERILATOR_BUILD
	// copy the data via individual 32b bus writes
	auto *s1u = reinterpret_cast<volatile uint32_t *>(s1);
	auto *s2u = reinterpret_cast<const volatile uint32_t *>(s2);
	size_t nu = n / 4;

	// algorithm copied verbatim from mpack, just acting on uint32
	while (nu-- != 0) wr32(s1u++, *s2u++);
	return (void *)s1u;
#else
	return memcpy((void *)s1, s2, n);
#endif
}

size_t hardware::hw_mpack_node_copy_data(mpack_node_t node, volatile char *buffer, size_t bufsize) {
#ifdef VERILATOR_BUILD
	// Copy the data via individual 32b bus writes
	char *tmp = reinterpret_cast<char *>(malloc(bufsize));
	size_t bytes_copied = mpack_node_copy_data(node, tmp, bufsize);

	// Inefficient, but won't be a major delay in the simulation anyway
	// TODO: check pointer arithmetic!
	auto tmp_u32 = reinterpret_cast<uint32_t *>(tmp);
	size_t offset = 0;
	for (size_t k = 0; k < bytes_copied/4; ++k) {
		wr32(reinterpret_cast<volatile uint32_t *>(buffer + offset), tmp_u32[k]);
		offset += 4;
	}

	free(tmp);
	return bytes_copied;
#else
	return mpack_node_copy_data(node, (char *)buffer, bufsize); // discard volatile qualifier
#endif
}
