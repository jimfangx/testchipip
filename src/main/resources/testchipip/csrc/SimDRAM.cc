#include <vpi_user.h>
#include <svdpi.h>
#include <stdint.h>
#include <cassert>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <sys/mman.h>
#include <fesvr/memif.h>
#include <fesvr/elfloader.h>

#include "mm_dramsim2.h"
#include "mm_dramsim3.h"

// Set this to true for non-sim-flow consumers that need to force DRAMSim2.
bool force_use_dramsim2 = false;
std::string dramsim2_ini_dir = "dramsim2_ini";
std::string dramsim2_memory_ini = "DDR3_micron_64M_8B_x4_sg15.ini";
std::string dramsim2_system_ini = "system.ini";
std::string dramsim3_ini = "DRAMSim3.ini";
std::string dramsim3_out = "dramsim-results";
std::string loadmem_file = "";
std::vector<std::map<long long int, backing_data_t>> backing_mem_data = {};
std::map<int, int> chip_id_to_channel_count = {};

// TODO FIX: This doesn't properly handle striped memory across multiple channels
// The full memory range is duplicated across each channel
extern "C" void *memory_init(
                             int chip_id,
                             long long int mem_size,
                             long long int word_size,
                             long long int line_size,
                             long long int id_bits,
                             long long int clock_hz,
                             long long int mem_base,
                             int addr_bits
			     )
{
    // in xcelium, while mem_base is a 40b unsigned param, when passed into a long long int DPI variable it becomes signed.
    // mask off the upper address bits (since we enforce that only address bits worth of address space is available).
    uint64_t mem_base_mask = (1ULL << addr_bits) - 1;
    mem_base &= mem_base_mask;

    mm_t *mm;
    s_vpi_vlog_info info;
    bool use_dramsim2 = force_use_dramsim2;

    if (!vpi_get_vlog_info(&info))
      abort();

    for (int i = 1; i < info.argc; i++) {
      std::string arg(info.argv[i]);

      if (arg == "+dramsim2" || arg == "+use_dramsim2")
        use_dramsim2 = true;
      if (arg == "+dramsim3" || arg == "+dramsim")
        use_dramsim2 = false;
      if (arg.find("+dramsim_ini_dir=") == 0)
        dramsim2_ini_dir = arg.substr(strlen("+dramsim_ini_dir="));
      if (arg.find("+dramsim_ini=") == 0)
        dramsim3_ini = arg.substr(strlen("+dramsim_ini="));
      if (arg.find("+dramsim_out=") == 0)
        dramsim3_out = arg.substr(strlen("+dramsim_out="));
      if (arg.find("+loadmem=") == 0)
        loadmem_file = arg.substr(strlen("+loadmem="));
    }

    while (chip_id >= backing_mem_data.size()) {
      backing_mem_data.push_back(std::map<long long int, backing_data_t>());
    }

    if (backing_mem_data[chip_id].find(mem_base) != backing_mem_data[chip_id].end()) {
      assert(backing_mem_data[chip_id][mem_base].size == mem_size);
    } else {
      uint8_t* data = (uint8_t*) mmap(NULL, mem_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

      class loadmem_memif_t : public memif_t {
      public:
        loadmem_memif_t(uint8_t* _data, size_t _start) : memif_t(nullptr), data(_data), start(_start) {}
        void write(addr_t taddr, size_t len, const void* src) override
        {
          addr_t addr = taddr - start;
          memcpy(data + addr, src, len);
        }
        void read(addr_t taddr, size_t len, void* bytes) override {
          assert(false);
        }
        endianness_t get_target_endianness() const override {
          return endianness_little;
        }
      private:
        uint8_t* data;
        size_t start;
      } loadmem_memif(data, mem_base);
      if (loadmem_file != "") {
        reg_t entry;
        load_elf(loadmem_file.c_str(), &loadmem_memif, &entry, 0);
      }

      backing_mem_data[chip_id][mem_base] = {data, (size_t)mem_size};
    }

    int channel_id = chip_id_to_channel_count[chip_id]++;

    if (use_dramsim2)
      mm = (mm_t *) (new mm_dramsim2_t(mem_base, mem_size, word_size, line_size,
                                       backing_mem_data[chip_id][mem_base],
                                       dramsim2_memory_ini, dramsim2_system_ini, dramsim2_ini_dir,
                                       1 << id_bits, clock_hz));
    else
      mm = (mm_t *) (new mm_dramsim3_t(mem_base, mem_size, word_size, line_size,
                                       backing_mem_data[chip_id][mem_base],
                                       dramsim3_ini, dramsim3_out,
                                       1 << id_bits, clock_hz, channel_id));


    return mm;
}

extern "C" void memory_tick(
        void *channel,

        unsigned char reset,

        unsigned char ar_valid,
        unsigned char *ar_ready,
        long long int ar_addr,
        int ar_id,
        int ar_size,
        int ar_len,

        unsigned char aw_valid,
        unsigned char *aw_ready,
        long long int aw_addr,
        int aw_id,
        int aw_size,
        int aw_len,

        unsigned char w_valid,
        unsigned char *w_ready,
        int w_strb,
        const svOpenArrayHandle w_data,
        unsigned char w_last,

        unsigned char *r_valid,
        unsigned char r_ready,
        int *r_id,
        int *r_resp,
        const svOpenArrayHandle r_data,
        unsigned char *r_last,

        unsigned char *b_valid,
        unsigned char b_ready,
        int *b_id,
        int *b_resp)
{
    mm_t *mm = (mm_t *) channel;

    int w_data_bytes = svSize(w_data, 1);
    unsigned char *w_data_arr = (unsigned char *) malloc(w_data_bytes * sizeof(char));
    for (int i = 0; i < w_data_bytes; i++) {
        const unsigned char* elem_ptr = static_cast<const unsigned char*>(svGetArrElemPtr1(w_data, i));
        assert(elem_ptr != nullptr);
        w_data_arr[i] = *elem_ptr;
    }

    mm->tick(
        reset,

        ar_valid,
        ar_addr,
        ar_id,
        ar_size,
        ar_len,

        aw_valid,
        aw_addr,
        aw_id,
        aw_size,
        aw_len,

        w_valid,
        w_strb,
        w_data_arr,
        w_last,

        r_ready,
        b_ready);

    free(w_data_arr);

    *ar_ready = mm->ar_ready();
    *aw_ready = mm->aw_ready();
    *w_ready = mm->w_ready();
    *r_valid = mm->r_valid();
    *r_id = mm->r_id();
    *r_resp = mm->r_resp();

    int r_data_bytes = svSize(r_data, 1);
    #ifdef VERILATOR
        for (int i = 0; i < r_data_bytes; i++) {
            svPutBitArrElemVecVal(r_data, (const uint32_t *)(((char *) mm->r_data()) + i), i);
        }
    #else
        for (int i = 0; i < r_data_bytes; i++) {
            svPutBitArrElem(r_data, ((unsigned char *) mm->r_data())[i], i);
        }
    #endif

    *r_last = mm->r_last();
    *b_valid = mm->b_valid();
    *b_id = mm->b_id();
    *b_resp = mm->b_resp();
}
