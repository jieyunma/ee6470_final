#ifndef _DWT_2_H_
#define _DWT_2_H_
#include <systemc>
#include <sys/time.h>
#include <cmath>
#include <iomanip>
#include "filter_def.h"
#include "tlm"
#include "tlm_utils/simple_target_socket.h"

using namespace std;
using namespace sc_core;
using namespace sc_dt;

const int HiD2[8] = {-4, -2, 13, 38, -103, 64, 4, -10};
const int LoD2[8] = {-10, -4, 64, 103, 38, -13, -2, 4};

struct DWT_2 : public sc_module {
    tlm_utils::simple_target_socket<DWT_2> tsock;

	sc_fifo< sc_dt::sc_int<32> > i_spike;
	sc_fifo< sc_dt::sc_int<32> > o_D;
	sc_fifo< sc_dt::sc_int<32> > o_A;

    SC_HAS_PROCESS(DWT_2);

    DWT_2(sc_module_name n):
        sc_module(n), 
        tsock("t_skt") {
        SC_THREAD( conv );

        tsock.register_b_transport(this, &DWT_2::blocking_transport);
    }

    ~DWT_2() {

    }

    void conv(){
        sc_int<32> spike1, spike2;
        sc_int<32> approx, detail;
        int spike_buffer[8] = {0};
        int m = 0;

        while(true) { // m loop

            for (; m < 8; m = m + 2) { // buffer initial
                spike1 = i_spike.read();
                spike2 = i_spike.read();
                spike_buffer[7] = spike_buffer[5];
                spike_buffer[6] = spike_buffer[4];
                spike_buffer[5] = spike_buffer[3];
                spike_buffer[4] = spike_buffer[2];
                spike_buffer[3] = spike_buffer[1];
                spike_buffer[2] = spike_buffer[0];
                spike_buffer[1] = spike2;
                spike_buffer[0] = spike1;
            }
// cout << "Get: " << spike1 << "," << spike2 << endl;
            if (m >= 8) {
                int val_D = 0;
                int val_A = 0;

                for (int n = 0; n < 8; n++) {
                    val_D += spike_buffer[n] * HiD2[n];
                    val_A += spike_buffer[n] * LoD2[n];
                }
                o_D.write(val_D);
                o_A.write(val_A);
//cout << "Send data: " << val_D << ", " << val_A << endl;
                spike1 = i_spike.read();
                spike2 = i_spike.read();
                spike_buffer[7] = spike_buffer[5];
                spike_buffer[6] = spike_buffer[4];
                spike_buffer[5] = spike_buffer[3];
                spike_buffer[4] = spike_buffer[2];
                spike_buffer[3] = spike_buffer[1];
                spike_buffer[2] = spike_buffer[0];
                spike_buffer[1] = spike2;
                spike_buffer[0] = spike1;                
            }
        }
        
    }
    void blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay) {
        wait(delay);
        sc_dt::uint64 addr = payload.get_address();
        //addr = addr - base_offset;
        unsigned char *mask_ptr = payload.get_byte_enable_ptr();
        unsigned char *data_ptr = payload.get_data_ptr();
        sc_int<32> approx;
        sc_int<32> detail;
        sc_int<32> spike;

        switch (payload.get_command()) {
            case tlm::TLM_READ_COMMAND:
                //cout << "TLM READ: " << addr << endl;
                switch (addr) {
                    case DETAIL_ADDR:
                        //cout << "conv1 socket try read result: " << endl;
                        detail = o_D.read();
                        // cout << "conv1 socket read result: " << result << endl;
                        break;
                    case APPROX_ADDR:
                        //cout << "conv1 socket try read result: " << endl;
                        approx = o_A.read();
                        // cout << "conv1 socket read result: " << result << endl;
                        break;                        
                    default:
                        std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                                << std::setfill('0') << std::setw(8) << std::hex << addr
                                << std::dec << " is not valid" << std::endl;
                        break;
                }
                word buffer;
                buffer.uint = (addr == DETAIL_ADDR) ? detail : approx;
                data_ptr[0] = buffer.uc[0];
                data_ptr[1] = buffer.uc[1];
                data_ptr[2] = buffer.uc[2];
                data_ptr[3] = buffer.uc[3];
                
                //cout << "conv socket set result: " << result << endl;
                break;
            case tlm::TLM_WRITE_COMMAND:
                //cout << "TLM WRITE: " << addr << endl;
                switch (addr) {
                    case SPIKE_ADDR:
                        spike.range(7,0) = data_ptr[0];
                        spike.range(15,8) = data_ptr[1];
                        spike.range(23,16) = data_ptr[2];
                        spike.range(31,24) = data_ptr[3];
                        // cout << "conv1 socket get source: " << source << endl;
                        i_spike.write(spike);
                        break;
                    default:
                        std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                                << std::setfill('0') << std::setw(8) << std::hex << addr
                                << std::dec << " is not valid" << std::endl;
                        break;
                }
                break;
            case tlm::TLM_IGNORE_COMMAND:
                payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
                return;
            default:
                payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
                return;
        }
        payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
    }

};
#endif
