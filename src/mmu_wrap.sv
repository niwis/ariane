// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 19/04/2017
// Description: Memory Management Unit for Ariane, contains TLB and
//              address translation unit. SV39 as defined in RISC-V
//              privilege specification 1.11-WIP

import ariane_pkg::*;

module mmu_wrap #(
      parameter int unsigned INSTR_TLB_ENTRIES     = 4,
      parameter int unsigned DATA_TLB_ENTRIES      = 4,
      parameter int unsigned ASID_WIDTH            = 1,
      parameter ariane_pkg::ariane_cfg_t ArianeCfg = ariane_pkg::ArianeDefaultConfig
) (

        input  logic                            clk_i,
        input  logic                            rst_ni,
        input  logic                            flush_i,
        input  logic                            enable_translation_i,
        input  logic                            en_ld_st_translation_i,   // enable virtual memory translation for load/stores
        
        // General control signals
        input riscv::priv_lvl_t                 priv_lvl_i,
        input riscv::priv_lvl_t                 ld_st_priv_lvl_i,
        input logic                             sum_i,
        input logic                             mxr_i,
        input logic [43:0]                      satp_ppn_i,
        input logic [ASID_WIDTH-1:0]            asid_i,
        input logic                             flush_tlb_i,
        input logic                             flush_tlb_plru_tree_i,

        ////////////////////////////1////////////////////////////////
        input  icache_areq_o_t                  icache_areq_i,
        output icache_areq_i_t                  icache_areq_o,
        // PTW memory interface
        input  dcache_req_o_t                   req_port_i,
        output dcache_req_i_t                   req_port_o,

        input  exception_t                      misaligned_ex_i,
        input  logic                            lsu_req_i,        // request address translation
        input  logic [riscv::VLEN-1:0]          lsu_vaddr_i,      // virtual address in
        input  logic                            lsu_is_store_i,   // the translation is requested by a store

        output logic                            lsu_dtlb_hit_o,   // sent in the same cycle as the request if translation hits in the DTLB
        output logic                            lsu_valid_o,      // translation is valid
        output logic [riscv::PLEN-1:0]          lsu_paddr_o,      // translated address
        output exception_t                      lsu_exception_o,  // address translation threw an exception
        // Performance counters
        output logic                            itlb_miss_o,
        output logic                            dtlb_miss_o,

        ////////////////////////////2////////////////////////////////
        input  icache_areq_o_t                  icache_areq_i2,
        output icache_areq_i_t                  icache_areq_o2,
        // PTW memory interface
        input  dcache_req_o_t                   req_port_i2,
        output dcache_req_i_t                   req_port_o2,

        input  exception_t                      misaligned_ex_i2,
        input  logic                            lsu_req_i2,        // request address translation
        input  logic [riscv::VLEN-1:0]          lsu_vaddr_i2,      // virtual address in
        input  logic                            lsu_is_store_i2,   // the translation is requested by a store

        output logic                            lsu_dtlb_hit_o2,   // sent in the same cycle as the request if translation hits in the DTLB
        output logic                            lsu_valid_o2,      // translation is valid
        output logic [riscv::PLEN-1:0]          lsu_paddr_o2,      // translated address
        output exception_t                      lsu_exception_o2,  // address translation threw an exception
        // Performance counters
        output logic                            itlb_miss_o2,
        output logic                            dtlb_miss_o2
);

    mmu #(
        .INSTR_TLB_ENTRIES      ( INSTR_TLB_ENTRIES                     ),
        .DATA_TLB_ENTRIES       ( DATA_TLB_ENTRIES                     ),
        .ASID_WIDTH             ( ASID_WIDTH             ),
        .ArianeCfg              ( ArianeCfg              )
    ) i_mmu (.*);

    mmu #(
        .INSTR_TLB_ENTRIES      ( INSTR_TLB_ENTRIES                     ),
        .DATA_TLB_ENTRIES       ( DATA_TLB_ENTRIES                     ),
        .ASID_WIDTH             ( ASID_WIDTH             ),
        .ArianeCfg              ( ArianeCfg              )
    ) i_mmu2 (
        .misaligned_ex_i        ( misaligned_ex_i2   ),
        .lsu_is_store_i         ( lsu_is_store_i2     ),
        .lsu_req_i              ( lsu_req_i2        ),
        .lsu_vaddr_i            ( lsu_vaddr_i2              ),
        .lsu_valid_o            ( lsu_valid_o2      ),
        .lsu_paddr_o            ( lsu_paddr_o2              ),
        .lsu_exception_o        ( lsu_exception_o2          ),
        .lsu_dtlb_hit_o         ( lsu_dtlb_hit_o2           ),
        // connecting PTW to D$ IF
        .req_port_i             ( req_port_i2 ),
        .req_port_o             ( req_port_o2 ),
        // icache address translation requests
        .icache_areq_i          ( icache_areq_i2          ),
        .icache_areq_o          ( icache_areq_o2          ),
        //perf count
        .itlb_miss_o            (itlb_miss_o2),
        .dtlb_miss_o            (dtlb_miss_o2),
        .*
    );

endmodule
