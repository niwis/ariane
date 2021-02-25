// Copyright 2017-2019 ETH Zurich and University of Bologna.
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
// Date: 19.03.2017
// Description: Ariane Top-level module

import ariane_pkg::*;

module ariane_wrap #(
  parameter ariane_pkg::ariane_cfg_t ArianeCfg     = ariane_pkg::ArianeDefaultConfig
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  // Core ID, Cluster ID and boot address are considered more or less static
  input  logic [63:0]                  boot_addr_i,  // reset boot address
  input  logic [63:0]                  hart_id_i,    // hart id in a multicore environment (reflected in a CSR)

  // Interrupt inputs
  input  logic [1:0]                   irq_i,        // level sensitive IR lines, mip & sip (async)
  input  logic                         ipi_i,        // inter-processor interrupts (async)
  // Timer facilities
  input  logic                         time_irq_i,   // timer interrupt in (async)
  input  logic                         debug_req_i,  // debug request (async)

  // L15 (memory side)
  // output wt_cache_pkg::l15_req_t       l15_req_o,
  // input  wt_cache_pkg::l15_rtrn_t      l15_rtrn_i,
  // output wt_cache_pkg::l15_req_t       l15_req_o2,
  // input  wt_cache_pkg::l15_rtrn_t      l15_rtrn_i2
  output ariane_axi::req_t             axi_req_o,
  input  ariane_axi::resp_t            axi_resp_i,
  output ariane_axi::req_t             axi_req_o2,
  input  ariane_axi::resp_t            axi_resp_i2
);
//genvar i;
//generate for (i = 0)
  ariane #(
    .ArianeCfg ( ArianeCfg )
  ) ariane1 (.*);

  ariane #(
    .ArianeCfg ( ArianeCfg )
  ) ariane2 (
    //.l15_req_o   ( l15_req_o2   ),
    //.l15_rtrn_i  ( l15_rtrn_i2  ),
    .axi_req_o (axi_req_o2),
    .axi_resp_i (axi_resp_i2),
    .*
  );

endmodule // ariane

