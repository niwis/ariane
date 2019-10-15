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
// Date: 19.03.2017
// Description: Test-harness for Ariane
//              Instantiates an AXI-Bus and memories

module ariane_testharness #(
  parameter int unsigned AXI_USER_WIDTH    = 1,
  parameter int unsigned AXI_ADDRESS_WIDTH = 64,
  parameter int unsigned AXI_DATA_WIDTH    = 64,
`ifdef DROMAJO
  parameter bit          InclSimDTM        = 1'b0,
`else
  parameter bit          InclSimDTM        = 1'b1,
`endif
  parameter int unsigned NUM_WORDS         = 2**25,         // memory size
  parameter bit          StallRandomOutput = 1'b0,
  parameter bit          StallRandomInput  = 1'b0
) (
  input  logic                           clk_i,
  input  logic                           rtc_i,
  input  logic                           rst_ni,
  output logic [31:0]                    exit_o
);

  // disable test-enable
  logic        test_en;
  logic        ndmreset;
  logic        ndmreset_n;
  logic        debug_req_core;

  int          jtag_enable;
  logic        init_done;
  logic [31:0] jtag_exit, dmi_exit;

  logic        jtag_TCK;
  logic        jtag_TMS;
  logic        jtag_TDI;
  logic        jtag_TRSTn;
  logic        jtag_TDO_data;
  logic        jtag_TDO_driven;

  logic        debug_req_valid;
  logic        debug_req_ready;
  logic        debug_resp_valid;
  logic        debug_resp_ready;

  logic        jtag_req_valid;
  logic [6:0]  jtag_req_bits_addr;
  logic [1:0]  jtag_req_bits_op;
  logic [31:0] jtag_req_bits_data;
  logic        jtag_resp_ready;
  logic        jtag_resp_valid;

  logic        dmi_req_valid;
  logic        dmi_resp_ready;
  logic        dmi_resp_valid;

  dm::dmi_req_t  jtag_dmi_req;
  dm::dmi_req_t  dmi_req;

  dm::dmi_req_t  debug_req;
  dm::dmi_resp_t debug_resp;

  assign test_en = 1'b0;

  //AXI_BUS #(
  //  .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH   ),
  //  .AXI_DATA_WIDTH ( AXI_DATA_WIDTH      ),
  //  .AXI_ID_WIDTH   ( ariane_soc::IdWidth ),
  //  .AXI_USER_WIDTH ( AXI_USER_WIDTH      )
  //) slave[ariane_soc::NrSlaves-1:0]();

 // AXI_BUS #(
 //   .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
 //   .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
 //   .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
 //   .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
 // ) master[ariane_soc::NB_PERIPHERALS-1:0]();

  // axi busses to the crossbar
  ariane_axi::req_t  [ariane_soc::NrSlaves-1:0] slv_ports_req;
  ariane_axi::resp_t [ariane_soc::NrSlaves-1:0] slv_ports_resp;

  ariane_axi::req_slv_t  [ariane_soc::NB_PERIPHERALS-1:0] mst_ports_req;
  ariane_axi::resp_slv_t [ariane_soc::NB_PERIPHERALS-1:0] mst_ports_resp;


  rstgen i_rstgen_main (
    .clk_i        ( clk_i                ),
    .rst_ni       ( rst_ni & (~ndmreset) ),
    .test_mode_i  ( test_en              ),
    .rst_no       ( ndmreset_n           ),
    .init_no      (                      ) // keep open
  );

  // ---------------
  // Debug
  // ---------------
  assign init_done = rst_ni;

  initial begin
    if (!$value$plusargs("jtag_rbb_enable=%b", jtag_enable)) jtag_enable = 'h0;
    if (riscv::XLEN != 32 & riscv::XLEN != 64) $error("XLEN different from 32 and 64");
  end

  // debug if MUX
  assign debug_req_valid     = (jtag_enable[0]) ? jtag_req_valid     : dmi_req_valid;
  assign debug_resp_ready    = (jtag_enable[0]) ? jtag_resp_ready    : dmi_resp_ready;
  assign debug_req           = (jtag_enable[0]) ? jtag_dmi_req       : dmi_req;
  assign exit_o              = (jtag_enable[0]) ? jtag_exit          : dmi_exit;
  assign jtag_resp_valid     = (jtag_enable[0]) ? debug_resp_valid   : 1'b0;
  assign dmi_resp_valid      = (jtag_enable[0]) ? 1'b0               : debug_resp_valid;

  // SiFive's SimJTAG Module
  // Converts to DPI calls
  SimJTAG i_SimJTAG (
    .clock                ( clk_i                ),
    .reset                ( ~rst_ni              ),
    .enable               ( jtag_enable[0]       ),
    .init_done            ( init_done            ),
    .jtag_TCK             ( jtag_TCK             ),
    .jtag_TMS             ( jtag_TMS             ),
    .jtag_TDI             ( jtag_TDI             ),
    .jtag_TRSTn           ( jtag_TRSTn           ),
    .jtag_TDO_data        ( jtag_TDO_data        ),
    .jtag_TDO_driven      ( jtag_TDO_driven      ),
    .exit                 ( jtag_exit            )
  );

  dmi_jtag i_dmi_jtag (
    .clk_i            ( clk_i           ),
    .rst_ni           ( rst_ni          ),
    .testmode_i       ( test_en         ),
    .dmi_req_o        ( jtag_dmi_req    ),
    .dmi_req_valid_o  ( jtag_req_valid  ),
    .dmi_req_ready_i  ( debug_req_ready ),
    .dmi_resp_i       ( debug_resp      ),
    .dmi_resp_ready_o ( jtag_resp_ready ),
    .dmi_resp_valid_i ( jtag_resp_valid ),
    .dmi_rst_no       (                 ), // not connected
    .tck_i            ( jtag_TCK        ),
    .tms_i            ( jtag_TMS        ),
    .trst_ni          ( jtag_TRSTn      ),
    .td_i             ( jtag_TDI        ),
    .td_o             ( jtag_TDO_data   ),
    .tdo_oe_o         ( jtag_TDO_driven )
  );

  // SiFive's SimDTM Module
  // Converts to DPI calls
  logic [1:0] debug_req_bits_op;
  assign dmi_req.op = dm::dtm_op_e'(debug_req_bits_op);

  if (InclSimDTM) begin
    SimDTM i_SimDTM (
      .clk                  ( clk_i                 ),
      .reset                ( ~rst_ni               ),
      .debug_req_valid      ( dmi_req_valid         ),
      .debug_req_ready      ( debug_req_ready       ),
      .debug_req_bits_addr  ( dmi_req.addr          ),
      .debug_req_bits_op    ( debug_req_bits_op     ),
      .debug_req_bits_data  ( dmi_req.data          ),
      .debug_resp_valid     ( dmi_resp_valid        ),
      .debug_resp_ready     ( dmi_resp_ready        ),
      .debug_resp_bits_resp ( debug_resp.resp       ),
      .debug_resp_bits_data ( debug_resp.data       ),
      .exit                 ( dmi_exit              )
    );
  end else begin
    assign dmi_req_valid = '0;
    assign debug_req_bits_op = '0;
    assign dmi_exit = 1'b0;
  end

  // this delay window allows the core to read and execute init code
  // from the bootrom before the first debug request can interrupt
  // core. this is needed in cases where an fsbl is involved that
  // expects a0 and a1 to be initialized with the hart id and a
  // pointer to the dev tree, respectively.
  localparam int unsigned DmiDelCycles = 500;

  logic debug_req_core_ungtd;
  int dmi_del_cnt_d, dmi_del_cnt_q;

  assign dmi_del_cnt_d  = (dmi_del_cnt_q) ? dmi_del_cnt_q - 1 : 0;
  assign debug_req_core = (dmi_del_cnt_q) ? 1'b0 : debug_req_core_ungtd;

  always_ff @(posedge clk_i or negedge rst_ni) begin : p_dmi_del_cnt
    if(!rst_ni) begin
      dmi_del_cnt_q <= DmiDelCycles;
    end else begin
      dmi_del_cnt_q <= dmi_del_cnt_d;
    end
  end

  ariane_axi_soc::req_t    dm_axi_m_req;
  ariane_axi_soc::resp_t   dm_axi_m_resp;

  logic                dm_slave_req;
  logic                dm_slave_we;
  logic [64-1:0]       dm_slave_addr;
  logic [64/8-1:0]     dm_slave_be;
  logic [64-1:0]       dm_slave_wdata;
  logic [64-1:0]       dm_slave_rdata;

  logic                dm_master_req;
  logic [64-1:0]       dm_master_add;
  logic                dm_master_we;
  logic [64-1:0]       dm_master_wdata;
  logic [64/8-1:0]     dm_master_be;
  logic                dm_master_gnt;
  logic                dm_master_r_valid;
  logic [64-1:0]       dm_master_r_rdata;

  // debug module
  dm_top #(
    .NrHarts              ( 1                           ),
    .BusWidth             ( AXI_DATA_WIDTH              ),
    .SelectableHarts      ( 1'b1                        )
  ) i_dm_top (
    .clk_i                ( clk_i                       ),
    .rst_ni               ( rst_ni                      ), // PoR
    .testmode_i           ( test_en                     ),
    .ndmreset_o           ( ndmreset                    ),
    .dmactive_o           (                             ), // active debug session
    .debug_req_o          ( debug_req_core_ungtd        ),
    .unavailable_i        ( '0                          ),
    .hartinfo_i           ( {ariane_pkg::DebugHartInfo} ),
    .slave_req_i          ( dm_slave_req                ),
    .slave_we_i           ( dm_slave_we                 ),
    .slave_addr_i         ( dm_slave_addr               ),
    .slave_be_i           ( dm_slave_be                 ),
    .slave_wdata_i        ( dm_slave_wdata              ),
    .slave_rdata_o        ( dm_slave_rdata              ),
    .master_req_o         ( dm_master_req               ),
    .master_add_o         ( dm_master_add               ),
    .master_we_o          ( dm_master_we                ),
    .master_wdata_o       ( dm_master_wdata             ),
    .master_be_o          ( dm_master_be                ),
    .master_gnt_i         ( dm_master_gnt               ),
    .master_r_valid_i     ( dm_master_r_valid           ),
    .master_r_rdata_i     ( dm_master_r_rdata           ),
    .dmi_rst_ni           ( rst_ni                      ),
    .dmi_req_valid_i      ( debug_req_valid             ),
    .dmi_req_ready_o      ( debug_req_ready             ),
    .dmi_req_i            ( debug_req                   ),
    .dmi_resp_valid_o     ( debug_resp_valid            ),
    .dmi_resp_ready_i     ( debug_resp_ready            ),
    .dmi_resp_o           ( debug_resp                  )
  );

  AXI_BUS #(
    .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) dm_bus ();

  axi_slave_connect_rev i_dm_connect (
    .axi_req_i ( mst_ports_req [ariane_soc::Debug] ),
    .axi_resp_o( mst_ports_resp[ariane_soc::Debug] ),
    .slave     ( dm_bus )
  );

  axi2mem #(
    .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) i_dm_axi2mem (
    .clk_i      ( clk_i                     ),
    .rst_ni     ( rst_ni                    ),
    .slave      ( dm_bus                    ),
    .req_o      ( dm_slave_req              ),
    .we_o       ( dm_slave_we               ),
    .addr_o     ( dm_slave_addr             ),
    .be_o       ( dm_slave_be               ),
    .data_o     ( dm_slave_wdata            ),
    .data_i     ( dm_slave_rdata            )
  );



  //axi_master_connect i_dm_axi_master_connect (
  //  .axi_req_i(dm_axi_m_req),
  //  .axi_resp_o(dm_axi_m_resp),
  //  .master(slave[1])
  //);

  axi_adapter #(
    .DATA_WIDTH            ( AXI_DATA_WIDTH            )
  ) i_dm_axi_master (
    .clk_i                 ( clk_i                     ),
    .rst_ni                ( rst_ni                    ),
    .req_i                 ( dm_master_req             ),
    .type_i                ( ariane_axi::SINGLE_REQ    ),
    .gnt_o                 ( dm_master_gnt             ),
    .gnt_id_o              (                           ),
    .addr_i                ( dm_master_add             ),
    .we_i                  ( dm_master_we              ),
    .wdata_i               ( dm_master_wdata           ),
    .be_i                  ( dm_master_be              ),
    .size_i                ( 2'b11                     ), // always do 64bit here and use byte enables to gate
    .id_i                  ( '0                        ),
    .valid_o               ( dm_master_r_valid         ),
    .rdata_o               ( dm_master_r_rdata         ),
    .id_o                  (                           ),
    .critical_word_o       (                           ),
    .critical_word_valid_o (                           ),
    .axi_req_o             ( slv_ports_req[1]          ), // dm_axi_m_req              ),
    .axi_resp_i            ( slv_ports_resp[1]         )  // dm_axi_m_resp             )
  );


  // ---------------
  // ROM
  // ---------------
  logic                         rom_req;
  logic [AXI_ADDRESS_WIDTH-1:0] rom_addr;
  logic [AXI_DATA_WIDTH-1:0]    rom_rdata;

  AXI_BUS #(
      .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
      .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
      .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
      .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) rom_bus();
  axi_slave_connect_rev i_rom_connect_rev (
    .axi_req_i  ( mst_ports_req [ariane_soc::ROM] ),
    .axi_resp_o ( mst_ports_resp[ariane_soc::ROM] ),
    .slave      ( rom_bus                         )
  );

  axi2mem #(
    .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) i_axi2rom (
    .clk_i  ( clk_i                   ),
    .rst_ni ( ndmreset_n              ),
    .slave  ( rom_bus                 ), // master[ariane_soc::ROM] ),
    .req_o  ( rom_req                 ),
    .we_o   (                         ),
    .addr_o ( rom_addr                ),
    .be_o   (                         ),
    .data_o (                         ),
    .data_i ( rom_rdata               )
  );

`ifdef DROMAJO
  dromajo_bootrom i_bootrom (
    .clk_i      ( clk_i     ),
    .req_i      ( rom_req   ),
    .addr_i     ( rom_addr  ),
    .rdata_o    ( rom_rdata )
  );
`else
  bootrom i_bootrom (
    .clk_i      ( clk_i     ),
    .req_i      ( rom_req   ),
    .addr_i     ( rom_addr  ),
    .rdata_o    ( rom_rdata )
  );
`endif
  // ------------------------------
  // Memory + LLC + Exclusive Access
  // ------------------------------
  AXI_BUS #(
    .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) dram();

  logic                         req;
  logic                         we;
  logic [AXI_ADDRESS_WIDTH-1:0] addr;
  logic [AXI_DATA_WIDTH/8-1:0]  be;
  logic [AXI_DATA_WIDTH-1:0]    wdata;
  logic [AXI_DATA_WIDTH-1:0]    rdata;

  ariane_axi::req_slv_t  llc_req;
  ariane_axi::resp_slv_t llc_resp;

  // AXI_BUS #(
  //     .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
  //     .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
  //     .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
  //     .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  // ) atomics_bus();
  // axi_slave_connect_rev i_atomics_connect_rev (
  //   .axi_req_i  ( mst_ports_req [ariane_soc::DRAM] ),
  //   .axi_resp_o ( mst_ports_resp[ariane_soc::DRAM] ),
  //   .slave      ( atomics_bus                      )
  // );

  axi_riscv_atomics #(
    .AXI_ADDR_WIDTH    ( AXI_ADDRESS_WIDTH        ),
    .AXI_DATA_WIDTH    ( AXI_DATA_WIDTH           ),
    .AXI_ID_WIDTH      ( ariane_soc::IdWidthSlave ),
    .AXI_USER_WIDTH    ( AXI_USER_WIDTH           ),
    .AXI_MAX_WRITE_TXNS( 1                        ),
    .RISCV_WORD_WIDTH  ( 64                       )
  ) i_axi_riscv_atomics (
    .clk_i           ( clk_i          ),
    .rst_ni          ( ndmreset_n     ),
    .slv_aw_id_i     ( mst_ports_req [ariane_soc::DRAM].aw.id     ),
    .slv_aw_addr_i   ( mst_ports_req [ariane_soc::DRAM].aw.addr   ),
    .slv_aw_prot_i   ( mst_ports_req [ariane_soc::DRAM].aw.prot   ),
    .slv_aw_region_i ( mst_ports_req [ariane_soc::DRAM].aw.region ),
    .slv_aw_atop_i   ( mst_ports_req [ariane_soc::DRAM].aw.atop   ),
    .slv_aw_len_i    ( mst_ports_req [ariane_soc::DRAM].aw.len    ),
    .slv_aw_size_i   ( mst_ports_req [ariane_soc::DRAM].aw.size   ),
    .slv_aw_burst_i  ( mst_ports_req [ariane_soc::DRAM].aw.burst  ),
    .slv_aw_lock_i   ( mst_ports_req [ariane_soc::DRAM].aw.lock   ),
    .slv_aw_cache_i  ( mst_ports_req [ariane_soc::DRAM].aw.cache  ),
    .slv_aw_qos_i    ( mst_ports_req [ariane_soc::DRAM].aw.qos    ),
    .slv_aw_user_i   ( '0                                         ),
    .slv_aw_valid_i  ( mst_ports_req [ariane_soc::DRAM].aw_valid  ),
    .slv_aw_ready_o  ( mst_ports_resp[ariane_soc::DRAM].aw_ready  ),

    .slv_ar_id_i     ( mst_ports_req [ariane_soc::DRAM].ar.id     ),
    .slv_ar_addr_i   ( mst_ports_req [ariane_soc::DRAM].ar.addr   ),
    .slv_ar_prot_i   ( mst_ports_req [ariane_soc::DRAM].ar.prot   ),
    .slv_ar_region_i ( mst_ports_req [ariane_soc::DRAM].ar.region ),
    .slv_ar_len_i    ( mst_ports_req [ariane_soc::DRAM].ar.len    ),
    .slv_ar_size_i   ( mst_ports_req [ariane_soc::DRAM].ar.size   ),
    .slv_ar_burst_i  ( mst_ports_req [ariane_soc::DRAM].ar.burst  ),
    .slv_ar_lock_i   ( mst_ports_req [ariane_soc::DRAM].ar.lock   ),
    .slv_ar_cache_i  ( mst_ports_req [ariane_soc::DRAM].ar.cache  ),
    .slv_ar_qos_i    ( mst_ports_req [ariane_soc::DRAM].ar.qos    ),
    .slv_ar_user_i   ( '0                                         ),
    .slv_ar_valid_i  ( mst_ports_req [ariane_soc::DRAM].ar_valid  ),
    .slv_ar_ready_o  ( mst_ports_resp[ariane_soc::DRAM].ar_ready  ),

    .slv_w_data_i    ( mst_ports_req [ariane_soc::DRAM].w.data    ),
    .slv_w_strb_i    ( mst_ports_req [ariane_soc::DRAM].w.strb    ),
    .slv_w_user_i    ( '0                                         ),
    .slv_w_last_i    ( mst_ports_req [ariane_soc::DRAM].w.last    ),
    .slv_w_valid_i   ( mst_ports_req [ariane_soc::DRAM].w_valid   ),
    .slv_w_ready_o   ( mst_ports_resp[ariane_soc::DRAM].w_ready   ),

    .slv_r_id_o      ( mst_ports_resp[ariane_soc::DRAM].r.id      ),
    .slv_r_data_o    ( mst_ports_resp[ariane_soc::DRAM].r.data    ),
    .slv_r_resp_o    ( mst_ports_resp[ariane_soc::DRAM].r.resp    ),
    .slv_r_last_o    ( mst_ports_resp[ariane_soc::DRAM].r.last    ),
    .slv_r_user_o    (                                            ),
    .slv_r_valid_o   ( mst_ports_resp[ariane_soc::DRAM].r_valid   ),
    .slv_r_ready_i   ( mst_ports_req [ariane_soc::DRAM].r_ready   ),

    .slv_b_id_o      ( mst_ports_resp[ariane_soc::DRAM].b.id      ),
    .slv_b_resp_o    ( mst_ports_resp[ariane_soc::DRAM].b.resp    ),
    .slv_b_user_o    (                                            ),
    .slv_b_valid_o   ( mst_ports_resp[ariane_soc::DRAM].b_valid   ),
    .slv_b_ready_i   ( mst_ports_req [ariane_soc::DRAM].b_ready   ),

    .mst_aw_id_o     ( llc_req.aw.id     ),
    .mst_aw_addr_o   ( llc_req.aw.addr   ),
    .mst_aw_prot_o   ( llc_req.aw.prot   ),
    .mst_aw_region_o ( llc_req.aw.region ),
    .mst_aw_atop_o   ( llc_req.aw.atop   ),
    .mst_aw_len_o    ( llc_req.aw.len    ),
    .mst_aw_size_o   ( llc_req.aw.size   ),
    .mst_aw_burst_o  ( llc_req.aw.burst  ),
    .mst_aw_lock_o   ( llc_req.aw.lock   ),
    .mst_aw_cache_o  ( llc_req.aw.cache  ),
    .mst_aw_qos_o    ( llc_req.aw.qos    ),
    .mst_aw_user_o   (                   ),
    .mst_aw_valid_o  ( llc_req.aw_valid  ),
    .mst_aw_ready_i  ( llc_resp.aw_ready ),

    .mst_ar_id_o     ( llc_req.ar.id     ),
    .mst_ar_addr_o   ( llc_req.ar.addr   ),
    .mst_ar_prot_o   ( llc_req.ar.prot   ),
    .mst_ar_region_o ( llc_req.ar.region ),
    .mst_ar_len_o    ( llc_req.ar.len    ),
    .mst_ar_size_o   ( llc_req.ar.size   ),
    .mst_ar_burst_o  ( llc_req.ar.burst  ),
    .mst_ar_lock_o   ( llc_req.ar.lock   ),
    .mst_ar_cache_o  ( llc_req.ar.cache  ),
    .mst_ar_qos_o    ( llc_req.ar.qos    ),
    .mst_ar_user_o   (                   ),
    .mst_ar_valid_o  ( llc_req.ar_valid  ),
    .mst_ar_ready_i  ( llc_resp.ar_ready ),

    .mst_w_data_o    ( llc_req.w.data    ),
    .mst_w_strb_o    ( llc_req.w.strb    ),
    .mst_w_last_o    ( llc_req.w.last    ),
    .mst_w_user_o    (                   ),
    .mst_w_valid_o   ( llc_req.w_valid   ),
    .mst_w_ready_i   ( llc_resp.w_ready  ),

    .mst_r_id_i      ( llc_resp.r.id     ),
    .mst_r_data_i    ( llc_resp.r.data   ),
    .mst_r_resp_i    ( llc_resp.r.resp   ),
    .mst_r_last_i    ( llc_resp.r.last   ),
    .mst_r_user_i    ( '0                ),
    .mst_r_valid_i   ( llc_resp.r_valid  ),
    .mst_r_ready_o   ( llc_req.r_ready   ),

    .mst_b_id_i      ( llc_resp.b.id     ),
    .mst_b_resp_i    ( llc_resp.b.resp   ),
    .mst_b_user_i    ( '0                ),
    .mst_b_valid_i   ( llc_resp.b_valid  ),
    .mst_b_ready_o   ( llc_req.b_ready   )
  );

  ariane_axi::req_slv_t  dram_req;
  ariane_axi::resp_slv_t dram_resp;

  //TODO: put llc here
  //assign dram_req = llc_req;
  //assign llc_resp = dram_resp;

  localparam llc_pkg::llc_axi_cfg_t LlcAxiCfg = '{
    SlvPortIdWidth:    ariane_soc::IdWidthSlave,
    MstPortIdWidth:    ariane_soc::IdWidthSlave,
    AddrWidthFull:     AXI_ADDRESS_WIDTH,
    DataWidthFull:     AXI_DATA_WIDTH,
    LitePortAddrWidth: AXI_ADDRESS_WIDTH,
    LitePortDataWidth: 32
  };
  typedef logic [AXI_ADDRESS_WIDTH-1:0] addr_t;
  typedef logic [31:0]                  data_lite_t;
  typedef logic [3:0]                   strb_lite_t;

  // AXI4 Lite
  typedef struct packed {
    addr_t          addr;
    // axi_pkg::prot_t prot; // not in pulp axi
  } aw_chan_lite_t;
  typedef struct packed {
    data_lite_t     data;
    strb_lite_t     strb;
  } w_chan_lite_t;
  typedef struct packed {
    axi_pkg::resp_t resp;
  } b_chan_lite_t;
  typedef struct packed {
    addr_t          addr;
    // axi_pkg::prot_t prot; // not in pulp axi
  } ar_chan_lite_t;
  typedef struct packed {
    data_lite_t     data;
    axi_pkg::resp_t resp;
  } r_chan_lite_t;
  typedef struct packed {
    aw_chan_lite_t  aw;
    logic           aw_valid;
    w_chan_lite_t   w;
    logic           w_valid;
    logic           b_ready;
    ar_chan_lite_t  ar;
    logic           ar_valid;
    logic           r_ready;
  } req_lite_t;
  typedef struct packed {
    logic           aw_ready;
    logic           w_ready;
    b_chan_lite_t   b;
    logic           b_valid;
    logic           ar_ready;
    r_chan_lite_t   r;
    logic           r_valid;
  } resp_lite_t;

  llc #(
    .SetAssociativity( 8                         ),
    .NoLines         ( 1024                      ),
    .NoBlocks        ( 8                         ),
    .AxiCfg          ( LlcAxiCfg                 ),
    .SPM_BASE        ( 64'h0000_0000             ),
    .RAM_BASE        ( ariane_soc::DRAMBase      ),
    .SPM_LENGTH      ( 64'h0020_0000             ),
    .RAM_LENGTH      ( ariane_soc::DRAMLength    ),
    .CFG_BASE        ( 64'h0000_0000             ),
    .slv_aw_chan_t   ( ariane_axi::aw_chan_slv_t ),
    .mst_aw_chan_t   ( ariane_axi::aw_chan_slv_t ),
    .w_chan_t        ( ariane_axi::w_chan_t      ),
    .slv_b_chan_t    ( ariane_axi::b_chan_slv_t  ),
    .mst_b_chan_t    ( ariane_axi::b_chan_slv_t  ),
    .slv_ar_chan_t   ( ariane_axi::ar_chan_slv_t ),
    .mst_ar_chan_t   ( ariane_axi::ar_chan_slv_t ),
    .slv_r_chan_t    ( ariane_axi::r_chan_slv_t  ),
    .mst_r_chan_t    ( ariane_axi::r_chan_slv_t  ),
    .slv_req_t       ( ariane_axi::req_slv_t     ),
    .slv_resp_t      ( ariane_axi::resp_slv_t    ),
    .mst_req_t       ( ariane_axi::req_slv_t     ),
    .mst_resp_t      ( ariane_axi::resp_slv_t    ),
    .lite_aw_chan_t  ( aw_chan_lite_t            ),
    .lite_w_chan_t   ( w_chan_lite_t             ),
    .lite_b_chan_t   ( b_chan_lite_t             ),
    .lite_ar_chan_t  ( ar_chan_lite_t            ),
    .lite_r_chan_t   ( r_chan_lite_t             ),
    .lite_req_t      ( req_lite_t                ),
    .lite_resp_t     ( resp_lite_t               ),
    .rule_t          ( axi_pkg::xbar_rule_64_t   )
  ) i_llc (
    .clk_i       ( clk_i          ),
    .rst_ni      ( ndmreset_n     ),
    .test_i      ( test_en        ),
    .slv_req_i   ( llc_req        ),
    .slv_resp_o  ( llc_resp       ),
    .mst_req_o   ( dram_req       ),
    .mst_resp_i  ( dram_resp      ),
    .conf_req_i  ( '0             ),
    .conf_resp_o (                )
  );

  //axi_riscv_atomics_wrap #(
  //  .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
  //  .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
  //  .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
  //  .AXI_USER_WIDTH ( AXI_USER_WIDTH           ),
  //  .AXI_MAX_WRITE_TXNS ( 1  ),
  //  .RISCV_WORD_WIDTH   ( 64 )
  //) i_axi_riscv_atomics (
  //  .clk_i,
  //  .rst_ni ( ndmreset_n               ),
  //  .slv    ( atomics_bus              ), // master[ariane_soc::DRAM] ),
  //  .mst    ( dram                     )
  //);

//  ariane_axi::aw_chan_slv_t aw_chan_i;
//  ariane_axi::w_chan_t      w_chan_i;
//  ariane_axi::b_chan_slv_t  b_chan_o;
//  ariane_axi::ar_chan_slv_t ar_chan_i;
//  ariane_axi::r_chan_slv_t  r_chan_o;

  ariane_axi::aw_chan_slv_t aw_chan_o;
  ariane_axi::w_chan_t      w_chan_o;
  ariane_axi::b_chan_slv_t  b_chan_i;
  ariane_axi::ar_chan_slv_t ar_chan_o;
  ariane_axi::r_chan_slv_t  r_chan_i;

  AXI_BUS #(
    .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) dram_delayed();

  ariane_axi_soc::aw_chan_slv_t aw_chan_i;
  ariane_axi_soc::w_chan_t      w_chan_i;
  ariane_axi_soc::b_chan_slv_t  b_chan_o;
  ariane_axi_soc::ar_chan_slv_t ar_chan_i;
  ariane_axi_soc::r_chan_slv_t  r_chan_o;
  ariane_axi_soc::aw_chan_slv_t aw_chan_o;
  ariane_axi_soc::w_chan_t      w_chan_o;
  ariane_axi_soc::b_chan_slv_t  b_chan_i;
  ariane_axi_soc::ar_chan_slv_t ar_chan_o;
  ariane_axi_soc::r_chan_slv_t  r_chan_i;

  axi_delayer #(
    .aw_t              ( ariane_axi_soc::aw_chan_slv_t ),
    .w_t               ( ariane_axi_soc::w_chan_t      ),
    .b_t               ( ariane_axi_soc::b_chan_slv_t  ),
    .ar_t              ( ariane_axi_soc::ar_chan_slv_t ),
    .r_t               ( ariane_axi_soc::r_chan_slv_t  ),
    .StallRandomOutput ( StallRandomOutput         ),
    .StallRandomInput  ( StallRandomInput          ),
    .FixedDelayInput   ( 0                         ),
    .FixedDelayOutput  ( 0                         )
  ) i_axi_delayer (
    .clk_i      ( clk_i                 ),
    .rst_ni     ( ndmreset_n            ),
    .aw_chan_i  ( dram_req.aw           ),
    .aw_valid_i ( dram_req.aw_valid     ),
    .aw_ready_o ( dram_resp.aw_ready    ),

    .w_chan_i   ( dram_req.w            ),
    .w_valid_i  ( dram_req.w_valid      ),
    .w_ready_o  ( dram_resp.w_ready     ),

    .b_chan_o   ( dram_resp.b           ),
    .b_valid_o  ( dram_resp.b_valid     ),
    .b_ready_i  ( dram_req.b_ready      ),

    .ar_chan_i  ( dram_req.ar           ),
    .ar_valid_i ( dram_req.ar_valid     ),
    .ar_ready_o ( dram_resp.ar_ready    ),

    .r_chan_o   ( dram_resp.r           ),
    .r_valid_o  ( dram_resp.r_valid     ),
    .r_ready_i  ( dram_req.r_ready      ),

    .aw_valid_o ( dram_delayed.aw_valid ),
    .aw_chan_o  ( aw_chan_o             ),
    .aw_ready_i ( dram_delayed.aw_ready ),
    .w_valid_o  ( dram_delayed.w_valid  ),
    .w_chan_o   ( w_chan_o              ),
    .w_ready_i  ( dram_delayed.w_ready  ),
    .b_valid_i  ( dram_delayed.b_valid  ),
    .b_chan_i   ( b_chan_i              ),
    .b_ready_o  ( dram_delayed.b_ready  ),
    .ar_valid_o ( dram_delayed.ar_valid ),
    .ar_chan_o  ( ar_chan_o             ),
    .ar_ready_i ( dram_delayed.ar_ready ),
    .r_valid_i  ( dram_delayed.r_valid  ),
    .r_chan_i   ( r_chan_i              ),
    .r_ready_o  ( dram_delayed.r_ready  )
  );

//  assign aw_chan_i.atop = dram.aw_atop;
//  assign aw_chan_i.id = dram.aw_id;
//  assign aw_chan_i.addr = dram.aw_addr;
//  assign aw_chan_i.len = dram.aw_len;
//  assign aw_chan_i.size = dram.aw_size;
//  assign aw_chan_i.burst = dram.aw_burst;
//  assign aw_chan_i.lock = dram.aw_lock;
//  assign aw_chan_i.cache = dram.aw_cache;
//  assign aw_chan_i.prot = dram.aw_prot;
//  assign aw_chan_i.qos = dram.aw_qos;
//  assign aw_chan_i.region = dram.aw_region;
//
//  assign ar_chan_i.id = dram.ar_id;
//  assign ar_chan_i.addr = dram.ar_addr;
//  assign ar_chan_i.len = dram.ar_len;
//  assign ar_chan_i.size = dram.ar_size;
//  assign ar_chan_i.burst = dram.ar_burst;
//  assign ar_chan_i.lock = dram.ar_lock;
//  assign ar_chan_i.cache = dram.ar_cache;
//  assign ar_chan_i.prot = dram.ar_prot;
//  assign ar_chan_i.qos = dram.ar_qos;
//  assign ar_chan_i.region = dram.ar_region;
//
//  assign w_chan_i.data = dram.w_data;
//  assign w_chan_i.strb = dram.w_strb;
//  assign w_chan_i.last = dram.w_last;
//
//  assign dram.r_id = r_chan_o.id;
//  assign dram.r_data = r_chan_o.data;
//  assign dram.r_resp = r_chan_o.resp;
//  assign dram.r_last = r_chan_o.last;
//
//  assign dram.b_id = b_chan_o.id;
//  assign dram.b_resp = b_chan_o.resp;

  assign dram_delayed.aw_id     = aw_chan_o.id;
  assign dram_delayed.aw_addr   = aw_chan_o.addr;
  assign dram_delayed.aw_len    = aw_chan_o.len;
  assign dram_delayed.aw_size   = aw_chan_o.size;
  assign dram_delayed.aw_burst  = aw_chan_o.burst;
  assign dram_delayed.aw_lock   = aw_chan_o.lock;
  assign dram_delayed.aw_cache  = aw_chan_o.cache;
  assign dram_delayed.aw_prot   = aw_chan_o.prot;
  assign dram_delayed.aw_qos    = aw_chan_o.qos;
  assign dram_delayed.aw_atop   = aw_chan_o.atop;
  assign dram_delayed.aw_region = aw_chan_o.region;
  assign dram_delayed.aw_user   = '0;

  assign dram_delayed.ar_id     = ar_chan_o.id;
  assign dram_delayed.ar_addr   = ar_chan_o.addr;
  assign dram_delayed.ar_len    = ar_chan_o.len;
  assign dram_delayed.ar_size   = ar_chan_o.size;
  assign dram_delayed.ar_burst  = ar_chan_o.burst;
  assign dram_delayed.ar_lock   = ar_chan_o.lock;
  assign dram_delayed.ar_cache  = ar_chan_o.cache;
  assign dram_delayed.ar_prot   = ar_chan_o.prot;
  assign dram_delayed.ar_qos    = ar_chan_o.qos;
  assign dram_delayed.ar_region = ar_chan_o.region;
  assign dram_delayed.ar_user   = '0;

  assign dram_delayed.w_data = w_chan_o.data;
  assign dram_delayed.w_strb = w_chan_o.strb;
  assign dram_delayed.w_last = w_chan_o.last;
  assign dram_delayed.w_user = '0;

  assign r_chan_i.id   = dram_delayed.r_id;
  assign r_chan_i.data = dram_delayed.r_data;
  assign r_chan_i.resp = dram_delayed.r_resp;
  assign r_chan_i.last = dram_delayed.r_last;
  // assign r_chan_i.  dram.r_user = '0;

  assign b_chan_i.id   = dram_delayed.b_id;
  assign b_chan_i.resp = dram_delayed.b_resp;
  // assign dram.b_user = '0;

  axi2mem #(
    .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) i_axi2mem (
    .clk_i  ( clk_i        ),
    .rst_ni ( ndmreset_n   ),
    .slave  ( dram_delayed ),
    .req_o  ( req          ),
    .we_o   ( we           ),
    .addr_o ( addr         ),
    .be_o   ( be           ),
    .data_o ( wdata        ),
    .data_i ( rdata        )
  );

  sram #(
    .DATA_WIDTH ( AXI_DATA_WIDTH ),
`ifdef DROMAJO
    .DROMAJO_RAM (1),
`endif
    .NUM_WORDS  ( NUM_WORDS      )
  ) i_sram (
    .clk_i      ( clk_i                                                                       ),
    .rst_ni     ( rst_ni                                                                      ),
    .req_i      ( req                                                                         ),
    .we_i       ( we                                                                          ),
    .addr_i     ( addr[$clog2(NUM_WORDS)-1+$clog2(AXI_DATA_WIDTH/8):$clog2(AXI_DATA_WIDTH/8)] ),
    .wdata_i    ( wdata                                                                       ),
    .be_i       ( be                                                                          ),
    .rdata_o    ( rdata                                                                       )
  );

  // ---------------
  // AXI Xbar
  // ---------------
  localparam axi_pkg::xbar_cfg_t XbarCfg = '{
    NoSlvPorts:         ariane_soc::NrSlaves,       // # of slave ports, so many masters are connected to the xbar
    NoMstPorts:         ariane_soc::NB_PERIPHERALS, // # of master ports, so many slaves are connected to the xbar
    MaxMstTrans:        8,                          // Maxi # of outstanding transactions per r/w per master
    MaxSlvTrans:        8,                          // Maxi # of outstanding write transactions per slave
    FallThrough:        1'b0,                       // AreAW -> W Fifo's in Fall through mode (1'b0 = long paths)
    LatencyMode:        axi_pkg::CUT_ALL_AX,        // See xbar_latency_t and get_xbarlatmode
    AxiIdWidthSlvPorts: ariane_soc::IdWidth,        // Axi Id Width of the Slave Ports
    AxiIdUsedSlvPorts:  ariane_soc::IdWidth - 1,    // this many LSB's of the SlvPortAxiId get used in demux
    AxiIdWidthMstPorts: ariane_soc::IdWidthSlave,   // ==> $clog2(NoSLVPorts) + AxiIdWidthSlvPorts !!
    AxiAddrWidth:       AXI_ADDRESS_WIDTH,          // Axi Address Width
    AxiDataWidth:       AXI_DATA_WIDTH,             // Axi Data Width
    NoAddrRules:        ariane_soc::NB_PERIPHERALS + 1  // # of Address Rules in the memory map
  };
  // localparam axi_pkg::xbar_rule_64_t [ariane_soc::NB_PERIPHERALS] AddrMap = '{
  //   '{mst_port_idx: ariane_soc::DRAM,
  //     start_addr:   ariane_soc::DRAMBase,
  //     end_addr:     ariane_soc::DRAMBase     + ariane_soc::DRAMLength    },
  //   '{mst_port_idx: ariane_soc::GPIO,
  //     start_addr:   ariane_soc::GPIOBase,
  //     end_addr:     ariane_soc::GPIOBase     + ariane_soc::GPIOLength    },
  //   '{mst_port_idx: ariane_soc::Ethernet,
  //     start_addr:   ariane_soc::EthernetBase,
  //     end_addr:     ariane_soc::EthernetBase + ariane_soc::EthernetLength},
  //   '{mst_port_idx: ariane_soc::SPI,
  //     start_addr:   ariane_soc::SPIBase,
  //     end_addr:     ariane_soc::SPIBase      + ariane_soc::SPILength     },
  //   '{mst_port_idx: ariane_soc::UART,
  //     start_addr:   ariane_soc::UARTBase,
  //     end_addr:     ariane_soc::UARTBase     + ariane_soc::UARTLength    },
  //   '{mst_port_idx: ariane_soc::PLIC,
  //     start_addr:   ariane_soc::PLICBase,
  //     end_addr:     ariane_soc::PLICBase     + ariane_soc::PLICLength    },
  //   '{mst_port_idx: ariane_soc::CLINT,
  //     start_addr:   ariane_soc::CLINTBase,
  //     end_addr:     ariane_soc::CLINTBase    + ariane_soc::CLINTLength   },
  //   '{mst_port_idx: ariane_soc::ROM,
  //     start_addr:   ariane_soc::ROMBase,
  //     end_addr:     ariane_soc::ROMLength    + ariane_soc::ROMLength     },
  //   '{mst_port_idx: ariane_soc::Debug,
  //     start_addr:   ariane_soc::DebugBase,
  //     end_addr:     ariane_soc::DebugBase    + ariane_soc::DebugLength   }
  // };

  assign mst_ports_resp[ariane_soc::LlcCfg] = '0;

  localparam axi_pkg::xbar_rule_64_t [ariane_soc::NB_PERIPHERALS + 1] AddrMap = '{
    '{mst_port_idx: ariane_soc::DRAM,
      start_addr:   ariane_soc::DRAMBase,
      end_addr:     ariane_soc::DRAMBase     + ariane_soc::DRAMLength    },
    '{mst_port_idx: ariane_soc::DRAM,
      start_addr:   ariane_soc::LlcSpmBase,
      end_addr:     ariane_soc::LlcSpmBase   + ariane_soc::LlcSpmLength  },
    '{mst_port_idx: ariane_soc::LlcCfg,
      start_addr:   ariane_soc::LlcCfgBase,
      end_addr:     ariane_soc::LlcCfgBase   + ariane_soc::LlcCfgLength  },
    '{mst_port_idx: ariane_soc::GPIO,
      start_addr:   ariane_soc::GPIOBase,
      end_addr:     ariane_soc::GPIOBase     + ariane_soc::GPIOLength    },
    '{mst_port_idx: ariane_soc::Ethernet,
      start_addr:   ariane_soc::EthernetBase,
      end_addr:     ariane_soc::EthernetBase + ariane_soc::EthernetLength},
    '{mst_port_idx: ariane_soc::SPI,
      start_addr:   ariane_soc::SPIBase,
      end_addr:     ariane_soc::SPIBase      + ariane_soc::SPILength     },
    '{mst_port_idx: ariane_soc::UART,
      start_addr:   ariane_soc::UARTBase,
      end_addr:     ariane_soc::UARTBase     + ariane_soc::UARTLength    },
    '{mst_port_idx: ariane_soc::PLIC,
      start_addr:   ariane_soc::PLICBase,
      end_addr:     ariane_soc::PLICBase     + ariane_soc::PLICLength    },
    '{mst_port_idx: ariane_soc::CLINT,
      start_addr:   ariane_soc::CLINTBase,
      end_addr:     ariane_soc::CLINTBase    + ariane_soc::CLINTLength   },
    '{mst_port_idx: ariane_soc::ROM,
      start_addr:   ariane_soc::ROMBase,
      end_addr:     ariane_soc::ROMLength    + ariane_soc::ROMLength     },
    '{mst_port_idx: ariane_soc::Debug,
      start_addr:   ariane_soc::DebugBase,
      end_addr:     ariane_soc::DebugBase    + ariane_soc::DebugLength   }
  };

  axi_xbar #(
    .Cfg            ( XbarCfg                   ),
    .slv_aw_chan_t  ( ariane_axi::aw_chan_t     ),
    .mst_aw_chan_t  ( ariane_axi::aw_chan_slv_t ),
    .w_chan_t       ( ariane_axi::w_chan_t      ),
    .slv_b_chan_t   ( ariane_axi::b_chan_t      ),
    .mst_b_chan_t   ( ariane_axi::b_chan_slv_t  ),
    .slv_ar_chan_t  ( ariane_axi::ar_chan_t     ),
    .mst_ar_chan_t  ( ariane_axi::ar_chan_slv_t ),
    .slv_r_chan_t   ( ariane_axi::r_chan_t      ),
    .mst_r_chan_t   ( ariane_axi::r_chan_slv_t  ),
    .slv_req_t      ( ariane_axi::req_t         ),
    .slv_resp_t     ( ariane_axi::resp_t        ),
    .mst_req_t      ( ariane_axi::req_slv_t     ),
    .mst_resp_t     ( ariane_axi::resp_slv_t    ),
    .rule_t         ( axi_pkg::xbar_rule_64_t   )
  ) i_axi_xbar (
    .clk_i  ( clk_i      ),
    .rst_ni ( rst_ni     ),
    .test_i ( 1'b0       ),
    // slave ports, connect here the master modules
    .slv_ports_req_i  ( slv_ports_req  ),
    .slv_ports_resp_o ( slv_ports_resp ),
    // master ports, connect here the slave modules
    .mst_ports_req_o  ( mst_ports_req  ),
    .mst_ports_resp_i ( mst_ports_resp ),
    // addr map input
    .addr_map_i       ( AddrMap        ),
    .en_default_mst_port_i ( '0        ),
    .default_mst_port_i    ( '0        )
  );
//  axi_node_intf_wrap #(
//    .NB_SLAVE           ( ariane_soc::NrSlaves       ),
//    .NB_MASTER          ( ariane_soc::NB_PERIPHERALS ),
//    .NB_REGION          ( ariane_soc::NrRegion       ),
//    .AXI_ADDR_WIDTH     ( AXI_ADDRESS_WIDTH          ),
//    .AXI_DATA_WIDTH     ( AXI_DATA_WIDTH             ),
//    .AXI_USER_WIDTH     ( AXI_USER_WIDTH             ),
//    .AXI_ID_WIDTH       ( ariane_soc::IdWidth        )
//    // .MASTER_SLICE_DEPTH ( 0                          ),
//    // .SLAVE_SLICE_DEPTH  ( 0                          )
//  ) i_axi_xbar (
//    .clk          ( clk_i      ),
//    .rst_n        ( ndmreset_n ),
//    .test_en_i    ( test_en    ),
//    .slave        ( slave      ),
//    .master       ( master     ),
//    .start_addr_i ({
//      ariane_soc::DebugBase,
//      ariane_soc::ROMBase,
//      ariane_soc::CLINTBase,
//      ariane_soc::PLICBase,
//      ariane_soc::UARTBase,
//      ariane_soc::SPIBase,
//      ariane_soc::EthernetBase,
//      ariane_soc::GPIOBase,
//      ariane_soc::DRAMBase
//    }),
//    .end_addr_i   ({
//      ariane_soc::DebugBase    + ariane_soc::DebugLength - 1,
//      ariane_soc::ROMBase      + ariane_soc::ROMLength - 1,
//      ariane_soc::CLINTBase    + ariane_soc::CLINTLength - 1,
//      ariane_soc::PLICBase     + ariane_soc::PLICLength - 1,
//      ariane_soc::UARTBase     + ariane_soc::UARTLength - 1,
//      ariane_soc::SPIBase      + ariane_soc::SPILength - 1,
//      ariane_soc::EthernetBase + ariane_soc::EthernetLength -1,
//      ariane_soc::GPIOBase     + ariane_soc::GPIOLength - 1,
//      ariane_soc::DRAMBase     + ariane_soc::DRAMLength - 1
//    }),
//    .valid_rule_i (ariane_soc::ValidRule)
//  );

  // ---------------
  // CLINT
  // ---------------
  logic ipi;
  logic timer_irq;

  ariane_axi_soc::req_t    axi_clint_req;
  ariane_axi_soc::resp_t   axi_clint_resp;

  clint #(
    .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    .NR_CORES       ( 1                        )
  ) i_clint (
    .clk_i       ( clk_i          ),
    .rst_ni      ( ndmreset_n     ),
    .testmode_i  ( test_en        ),
    .axi_req_i   ( mst_ports_req [ariane_soc::CLINT]), //  axi_clint_req  ),
    .axi_resp_o  ( mst_ports_resp[ariane_soc::CLINT]), //  axi_clint_resp ),
    .rtc_i       ( rtc_i          ),
    .timer_irq_o ( timer_irq      ),
    .ipi_o       ( ipi            )
  );

  // axi_slave_connect i_axi_slave_connect_clint (
  //   .axi_req_o(axi_clint_req),
  //   .axi_resp_i(axi_clint_resp),
  //   .slave(master[ariane_soc::CLINT])
  // );

  // ---------------
  // Peripherals
  // ---------------
  logic tx, rx;
  logic [1:0] irqs;

  AXI_BUS #(
      .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
      .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
      .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
      .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) plic_bus();
  axi_slave_connect_rev i_plic_connect_rev (
    .axi_req_i  ( mst_ports_req [ariane_soc::PLIC] ),
    .axi_resp_o ( mst_ports_resp[ariane_soc::PLIC] ),
    .slave      ( plic_bus                         )
  );
  AXI_BUS #(
      .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
      .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
      .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
      .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) uart_bus();
  axi_slave_connect_rev i_uart_connect_rev (
    .axi_req_i  ( mst_ports_req [ariane_soc::UART] ),
    .axi_resp_o ( mst_ports_resp[ariane_soc::UART] ),
    .slave      ( uart_bus                         )
  );
  AXI_BUS #(
      .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
      .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
      .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
      .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) spi_bus();
  axi_slave_connect_rev i_spi_connect_rev (
    .axi_req_i  ( mst_ports_req [ariane_soc::SPI] ),
    .axi_resp_o ( mst_ports_resp[ariane_soc::SPI] ),
    .slave      ( spi_bus                          )
  );
  AXI_BUS #(
      .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
      .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
      .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
      .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
  ) ethernet_bus();
  axi_slave_connect_rev i_ethernet_connect_rev (
    .axi_req_i  ( mst_ports_req [ariane_soc::Ethernet] ),
    .axi_resp_o ( mst_ports_resp[ariane_soc::Ethernet] ),
    .slave      ( ethernet_bus                     )
  );

  ariane_peripherals #(
    .AxiAddrWidth ( AXI_ADDRESS_WIDTH        ),
    .AxiDataWidth ( AXI_DATA_WIDTH           ),
    .AxiIdWidth   ( ariane_soc::IdWidthSlave ),
`ifndef VERILATOR
  // disable UART when using Spike, as we need to rely on the mockuart
  `ifdef SPIKE_TANDEM
    .InclUART     ( 1'b0                     ),
  `else
    .InclUART     ( 1'b1                     ),
  `endif
`else
    .InclUART     ( 1'b0                     ),
`endif
    .InclSPI      ( 1'b0                     ),
    .InclEthernet ( 1'b0                     )
  ) i_ariane_peripherals (
    .clk_i     ( clk_i                        ),
    .rst_ni    ( ndmreset_n                   ),
    .plic      ( plic_bus                     ),
    .uart      ( uart_bus                     ),
    .spi       ( spi_bus                      ),
    .ethernet  ( ethernet_bus                 ),
    .irq_o     ( irqs                         ),
    .rx_i      ( rx                           ),
    .tx_o      ( tx                           ),
    .eth_txck  ( ),
    .eth_rxck  ( ),
    .eth_rxctl ( ),
    .eth_rxd   ( ),
    .eth_rst_n ( ),
    .eth_tx_en ( ),
    .eth_txd   ( ),
    .phy_mdio  ( ),
    .eth_mdc   ( ),
    .mdio      ( ),
    .mdc       ( ),
    .spi_clk_o ( ),
    .spi_mosi  ( ),
    .spi_miso  ( ),
    .spi_ss    ( )
  );

  uart_bus #(.BAUD_RATE(115200), .PARITY_EN(0)) i_uart_bus (.rx(tx), .tx(rx), .rx_en(1'b1));

  // ---------------
  // Core
  // ---------------
  ariane_axi_soc::req_t    axi_ariane_req;
  ariane_axi_soc::resp_t   axi_ariane_resp;

  ariane #(
    .ArianeCfg  ( ariane_soc::ArianeSocCfg )
  ) i_ariane (
    .clk_i                ( clk_i               ),
    .rst_ni               ( ndmreset_n          ),
    .boot_addr_i          ( ariane_soc::ROMBase ), // start fetching from ROM
    .hart_id_i            ( '0                  ),
    .irq_i                ( irqs                ),
    .ipi_i                ( ipi                 ),
    .time_irq_i           ( timer_irq           ),
// Disable Debug when simulating with Spike
`ifdef SPIKE_TANDEM
    .debug_req_i          ( 1'b0                ),
`else
    .debug_req_i          ( debug_req_core      ),
`endif
    .axi_req_o            ( axi_ariane_req      ),
    .axi_resp_i           ( axi_ariane_resp     )
  );

  assign slv_ports_req[0] = axi_ariane_req;
  assign axi_ariane_resp  = slv_ports_resp[0];


  //axi_master_connect i_axi_master_connect_ariane (
  //  .axi_req_i(axi_ariane_req),
  //  .axi_resp_o(axi_ariane_resp),
  //  .master(slave[0])
  //);

  // -------------
  // Simulation Helper Functions
  // -------------
  // check for response errors
  always_ff @(posedge clk_i) begin : p_assert
    if (axi_ariane_req.r_ready &&
      axi_ariane_resp.r_valid &&
      axi_ariane_resp.r.resp inside {axi_pkg::RESP_DECERR, axi_pkg::RESP_SLVERR}) begin
      $warning("R Response Errored");
    end
    if (axi_ariane_req.b_ready &&
      axi_ariane_resp.b_valid &&
      axi_ariane_resp.b.resp inside {axi_pkg::RESP_DECERR, axi_pkg::RESP_SLVERR}) begin
      $warning("B Response Errored");
    end
  end

`ifdef AXI_SVA
  // AXI 4 Assertion IP integration - You will need to get your own copy of this IP if you want
  // to use it
  Axi4PC #(
    .DATA_WIDTH(ariane_axi_soc::DataWidth),
    .WID_WIDTH(ariane_soc::IdWidthSlave),
    .RID_WIDTH(ariane_soc::IdWidthSlave),
    .AWUSER_WIDTH(ariane_axi_soc::UserWidth),
    .WUSER_WIDTH(ariane_axi_soc::UserWidth),
    .BUSER_WIDTH(ariane_axi_soc::UserWidth),
    .ARUSER_WIDTH(ariane_axi_soc::UserWidth),
    .RUSER_WIDTH(ariane_axi_soc::UserWidth),
    .ADDR_WIDTH(ariane_axi_soc::AddrWidth)
  ) i_Axi4PC (
    .ACLK(clk_i),
    .ARESETn(ndmreset_n),
    .AWID(dram.aw_id),
    .AWADDR(dram.aw_addr),
    .AWLEN(dram.aw_len),
    .AWSIZE(dram.aw_size),
    .AWBURST(dram.aw_burst),
    .AWLOCK(dram.aw_lock),
    .AWCACHE(dram.aw_cache),
    .AWPROT(dram.aw_prot),
    .AWQOS(dram.aw_qos),
    .AWREGION(dram.aw_region),
    .AWUSER(dram.aw_user),
    .AWVALID(dram.aw_valid),
    .AWREADY(dram.aw_ready),
    .WLAST(dram.w_last),
    .WDATA(dram.w_data),
    .WSTRB(dram.w_strb),
    .WUSER(dram.w_user),
    .WVALID(dram.w_valid),
    .WREADY(dram.w_ready),
    .BID(dram.b_id),
    .BRESP(dram.b_resp),
    .BUSER(dram.b_user),
    .BVALID(dram.b_valid),
    .BREADY(dram.b_ready),
    .ARID(dram.ar_id),
    .ARADDR(dram.ar_addr),
    .ARLEN(dram.ar_len),
    .ARSIZE(dram.ar_size),
    .ARBURST(dram.ar_burst),
    .ARLOCK(dram.ar_lock),
    .ARCACHE(dram.ar_cache),
    .ARPROT(dram.ar_prot),
    .ARQOS(dram.ar_qos),
    .ARREGION(dram.ar_region),
    .ARUSER(dram.ar_user),
    .ARVALID(dram.ar_valid),
    .ARREADY(dram.ar_ready),
    .RID(dram.r_id),
    .RLAST(dram.r_last),
    .RDATA(dram.r_data),
    .RRESP(dram.r_resp),
    .RUSER(dram.r_user),
    .RVALID(dram.r_valid),
    .RREADY(dram.r_ready),
    .CACTIVE('0),
    .CSYSREQ('0),
    .CSYSACK('0)
  );
`endif
endmodule
