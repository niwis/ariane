// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Description: Xilinx FPGA top-level
// Author: Florian Zaruba <zarubaf@iis.ee.ethz.ch>

module ariane_xilinx (
`ifdef GENESYSII
  input  logic         sys_clk_p   ,
  input  logic         sys_clk_n   ,
  input  logic         cpu_resetn  ,
  inout  wire  [31:0]  ddr3_dq     ,
  inout  wire  [ 3:0]  ddr3_dqs_n  ,
  inout  wire  [ 3:0]  ddr3_dqs_p  ,
  output logic [14:0]  ddr3_addr   ,
  output logic [ 2:0]  ddr3_ba     ,
  output logic         ddr3_ras_n  ,
  output logic         ddr3_cas_n  ,
  output logic         ddr3_we_n   ,
  output logic         ddr3_reset_n,
  output logic [ 0:0]  ddr3_ck_p   ,
  output logic [ 0:0]  ddr3_ck_n   ,
  output logic [ 0:0]  ddr3_cke    ,
  output logic [ 0:0]  ddr3_cs_n   ,
  output logic [ 3:0]  ddr3_dm     ,
  output logic [ 0:0]  ddr3_odt    ,

  output wire          eth_rst_n   ,
  input  wire          eth_rxck    ,
  input  wire          eth_rxctl   ,
  input  wire [3:0]    eth_rxd     ,
  output wire          eth_txck    ,
  output wire          eth_txctl   ,
  output wire [3:0]    eth_txd     ,
  inout  wire          eth_mdio    ,
  output logic         eth_mdc     ,
  output logic [ 7:0]  led         ,
  input  logic [ 7:0]  sw          ,
  output logic         fan_pwm     ,
  input  logic         trst_n      ,
`elsif KC705
  input  logic         sys_clk_p   ,
  input  logic         sys_clk_n   ,

  input  logic         cpu_reset   ,
  inout  logic [63:0]  ddr3_dq     ,
  inout  logic [ 7:0]  ddr3_dqs_n  ,
  inout  logic [ 7:0]  ddr3_dqs_p  ,
  output logic [13:0]  ddr3_addr   ,
  output logic [ 2:0]  ddr3_ba     ,
  output logic         ddr3_ras_n  ,
  output logic         ddr3_cas_n  ,
  output logic         ddr3_we_n   ,
  output logic         ddr3_reset_n,
  output logic [ 0:0]  ddr3_ck_p   ,
  output logic [ 0:0]  ddr3_ck_n   ,
  output logic [ 0:0]  ddr3_cke    ,
  output logic [ 0:0]  ddr3_cs_n   ,
  output logic [ 7:0]  ddr3_dm     ,
  output logic [ 0:0]  ddr3_odt    ,

  output wire          eth_rst_n   ,
  input  wire          eth_rxck    ,
  input  wire          eth_rxctl   ,
  input  wire [3:0]    eth_rxd     ,
  output wire          eth_txck    ,
  output wire          eth_txctl   ,
  output wire [3:0]    eth_txd     ,
  inout  wire          eth_mdio    ,
  output logic         eth_mdc     ,
  output logic [ 3:0]  led         ,
  input  logic [ 3:0]  sw          ,
  output logic         fan_pwm     ,
  input  logic         trst_n      ,
`elsif VC707
  input  logic         sys_clk_p   ,
  input  logic         sys_clk_n   ,
  input  logic         cpu_reset   ,
  inout  wire  [63:0]  ddr3_dq     ,
  inout  wire  [ 7:0]  ddr3_dqs_n  ,
  inout  wire  [ 7:0]  ddr3_dqs_p  ,
  output logic [13:0]  ddr3_addr   ,
  output logic [ 2:0]  ddr3_ba     ,
  output logic         ddr3_ras_n  ,
  output logic         ddr3_cas_n  ,
  output logic         ddr3_we_n   ,
  output logic         ddr3_reset_n,
  output logic [ 0:0]  ddr3_ck_p   ,
  output logic [ 0:0]  ddr3_ck_n   ,
  output logic [ 0:0]  ddr3_cke    ,
  output logic [ 0:0]  ddr3_cs_n   ,
  output logic [ 7:0]  ddr3_dm     ,
  output logic [ 0:0]  ddr3_odt    ,
  output wire          eth_rst_n   ,
  input  wire          eth_rxck    ,
  input  wire          eth_rxctl   ,
  input  wire [3:0]    eth_rxd     ,
  output wire          eth_txck    ,
  output wire          eth_txctl   ,
  output wire [3:0]    eth_txd     ,
  inout  wire          eth_mdio    ,
  output logic         eth_mdc     ,
  output logic [ 7:0]  led         ,
  input  logic [ 7:0]  sw          ,
  output logic         fan_pwm     ,
  input  logic         trst        ,
`elsif VCU118
  input  wire          c0_sys_clk_p    ,  // 250 MHz Clock for DDR
  input  wire          c0_sys_clk_n    ,  // 250 MHz Clock for DDR
  input  wire          sys_clk_p       ,  // 100 MHz Clock for PCIe
  input  wire          sys_clk_n       ,  // 100 MHz Clock for PCIE
  input  wire          sys_rst_n       ,  // PCIe Reset
  input  logic         cpu_reset       ,  // CPU subsystem reset
  output wire [16:0]   c0_ddr4_adr     ,
  output wire [1:0]    c0_ddr4_ba      ,
  output wire [0:0]    c0_ddr4_cke     ,
  output wire [0:0]    c0_ddr4_cs_n    ,
  inout  wire [7:0]    c0_ddr4_dm_dbi_n,
  inout  wire [63:0]   c0_ddr4_dq      ,
  inout  wire [7:0]    c0_ddr4_dqs_c   ,
  inout  wire [7:0]    c0_ddr4_dqs_t   ,
  output wire [0:0]    c0_ddr4_odt     ,
  output wire [0:0]    c0_ddr4_bg      ,
  output wire          c0_ddr4_reset_n ,
  output wire          c0_ddr4_act_n   ,
  output wire [0:0]    c0_ddr4_ck_c    ,
  output wire [0:0]    c0_ddr4_ck_t    ,
  output wire [7:0]    pci_exp_txp     ,
  output wire [7:0]    pci_exp_txn     ,
  input  wire [7:0]    pci_exp_rxp     ,
  input  wire [7:0]    pci_exp_rxn     ,
  input  logic         trst_n          ,
`endif
  // SPI
  output logic        spi_mosi    ,
  input  logic        spi_miso    ,
  output logic        spi_ss      ,
  output logic        spi_clk_o   ,
  // common part
  // input logic      trst_n      ,
  input  logic        tck         ,
  input  logic        tms         ,
  input  logic        tdi         ,
  output wire         tdo         ,
  input  logic        rx          ,
  output logic        tx
);
// 24 MByte in 8 byte words
localparam NumWords = (24 * 1024 * 1024) / 8;
localparam NBSlave = 2; // debug, ariane
localparam AxiAddrWidth = 64;
localparam AxiDataWidth = 64;
localparam AxiIdWidthMaster = 4;
localparam AxiIdWidthSlaves = AxiIdWidthMaster + $clog2(NBSlave); // 5
localparam AxiUserWidth = 1;

//AXI_BUS #(
//    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
//    .AXI_DATA_WIDTH ( AxiDataWidth     ),
//    .AXI_ID_WIDTH   ( AxiIdWidthMaster ),
//    .AXI_USER_WIDTH ( AxiUserWidth     )
//) slave[NBSlave-1:0]();

AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) master[ariane_soc::NB_PERIPHERALS-1:0]();

// axi busses to the crossbar
ariane_axi::req_t  [NBSlave-1:0] slv_ports_req;
ariane_axi::resp_t [NBSlave-1:0] slv_ports_resp;

ariane_axi::req_slv_t  [ariane_soc::NB_PERIPHERALS-1:0] mst_ports_req;
ariane_axi::resp_slv_t [ariane_soc::NB_PERIPHERALS-1:0] mst_ports_resp;


// disable test-enable
logic test_en;
logic ndmreset;
logic ndmreset_n;
logic debug_req_irq;
logic timer_irq;
logic ipi;

logic clk;
logic eth_clk;
logic spi_clk_i;
logic phy_tx_clk;
logic sd_clk_sys;

logic ddr_sync_reset;
logic ddr_clock_out;

logic rst_n, rst;
logic rtc;

// we need to switch reset polarity
`ifdef VCU118
logic cpu_resetn;
assign cpu_resetn = ~cpu_reset;
`elsif GENESYSII
logic cpu_reset;
assign cpu_reset  = ~cpu_resetn;
`elsif KC705
assign cpu_resetn = ~cpu_reset;
`elsif VC707
assign cpu_resetn = ~cpu_reset;
assign trst_n = ~trst;
`endif

logic pll_locked;

// ROM
logic                    rom_req;
logic [AxiAddrWidth-1:0] rom_addr;
logic [AxiDataWidth-1:0] rom_rdata;

// Debug
logic          debug_req_valid;
logic          debug_req_ready;
dm::dmi_req_t  debug_req;
logic          debug_resp_valid;
logic          debug_resp_ready;
dm::dmi_resp_t debug_resp;

logic dmactive;

// IRQ
logic [1:0] irq;
assign test_en    = 1'b0;

logic [NBSlave-1:0] pc_asserted;

rstgen i_rstgen_main (
    .clk_i        ( clk                      ),
    .rst_ni       ( pll_locked & (~ndmreset) ),
    .test_mode_i  ( test_en                  ),
    .rst_no       ( ndmreset_n               ),
    .init_no      (                          ) // keep open
);

assign rst_n = ~ddr_sync_reset;
assign rst = ddr_sync_reset;

// ---------------
// AXI Xbar
// ---------------
localparam axi_pkg::xbar_cfg_t XbarCfg = '{
  NoSlvPorts:         NBSlave,                    // # of slave ports, so many masters are connected to the xbar
  NoMstPorts:         ariane_soc::NB_PERIPHERALS, // # of master ports, so many slaves are connected to the xbar
  MaxMstTrans:        8,                          // Maxi # of outstanding transactions per r/w per master
  MaxSlvTrans:        8,                          // Maxi # of outstanding write transactions per slave
  FallThrough:        1'b0,                       // AreAW -> W Fifo's in Fall through mode (1'b0 = long paths)
  LatencyMode:        axi_pkg::CUT_ALL_AX,        // See xbar_latency_t and get_xbarlatmode
  AxiIdWidthSlvPorts: AxiIdWidthMaster,           // Axi Id Width of the Slave Ports
  AxiIdUsedSlvPorts:  AxiIdWidthMaster - 1,       // this many LSB's of the SlvPortAxiId get used in demux
  AxiIdWidthMstPorts: AxiIdWidthSlaves,           // ==> $clog2(NoSLVPorts) + AxiIdWidthSlvPorts !!
  AxiAddrWidth:       AxiAddrWidth,               // Axi Address Width
  AxiDataWidth:       AxiDataWidth,               // Axi Data Width
  NoAddrRules:        ariane_soc::NB_PERIPHERALS + 1  // # of Address Rules in the memory map
};
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
  //'{mst_port_idx: ariane_soc::LlcMon,
  //  start_addr:   ariane_soc::LlcMonBase,
  //  end_addr:     ariane_soc::LlcMonBase   + ariane_soc::LlcMonLength  },
  '{mst_port_idx: ariane_soc::GPIO,
    start_addr:   ariane_soc::GPIOBase,
    end_addr:     ariane_soc::GPIOBase     + ariane_soc::GPIOLength    },
  '{mst_port_idx: ariane_soc::Ethernet,
    start_addr:   ariane_soc::EthernetBase,
    end_addr:     ariane_soc::EthernetBase + ariane_soc::EthernetLength},
  '{mst_port_idx: ariane_soc::SPI,
    start_addr:   ariane_soc::SPIBase,
    end_addr:     ariane_soc::SPIBase      + ariane_soc::SPILength     },
  '{mst_port_idx: ariane_soc::Timer,
    start_addr:   ariane_soc::TimerBase,
    end_addr:     ariane_soc::TimerBase    + ariane_soc::TimerLength   },
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
  .clk_i  ( clk        ),
  .rst_ni ( ndmreset_n ),
  .test_i ( test_en    ),
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


//axi_node_wrap_with_slices #(
//    // three ports from Ariane (instruction, data and bypass)
//    .NB_SLAVE           ( NBSlave                    ),
//    .NB_MASTER          ( ariane_soc::NB_PERIPHERALS ),
//    .NB_REGION          ( ariane_soc::NrRegion       ),
//    .AXI_ADDR_WIDTH     ( AxiAddrWidth               ),
//    .AXI_DATA_WIDTH     ( AxiDataWidth               ),
//    .AXI_USER_WIDTH     ( AxiUserWidth               ),
//    .AXI_ID_WIDTH       ( AxiIdWidthMaster           ),
//    .MASTER_SLICE_DEPTH ( 2                          ),
//    .SLAVE_SLICE_DEPTH  ( 2                          )
//) i_axi_xbar (
//    .clk          ( clk        ),
//    .rst_n        ( ndmreset_n ),
//    .test_en_i    ( test_en    ),
//    .slave        ( slave      ),
//    .master       ( master     ),
//    .start_addr_i ({
//        ariane_soc::DebugBase,
//        ariane_soc::ROMBase,
//        ariane_soc::CLINTBase,
//        ariane_soc::PLICBase,
//        ariane_soc::UARTBase,
//        ariane_soc::SPIBase,
//        ariane_soc::EthernetBase,
//        ariane_soc::GPIOBase,
//        ariane_soc::DRAMBase
//    }),
//    .end_addr_i   ({
//        ariane_soc::DebugBase    + ariane_soc::DebugLength - 1,
//        ariane_soc::ROMBase      + ariane_soc::ROMLength - 1,
//        ariane_soc::CLINTBase    + ariane_soc::CLINTLength - 1,
//        ariane_soc::PLICBase     + ariane_soc::PLICLength - 1,
//        ariane_soc::UARTBase     + ariane_soc::UARTLength - 1,
//        ariane_soc::SPIBase      + ariane_soc::SPILength - 1,
//        ariane_soc::EthernetBase + ariane_soc::EthernetLength -1,
//        ariane_soc::GPIOBase     + ariane_soc::GPIOLength - 1,
//        ariane_soc::DRAMBase     + ariane_soc::DRAMLength - 1
//    }),
//    .valid_rule_i (ariane_soc::ValidRule)
//);

// ---------------
// Debug Module
// ---------------
dmi_jtag i_dmi_jtag (
    .clk_i                ( clk                  ),
    .rst_ni               ( rst_n                ),
    .dmi_rst_no           (                      ), // keep open
    .testmode_i           ( test_en              ),
    .dmi_req_valid_o      ( debug_req_valid      ),
    .dmi_req_ready_i      ( debug_req_ready      ),
    .dmi_req_o            ( debug_req            ),
    .dmi_resp_valid_i     ( debug_resp_valid     ),
    .dmi_resp_ready_o     ( debug_resp_ready     ),
    .dmi_resp_i           ( debug_resp           ),
    .tck_i                ( tck    ),
    .tms_i                ( tms    ),
    .trst_ni              ( trst_n ),
    .td_i                 ( tdi    ),
    .td_o                 ( tdo    ),
    .tdo_oe_o             (        )
);

ariane_axi::req_t    dm_axi_m_req;
ariane_axi::resp_t   dm_axi_m_resp;

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
    .NrHarts          ( 1                 ),
    .BusWidth         ( AxiDataWidth      ),
    .SelectableHarts  ( 1'b1              )
) i_dm_top (
    .clk_i            ( clk               ),
    .rst_ni           ( rst_n             ), // PoR
    .testmode_i       ( test_en           ),
    .ndmreset_o       ( ndmreset          ),
    .dmactive_o       ( dmactive          ), // active debug session
    .debug_req_o      ( debug_req_irq     ),
    .unavailable_i    ( '0                ),
    .hartinfo_i       ( {ariane_pkg::DebugHartInfo} ),
    .slave_req_i      ( dm_slave_req      ),
    .slave_we_i       ( dm_slave_we       ),
    .slave_addr_i     ( dm_slave_addr     ),
    .slave_be_i       ( dm_slave_be       ),
    .slave_wdata_i    ( dm_slave_wdata    ),
    .slave_rdata_o    ( dm_slave_rdata    ),
    .master_req_o     ( dm_master_req     ),
    .master_add_o     ( dm_master_add     ),
    .master_we_o      ( dm_master_we      ),
    .master_wdata_o   ( dm_master_wdata   ),
    .master_be_o      ( dm_master_be      ),
    .master_gnt_i     ( dm_master_gnt     ),
    .master_r_valid_i ( dm_master_r_valid ),
    .master_r_rdata_i ( dm_master_r_rdata ),
    .dmi_rst_ni       ( rst_n             ),
    .dmi_req_valid_i  ( debug_req_valid   ),
    .dmi_req_ready_o  ( debug_req_ready   ),
    .dmi_req_i        ( debug_req         ),
    .dmi_resp_valid_o ( debug_resp_valid  ),
    .dmi_resp_ready_i ( debug_resp_ready  ),
    .dmi_resp_o       ( debug_resp        )
);

AXI_BUS #(
  .AXI_ADDR_WIDTH ( AxiAddrWidth             ),
  .AXI_DATA_WIDTH ( AxiDataWidth             ),
  .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
  .AXI_USER_WIDTH ( AxiUserWidth             )
) dm_bus ();

axi_slave_connect_rev i_dm_connect (
  .axi_req_i ( mst_ports_req [ariane_soc::Debug] ),
  .axi_resp_o( mst_ports_resp[ariane_soc::Debug] ),
  .slave     ( dm_bus )
);

axi2mem #(
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves    ),
    .AXI_ADDR_WIDTH ( AxiAddrWidth        ),
    .AXI_DATA_WIDTH ( AxiDataWidth        ),
    .AXI_USER_WIDTH ( AxiUserWidth        )
) i_dm_axi2mem (
    .clk_i      ( clk                       ),
    .rst_ni     ( rst_n                     ),
    .slave      ( dm_bus                    ), //master[ariane_soc::Debug] ),
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
    .DATA_WIDTH            ( AxiDataWidth              )
) i_dm_axi_master (
    .clk_i                 ( clk                       ),
    .rst_ni                ( rst_n                     ),
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
    .axi_resp_i            ( slv_ports_resp[1]         ) // dm_axi_m_resp             )
);

// ---------------
// Core
// ---------------
//ariane_axi::req_t    axi_ariane_req;
//ariane_axi::resp_t   axi_ariane_resp;

ariane #(
    .ArianeCfg ( ariane_soc::ArianeSocCfg )
) i_ariane (
    .clk_i        ( clk                 ),
    .rst_ni       ( ndmreset_n          ),
    .boot_addr_i  ( ariane_soc::ROMBase ), // start fetching from ROM
    .hart_id_i    ( '0                  ),
    .irq_i        ( irq                 ),
    .ipi_i        ( ipi                 ),
    .time_irq_i   ( timer_irq           ),
    .debug_req_i  ( debug_req_irq       ),
    .axi_req_o    ( slv_ports_req[0]    ), // axi_ariane_req      ),
    .axi_resp_i   ( slv_ports_resp[0]   ) // axi_ariane_resp     )
);

//axi_master_connect i_axi_master_connect_ariane (.axi_req_i(axi_ariane_req), .axi_resp_o(axi_ariane_resp), .master(slave[0]));

// ---------------
// CLINT
// ---------------
// divide clock by two
always_ff @(posedge clk or negedge ndmreset_n) begin
  if (~ndmreset_n) begin
    rtc <= 0;
  end else begin
    rtc <= rtc ^ 1'b1;
  end
end

///ariane_axi::req_t    axi_clint_req;
// ariane_axi::resp_t   axi_clint_resp;

clint #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .NR_CORES       ( 1                )
) i_clint (
    .clk_i       ( clk                               ),
    .rst_ni      ( ndmreset_n                        ),
    .testmode_i  ( test_en                           ),
    .axi_req_i   ( mst_ports_req[ariane_soc::CLINT]  ), // axi_clint_req  ),
    .axi_resp_o  ( mst_ports_resp[ariane_soc::CLINT] ), // axi_clint_resp ),
    .rtc_i       ( rtc                               ),
    .timer_irq_o ( timer_irq                         ),
    .ipi_o       ( ipi                               )
);

// axi_slave_connect i_axi_slave_connect_clint (.axi_req_o(axi_clint_req), .axi_resp_i(axi_clint_resp), .slave(master[ariane_soc::CLINT]));

// ---------------
// ROM
// ---------------
AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) rom_bus();
axi_slave_connect_rev i_rom_connect_rev (
  .axi_req_i  ( mst_ports_req[ariane_soc::ROM]  ),
  .axi_resp_o ( mst_ports_resp[ariane_soc::ROM] ),
  .slave      ( rom_bus                         )
);

axi2mem #(
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) i_axi2rom (
    .clk_i  ( clk                     ),
    .rst_ni ( ndmreset_n              ),
    .slave  ( rom_bus                 ), // master[ariane_soc::ROM] ),
    .req_o  ( rom_req                 ),
    .we_o   (                         ),
    .addr_o ( rom_addr                ),
    .be_o   (                         ),
    .data_o (                         ),
    .data_i ( rom_rdata               )
);

bootrom i_bootrom (
    .clk_i   ( clk       ),
    .req_i   ( rom_req   ),
    .addr_i  ( rom_addr  ),
    .rdata_o ( rom_rdata )
);

// ---------------
// Peripherals
// ---------------
`ifdef KC705
  logic [7:0] unused_led;
  logic [3:0] unused_switches = 4'b0000;
`endif
AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) plic_bus();
axi_slave_connect_rev i_plic_connect_rev (
  .axi_req_i  ( mst_ports_req [ariane_soc::PLIC] ),
  .axi_resp_o ( mst_ports_resp[ariane_soc::PLIC] ),
  .slave      ( plic_bus                         )
);
AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) uart_bus();
axi_slave_connect_rev i_uart_connect_rev (
  .axi_req_i  ( mst_ports_req [ariane_soc::UART] ),
  .axi_resp_o ( mst_ports_resp[ariane_soc::UART] ),
  .slave      ( uart_bus                         )
);
AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) spi_bus();
axi_slave_connect_rev i_spi_connect_rev (
  .axi_req_i  ( mst_ports_req [ariane_soc::SPI] ),
  .axi_resp_o ( mst_ports_resp[ariane_soc::SPI] ),
  .slave      ( spi_bus                         )
);
AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) gpio_bus();
axi_slave_connect_rev i_gpio_connect_rev (
  .axi_req_i  ( mst_ports_req [ariane_soc::GPIO] ),
  .axi_resp_o ( mst_ports_resp[ariane_soc::GPIO] ),
  .slave      ( gpio_bus                         )
);
AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) ethernet_bus();
axi_slave_connect_rev i_ethernet_connect_rev (
  .axi_req_i  ( mst_ports_req [ariane_soc::Ethernet] ),
  .axi_resp_o ( mst_ports_resp[ariane_soc::Ethernet] ),
  .slave      ( ethernet_bus                         )
);
AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) timer_bus();
axi_slave_connect_rev i_timer_connect_rev (
  .axi_req_i  ( mst_ports_req [ariane_soc::Timer] ),
  .axi_resp_o ( mst_ports_resp[ariane_soc::Timer] ),
  .slave      ( timer_bus                         )
);

ariane_peripherals #(
    .AxiAddrWidth ( AxiAddrWidth     ),
    .AxiDataWidth ( AxiDataWidth     ),
    .AxiIdWidth   ( AxiIdWidthSlaves ),
    .AxiUserWidth ( AxiUserWidth     ),
    .InclUART     ( 1'b1             ),
    .InclGPIO     ( 1'b1             ),
    `ifdef KINTEX7
    .InclSPI      ( 1'b1         ),
    .InclEthernet ( 1'b1         )
    `elsif KC705
    .InclSPI      ( 1'b1         ),
    .InclEthernet ( 1'b0         ) // Ethernet requires RAMB16 fpga/src/ariane-ethernet/dualmem_widen8.sv to be defined
    `elsif VC707
    .InclSPI      ( 1'b1         ),
    .InclEthernet ( 1'b0         )
    `elsif VCU118
    .InclSPI      ( 1'b0         ),
    .InclEthernet ( 1'b0         )
    `endif
) i_ariane_peripherals (
    .clk_i        ( clk           ),
    .clk_200MHz_i ( ddr_clock_out ),
    .rst_ni       ( ndmreset_n    ),
    .plic         ( plic_bus      ),
    .uart         ( uart_bus      ),
    .spi          ( spi_bus       ),
    .gpio         ( gpio_bus      ),
    .eth_clk_i    ( eth_clk       ),
    .ethernet     ( ethernet_bus  ),
    .timer        ( timer_bus     ),
    .irq_o        ( irq           ),
    .rx_i         ( rx            ),
    .tx_o         ( tx            ),
    .eth_txck,
    .eth_rxck,
    .eth_rxctl,
    .eth_rxd,
    .eth_rst_n,
    .eth_txctl,
    .eth_txd,
    .eth_mdio,
    .eth_mdc,
    .phy_tx_clk_i   ( phy_tx_clk                  ),
    .sd_clk_i       ( sd_clk_sys                  ),
    .spi_clk_o      ( spi_clk_o                   ),
    .spi_mosi       ( spi_mosi                    ),
    .spi_miso       ( spi_miso                    ),
    .spi_ss         ( spi_ss                      ),
    `ifdef KC705
      .leds_o         ( {led[3:0], unused_led[7:4]}),
      .dip_switches_i ( {sw, unused_switches}     )
    `else
      .leds_o         ( led                       ),
      .dip_switches_i ( sw                        )
    `endif
);


// ---------------------
// Board peripherals
// ---------------------
// ---------------
// DDR
// ---------------
logic [AxiIdWidthSlaves:0] s_axi_awid;
logic [AxiAddrWidth-1:0]     s_axi_awaddr;
logic [7:0]                  s_axi_awlen;
logic [2:0]                  s_axi_awsize;
logic [1:0]                  s_axi_awburst;
logic [0:0]                  s_axi_awlock;
logic [3:0]                  s_axi_awcache;
logic [2:0]                  s_axi_awprot;
logic [3:0]                  s_axi_awregion;
logic [3:0]                  s_axi_awqos;
logic                        s_axi_awvalid;
logic                        s_axi_awready;
logic [AxiDataWidth-1:0]     s_axi_wdata;
logic [AxiDataWidth/8-1:0]   s_axi_wstrb;
logic                        s_axi_wlast;
logic                        s_axi_wvalid;
logic                        s_axi_wready;
logic [AxiIdWidthSlaves:0] s_axi_bid;
logic [1:0]                  s_axi_bresp;
logic                        s_axi_bvalid;
logic                        s_axi_bready;
logic [AxiIdWidthSlaves:0] s_axi_arid;
logic [AxiAddrWidth-1:0]     s_axi_araddr;
logic [7:0]                  s_axi_arlen;
logic [2:0]                  s_axi_arsize;
logic [1:0]                  s_axi_arburst;
logic [0:0]                  s_axi_arlock;
logic [3:0]                  s_axi_arcache;
logic [2:0]                  s_axi_arprot;
logic [3:0]                  s_axi_arregion;
logic [3:0]                  s_axi_arqos;
logic                        s_axi_arvalid;
logic                        s_axi_arready;
logic [AxiIdWidthSlaves:0] s_axi_rid;
logic [AxiDataWidth-1:0]     s_axi_rdata;
logic [1:0]                  s_axi_rresp;
logic                        s_axi_rlast;
logic                        s_axi_rvalid;
logic                        s_axi_rready;

//AXI_BUS #(
//    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
//    .AXI_DATA_WIDTH ( AxiDataWidth     ),
//    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
//    .AXI_USER_WIDTH ( AxiUserWidth     )
//) atomics_bus();
//axi_slave_connect_rev i_atomics_connect_rev (
//  .axi_req_i  ( mst_ports_req [ariane_soc::DRAM] ),
//  .axi_resp_o ( mst_ports_resp[ariane_soc::DRAM] ),
//  .slave      ( atomics_bus                      )
//);


//AXI_BUS #(
//    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
//    .AXI_DATA_WIDTH ( AxiDataWidth     ),
//    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
//    .AXI_USER_WIDTH ( AxiUserWidth     )
//) dram();

ariane_axi::req_slv_t  llc_req;
ariane_axi::req_llc_t  dram_req,  dram_del_req;
ariane_axi::resp_slv_t llc_resp;
ariane_axi::resp_llc_t dram_resp, dram_del_resp;

axi_riscv_atomics #(
    .AXI_ADDR_WIDTH    ( AxiAddrWidth        ),
    .AXI_DATA_WIDTH    ( AxiDataWidth           ),
    .AXI_ID_WIDTH      ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH    ( AxiUserWidth           ),
    .AXI_MAX_WRITE_TXNS( 1                        ),
    .RISCV_WORD_WIDTH  ( 64                       )
  ) i_axi_riscv_atomics (
    .clk_i           ( clk            ),
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

//  assign dram_req = llc_req;
//  assign llc_resp = dram_resp;

//axi_riscv_atomics_wrap #(
//    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
//    .AXI_DATA_WIDTH ( AxiDataWidth     ),
//    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
//    .AXI_USER_WIDTH ( AxiUserWidth     ),
//    .AXI_MAX_WRITE_TXNS ( 1  ),
//    .RISCV_WORD_WIDTH   ( 64 )
//) i_axi_riscv_atomics (
//    .clk_i  ( clk         ),
//    .rst_ni ( ndmreset_n  ),
//    .slv    ( atomics_bus ), // master[ariane_soc::DRAM] ),
//    .mst    ( dram        )
//);

  localparam llc_pkg::llc_axi_cfg_t LlcAxiCfg = '{
    SlvPortIdWidth:    AxiIdWidthSlaves,
    MstPortIdWidth:    AxiIdWidthSlaves + 1,
    AddrWidthFull:     AxiAddrWidth,
    DataWidthFull:     AxiDataWidth,
    LitePortAddrWidth: AxiAddrWidth,
    LitePortDataWidth: AxiDataWidth
  };
  typedef logic [AxiAddrWidth-1:0]   addr_t;
  typedef logic [AxiDataWidth-1:0]   data_lite_t;
  typedef logic [AxiDataWidth/8-1:0] strb_lite_t;

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
//
  req_lite_t  llc_cfg_req;
  resp_lite_t llc_cfg_resp;
//
  AXI_BUS #(
    .AXI_ADDR_WIDTH( AxiAddrWidth     ),
    .AXI_DATA_WIDTH( AxiDataWidth     ),
    .AXI_ID_WIDTH  ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH( AxiUserWidth     )
  ) llc_cfg_64 ();

  //AXI_BUS #(
  //  .AXI_ADDR_WIDTH( AxiAddrWidth     ),
  //  .AXI_DATA_WIDTH( 32               ),
  //  .AXI_ID_WIDTH  ( AxiIdWidthSlaves ),
  //  .AXI_USER_WIDTH( AxiUserWidth     )
  //) llc_cfg_32 ();

  AXI_LITE #(
    .AXI_ADDR_WIDTH (AxiAddrWidth),
    .AXI_DATA_WIDTH (AxiDataWidth)
  ) llc_lite_cfg ();

  axi_slave_connect_rev i_llc_cfg_connect_rev (
    .axi_req_i  ( mst_ports_req [ariane_soc::LlcCfg] ),
    .axi_resp_o ( mst_ports_resp[ariane_soc::LlcCfg] ),
    .slave      ( llc_cfg_64                         )
  );

  //axi_data_width_converter #(
  //  .ADDR_WIDTH     ( AxiAddrWidth     ),
  //  .SI_DATA_WIDTH  ( AxiDataWidth     ),
  //  .MI_DATA_WIDTH  ( 32               ),
  //  .ID_WIDTH       ( AxiIdWidthSlaves ),
  //  .USER_WIDTH     ( AxiUserWidth     ) ,
  //  .NR_OUTSTANDING ( 1                )
  //) i_llc_cfg_data_width_convert (
  //  .clk_i ( clk        ),
  //  .rst_ni( ndmreset_n ),
  //  .slv   ( llc_cfg_64 ),
  //  .mst   ( llc_cfg_32 )
  //);

  axi_to_axi_lite #(
  /// Maximum number of outstanding reads.
    .NUM_PENDING_RD (1),
  /// Maximum number of outstanding writes.
    .NUM_PENDING_WR (1)
  ) i_llc_cfg_axi_convert (
    .clk_i      ( clk          ),
    .rst_ni     ( ndmreset_n   ),
    .testmode_i ( test_en      ),
    .in         ( llc_cfg_64   ),
    .out        ( llc_lite_cfg )
  );

  assign llc_cfg_req.aw.addr   = llc_lite_cfg.aw_addr;
  assign llc_cfg_req.aw_valid  = llc_lite_cfg.aw_valid;
  assign llc_lite_cfg.aw_ready = llc_cfg_resp.aw_ready;

  assign llc_cfg_req.w.data    = llc_lite_cfg.w_data;
  assign llc_cfg_req.w.strb    = llc_lite_cfg.w_strb;
  assign llc_cfg_req.w_valid   = llc_lite_cfg.w_valid;
  assign llc_lite_cfg.w_ready  = llc_cfg_resp.w_ready;

  assign llc_lite_cfg.b_resp   = llc_cfg_resp.b.resp;
  assign llc_lite_cfg.b_valid  = llc_cfg_resp.b_valid;
  assign llc_cfg_req.b_ready   = llc_lite_cfg.b_ready;

  assign llc_cfg_req.ar.addr   = llc_lite_cfg.ar_addr;
  assign llc_cfg_req.ar_valid  = llc_lite_cfg.ar_valid;
  assign llc_lite_cfg.ar_ready = llc_cfg_resp.ar_ready;

  assign llc_lite_cfg.r_data   = llc_cfg_resp.r.data;
  assign llc_lite_cfg.r_resp   = llc_cfg_resp.r.resp;
  assign llc_lite_cfg.r_valid  = llc_cfg_resp.r_valid;
  assign llc_cfg_req.r_ready   = llc_lite_cfg.r_ready;
  // assign mst_ports_resp[ariane_soc::LlcCfg] = '0;

  llc #(
    .SetAssociativity( 8                         ),
    .NoLines         ( 1024                      ),
    .NoBlocks        ( 8                         ),
    .AxiCfg          ( LlcAxiCfg                 ),
    .slv_aw_chan_t   ( ariane_axi::aw_chan_slv_t ),
    .mst_aw_chan_t   ( ariane_axi::aw_chan_llc_t ),
    .w_chan_t        ( ariane_axi::w_chan_t      ),
    .slv_b_chan_t    ( ariane_axi::b_chan_slv_t  ),
    .mst_b_chan_t    ( ariane_axi::b_chan_llc_t  ),
    .slv_ar_chan_t   ( ariane_axi::ar_chan_slv_t ),
    .mst_ar_chan_t   ( ariane_axi::ar_chan_llc_t ),
    .slv_r_chan_t    ( ariane_axi::r_chan_slv_t  ),
    .mst_r_chan_t    ( ariane_axi::r_chan_llc_t  ),
    .slv_req_t       ( ariane_axi::req_slv_t     ),
    .slv_resp_t      ( ariane_axi::resp_slv_t    ),
    .mst_req_t       ( ariane_axi::req_llc_t     ),
    .mst_resp_t      ( ariane_axi::resp_llc_t    ),
    .lite_aw_chan_t  ( aw_chan_lite_t            ),
    .lite_w_chan_t   ( w_chan_lite_t             ),
    .lite_b_chan_t   ( b_chan_lite_t             ),
    .lite_ar_chan_t  ( ar_chan_lite_t            ),
    .lite_r_chan_t   ( r_chan_lite_t             ),
    .lite_req_t      ( req_lite_t                ),
    .lite_resp_t     ( resp_lite_t               ),
    .rule_full_t     ( axi_pkg::xbar_rule_64_t   ),
    .rule_lite_t     ( axi_pkg::xbar_rule_64_t   )
  ) i_llc (
    .clk_i       ( clk            ),
    .rst_ni      ( ndmreset_n     ),
    .test_i      ( test_en        ),
    .slv_req_i   ( llc_req        ),
    .slv_resp_o  ( llc_resp       ),
    .mst_req_o   ( dram_del_req   ),
    .mst_resp_i  ( dram_del_resp  ),
    .conf_req_i  ( llc_cfg_req    ),
    .conf_resp_o ( llc_cfg_resp   ),
    .ram_start_addr_i ( ariane_soc::DRAMBase                            ),
    .ram_end_addr_i   ( ariane_soc::DRAMBase   + ariane_soc::DRAMLength ),
    .spm_start_addr_i ( ariane_soc::LlcSpmBase                          ),
    .cfg_start_addr_i ( ariane_soc::LlcCfgBase                          )
  );

  // axi delayer for test
//axi_delayer #(
//  .aw_t ( ariane_axi::aw_chan_llc_t ),
//  .w_t  ( ariane_axi::w_chan_t      ),
//  .b_t  ( ariane_axi::b_chan_llc_t  ),
//  .ar_t ( ariane_axi::ar_chan_llc_t ),
//  .r_t  ( ariane_axi::r_chan_llc_t  ),
//  .StallRandomOutput (  1'b0   ),
//  .StallRandomInput  (  1'b0   ),
//  .FixedDelayInput   ( 32'd63  ),
//  .FixedDelayOutput  ( 32'd63  )
//) i_dram_delay (
//  .clk_i  ( clk        ),   // Clock
//  .rst_ni ( ndmreset_n ),   // Asynchronous reset active low
//  // input side
//  .aw_valid_i ( dram_del_req.aw_valid  ),
//  .aw_chan_i  ( dram_del_req.aw        ),
//  .aw_ready_o ( dram_del_resp.aw_ready ),

//  .w_valid_i  ( dram_del_req.w_valid   ),
//  .w_chan_i   ( dram_del_req.w         ),
//  .w_ready_o  ( dram_del_resp.w_ready  ),

//  .b_valid_o  ( dram_del_resp.b_valid  ),
//  .b_chan_o   ( dram_del_resp.b        ),
//  .b_ready_i  ( dram_del_req.b_ready   ),

//  .ar_valid_i ( dram_del_req.ar_valid  ),
//  .ar_chan_i  ( dram_del_req.ar        ),
//  .ar_ready_o ( dram_del_resp.ar_ready ),

//  .r_valid_o  ( dram_del_resp.r_valid  ),
//  .r_chan_o   ( dram_del_resp.r        ),
//  .r_ready_i  ( dram_del_req.r_ready   ),

//  // output side
//  .aw_valid_o ( dram_req.aw_valid  ),
//  .aw_chan_o  ( dram_req.aw        ),
//  .aw_ready_i ( dram_resp.aw_ready ),

//  .w_valid_o  ( dram_req.w_valid   ),
//  .w_chan_o   ( dram_req.w         ),
//  .w_ready_i  ( dram_resp.w_ready  ),

//  .b_valid_i  ( dram_resp.b_valid  ),
//  .b_chan_i   ( dram_resp.b        ),
//  .b_ready_o  ( dram_req.b_ready   ),

//  .ar_valid_o ( dram_req.ar_valid  ),
//  .ar_chan_o  ( dram_req.ar        ),
//  .ar_ready_i ( dram_resp.ar_ready ),

//  .r_valid_i  ( dram_resp.r_valid  ),
//  .r_chan_i   ( dram_resp.r        ),
//  .r_ready_o  ( dram_req.r_ready   )
//);

  AXI_BUS #(
    .AXI_ADDR_WIDTH( AxiAddrWidth         ),
    .AXI_DATA_WIDTH( AxiDataWidth         ),
    .AXI_ID_WIDTH  ( AxiIdWidthSlaves + 1 ),
    .AXI_USER_WIDTH( AxiUserWidth         )
  ) llc_2_delay ();

  AXI_BUS #(
    .AXI_ADDR_WIDTH( AxiAddrWidth         ),
    .AXI_DATA_WIDTH( AxiDataWidth         ),
    .AXI_ID_WIDTH  ( AxiIdWidthSlaves + 1 ),
    .AXI_USER_WIDTH( AxiUserWidth         )
  ) delay_2_dram ();

  axi_multicut #(
  /// The address width.
    .ADDR_WIDTH ( AxiAddrWidth         ),
  /// The data width.
    .DATA_WIDTH ( AxiDataWidth         ),
  /// The ID width.
    .ID_WIDTH   ( AxiIdWidthSlaves + 1 ),
  // The user data width.
    .USER_WIDTH ( AxiUserWidth         ),
  // The number of cuts. Must be >= 0.
    .NUM_CUTS   ( 32'd0               )
  ) i_dram_delay (
    .clk_i ( clk          ),
    .rst_ni( ndmreset_n   ),
    .in    ( llc_2_delay  ),
    .out   ( delay_2_dram )
  );

  assign llc_2_delay.aw_id      = dram_del_req.aw.id    ;
  assign llc_2_delay.aw_addr    = dram_del_req.aw.addr  ;
  assign llc_2_delay.aw_len     = dram_del_req.aw.len   ;
  assign llc_2_delay.aw_size    = dram_del_req.aw.size  ;
  assign llc_2_delay.aw_burst   = dram_del_req.aw.burst ;
  assign llc_2_delay.aw_lock    = dram_del_req.aw.lock  ;
  assign llc_2_delay.aw_cache   = dram_del_req.aw.cache ;
  assign llc_2_delay.aw_prot    = dram_del_req.aw.prot  ;
  assign llc_2_delay.aw_qos     = dram_del_req.aw.qos   ;
  assign llc_2_delay.aw_region  = dram_del_req.aw.region;
  assign llc_2_delay.aw_atop    = dram_del_req.aw.atop  ;
  assign llc_2_delay.aw_user    = '0                    ;
  assign llc_2_delay.aw_valid   = dram_del_req.aw_valid ;
  assign dram_del_resp.aw_ready = llc_2_delay.aw_ready  ;

  assign llc_2_delay.w_data     = dram_del_req.w.data   ;
  assign llc_2_delay.w_strb     = dram_del_req.w.strb   ;
  assign llc_2_delay.w_last     = dram_del_req.w.last   ;
  assign llc_2_delay.w_user     = '0                    ;
  assign llc_2_delay.w_valid    = dram_del_req.w_valid  ;
  assign dram_del_resp.w_ready  = llc_2_delay.w_ready   ;

  assign dram_del_resp.b.id     = llc_2_delay.b_id      ;
  assign dram_del_resp.b.resp   = llc_2_delay.b_resp    ;
  assign dram_del_resp.b_valid  = llc_2_delay.b_valid   ;
  assign llc_2_delay.b_ready    = dram_del_req.b_ready  ;

  assign llc_2_delay.ar_id      = dram_del_req.ar.id    ;
  assign llc_2_delay.ar_addr    = dram_del_req.ar.addr  ;
  assign llc_2_delay.ar_len     = dram_del_req.ar.len   ;
  assign llc_2_delay.ar_size    = dram_del_req.ar.size  ;
  assign llc_2_delay.ar_burst   = dram_del_req.ar.burst ;
  assign llc_2_delay.ar_lock    = dram_del_req.ar.lock  ;
  assign llc_2_delay.ar_cache   = dram_del_req.ar.cache ;
  assign llc_2_delay.ar_prot    = dram_del_req.ar.prot  ;
  assign llc_2_delay.ar_qos     = dram_del_req.ar.qos   ;
  assign llc_2_delay.ar_region  = dram_del_req.ar.region;
  assign llc_2_delay.ar_user    = '0                    ;
  assign llc_2_delay.ar_valid   = dram_del_req.ar_valid ;
  assign dram_del_resp.ar_ready = llc_2_delay.ar_ready  ;

  assign dram_del_resp.r.id     = llc_2_delay.r_id      ;
  assign dram_del_resp.r.data   = llc_2_delay.r_data    ;
  assign dram_del_resp.r.resp   = llc_2_delay.r_resp    ;
  assign dram_del_resp.r.last   = llc_2_delay.r_last    ;
  assign dram_del_resp.r_valid  = llc_2_delay.r_valid   ;
  assign llc_2_delay.r_ready    = dram_del_req.r_ready  ;


  assign dram_req.aw.id         = delay_2_dram.aw_id     ;
  assign dram_req.aw.addr       = delay_2_dram.aw_addr   ;
  assign dram_req.aw.len        = delay_2_dram.aw_len    ;
  assign dram_req.aw.size       = delay_2_dram.aw_size   ;
  assign dram_req.aw.burst      = delay_2_dram.aw_burst  ;
  assign dram_req.aw.lock       = delay_2_dram.aw_lock   ;
  assign dram_req.aw.cache      = delay_2_dram.aw_cache  ;
  assign dram_req.aw.prot       = delay_2_dram.aw_prot   ;
  assign dram_req.aw.qos        = delay_2_dram.aw_qos    ;
  assign dram_req.aw.region     = delay_2_dram.aw_region ;
  assign dram_req.aw.atop       = delay_2_dram.aw_atop   ;
  assign dram_req.aw_valid      = delay_2_dram.aw_valid  ;
  assign delay_2_dram.aw_ready  = dram_resp.aw_ready     ;

  assign dram_req.w.data        = delay_2_dram.w_data    ;
  assign dram_req.w.strb        = delay_2_dram.w_strb    ;
  assign dram_req.w.last        = delay_2_dram.w_last    ;
  assign dram_req.w_valid       = delay_2_dram.w_valid   ;
  assign delay_2_dram.w_ready   = dram_resp.w_ready      ;

  assign delay_2_dram.b_id      = dram_resp.b.id         ;
  assign delay_2_dram.b_resp    = dram_resp.b.resp       ;
  assign delay_2_dram.b_user    = '0                     ;
  assign delay_2_dram.b_valid   = dram_resp.b_valid      ;
  assign dram_req.b_ready       = delay_2_dram.b_ready   ;

  assign dram_req.ar.id         = delay_2_dram.ar_id     ;
  assign dram_req.ar.addr       = delay_2_dram.ar_addr   ;
  assign dram_req.ar.len        = delay_2_dram.ar_len    ;
  assign dram_req.ar.size       = delay_2_dram.ar_size   ;
  assign dram_req.ar.burst      = delay_2_dram.ar_burst  ;
  assign dram_req.ar.lock       = delay_2_dram.ar_lock   ;
  assign dram_req.ar.cache      = delay_2_dram.ar_cache  ;
  assign dram_req.ar.prot       = delay_2_dram.ar_prot   ;
  assign dram_req.ar.qos        = delay_2_dram.ar_qos    ;
  assign dram_req.ar.region     = delay_2_dram.ar_region ;
  assign dram_req.ar_valid      = delay_2_dram.ar_valid  ;
  assign delay_2_dram.ar_ready  = dram_resp.ar_ready     ;

  assign delay_2_dram.r_id      = dram_resp.r.id         ;
  assign delay_2_dram.r_data    = dram_resp.r.data       ;
  assign delay_2_dram.r_resp    = dram_resp.r.resp       ;
  assign delay_2_dram.r_last    = dram_resp.r.last       ;
  assign delay_2_dram.r_user    = '0;
  assign delay_2_dram.r_valid   = dram_resp.r_valid      ;
  assign dram_req.r_ready       = delay_2_dram.r_ready   ;


  // performance monitor for the LLC
//  logic        llc_mon_apb_penable;
//  logic        llc_mon_apb_pwrite;
//  logic [31:0] llc_mon_apb_paddr;
//  logic        llc_mon_apb_psel;
//  logic [31:0] llc_mon_apb_pwdata;
//  logic [31:0] llc_mon_apb_prdata;
//  logic        llc_mon_apb_pready;
//  logic        llc_mon_apb_pslverr;
//
//  axi2apb_64_32  #(
//      .AXI4_ADDRESS_WIDTH ( AxiAddrWidth     ),
//      .AXI4_RDATA_WIDTH   ( AxiDataWidth     ),
//      .AXI4_WDATA_WIDTH   ( AxiDataWidth     ),
//      .AXI4_ID_WIDTH      ( AxiIdWidthSlaves ),
//      .AXI4_USER_WIDTH    ( AxiUserWidth     ),
//
//      .BUFF_DEPTH_SLAVE   ( 2              ),
//      .APB_ADDR_WIDTH     ( 32'd32         )
//  ) i_llc_axi_perf_mon_axi2apb (
//      .ACLK       ( clk                    ),
//      .ARESETn    ( ndmreset_n             ),
//      .test_en_i  ( test_en                ),
//
//      .AWID_i     ( mst_ports_req[ariane_soc::LlcMon].aw.id        ),
//      .AWADDR_i   ( mst_ports_req[ariane_soc::LlcMon].aw.addr      ),
//      .AWLEN_i    ( mst_ports_req[ariane_soc::LlcMon].aw.len       ),
//      .AWSIZE_i   ( mst_ports_req[ariane_soc::LlcMon].aw.size      ),
//      .AWBURST_i  ( mst_ports_req[ariane_soc::LlcMon].aw.burst     ),
//      .AWLOCK_i   ( mst_ports_req[ariane_soc::LlcMon].aw.lock      ),
//      .AWCACHE_i  ( mst_ports_req[ariane_soc::LlcMon].aw.cache     ),
//      .AWPROT_i   ( mst_ports_req[ariane_soc::LlcMon].aw.prot      ),
//      .AWREGION_i ( mst_ports_req[ariane_soc::LlcMon].aw.region    ),
//      .AWUSER_i   ( '0                                             ),
//      .AWQOS_i    ( mst_ports_req[ariane_soc::LlcMon].aw.qos       ),
//      .AWVALID_i  ( mst_ports_req[ariane_soc::LlcMon].aw_valid     ),
//      .AWREADY_o  ( mst_ports_resp[ariane_soc::LlcMon].aw_ready    ),
//
//      .WDATA_i    ( mst_ports_req[ariane_soc::LlcMon].w.data       ),
//      .WSTRB_i    ( mst_ports_req[ariane_soc::LlcMon].w.strb       ),
//      .WLAST_i    ( mst_ports_req[ariane_soc::LlcMon].w.last       ),
//      .WUSER_i    ( '0                                             ),
//      .WVALID_i   ( mst_ports_req[ariane_soc::LlcMon].w_valid      ),
//      .WREADY_o   ( mst_ports_resp[ariane_soc::LlcMon].w_ready     ),
//
//      .BID_o      ( mst_ports_resp[ariane_soc::LlcMon].b.id        ),
//      .BRESP_o    ( mst_ports_resp[ariane_soc::LlcMon].b.resp      ),
//      .BVALID_o   ( mst_ports_resp[ariane_soc::LlcMon].b_valid     ),
//      .BUSER_o    ( /* not used */                                 ),
//      .BREADY_i   ( mst_ports_req[ariane_soc::LlcMon].b_ready      ),
//
//      .ARID_i     ( mst_ports_req[ariane_soc::LlcMon].ar.id        ),
//      .ARADDR_i   ( mst_ports_req[ariane_soc::LlcMon].ar.addr      ),
//      .ARLEN_i    ( mst_ports_req[ariane_soc::LlcMon].ar.len       ),
//      .ARSIZE_i   ( mst_ports_req[ariane_soc::LlcMon].ar.size      ),
//      .ARBURST_i  ( mst_ports_req[ariane_soc::LlcMon].ar.burst     ),
//      .ARLOCK_i   ( mst_ports_req[ariane_soc::LlcMon].ar.lock      ),
//      .ARCACHE_i  ( mst_ports_req[ariane_soc::LlcMon].ar.cache     ),
//      .ARPROT_i   ( mst_ports_req[ariane_soc::LlcMon].ar.prot      ),
//      .ARREGION_i ( mst_ports_req[ariane_soc::LlcMon].ar.region    ),
//      .ARUSER_i   ( '0                                             ),
//      .ARQOS_i    ( mst_ports_req[ariane_soc::LlcMon].ar.qos       ),
//      .ARVALID_i  ( mst_ports_req[ariane_soc::LlcMon].ar_valid     ),
//      .ARREADY_o  ( mst_ports_resp[ariane_soc::LlcMon].ar_ready    ),
//
//      .RID_o      ( mst_ports_resp[ariane_soc::LlcMon].r.id        ),
//      .RDATA_o    ( mst_ports_resp[ariane_soc::LlcMon].r.data      ),
//      .RRESP_o    ( mst_ports_resp[ariane_soc::LlcMon].r.resp      ),
//      .RLAST_o    ( mst_ports_resp[ariane_soc::LlcMon].r.last      ),
//      .RUSER_o    ( /* not used */                                 ),
//      .RVALID_o   ( mst_ports_resp[ariane_soc::LlcMon].r_valid     ),
//      .RREADY_i   ( mst_ports_req[ariane_soc::LlcMon].r_ready      ),
//
//      .PENABLE    ( llc_mon_apb_penable    ),
//      .PWRITE     ( llc_mon_apb_pwrite     ),
//      .PADDR      ( llc_mon_apb_paddr      ),
//      .PSEL       ( llc_mon_apb_psel       ),
//      .PWDATA     ( llc_mon_apb_pwdata     ),
//      .PRDATA     ( llc_mon_apb_prdata     ),
//      .PREADY     ( llc_mon_apb_pready     ),
//      .PSLVERR    ( llc_mon_apb_pslverr    )
//  );
//
//  axi_perf_mon #(
//     // Number of monitored AXI interfaces
//    .N_MON  ( 32'd2                 ),
//     // ID width of the monitored AXI interfaces
//    .IW     ( LlcAxiCfg.MstPortIdWidth ), // extend the slv port!
//    // Capabilities of all interface monitors
//    .CAP_HS        ( 1'b1   ),   // handshakes
//    .CAP_FL_TXN    ( 1'b1   ),   // transactions in flight
//    .CAP_FL_DAT    ( 1'b1   ),   // data bytes in flight
//    .CAP_TX_DAT    ( 1'b1   ),   // data transferred
//    .CAP_STALL     ( 1'b1   ),   // stalls
//    .CAP_RT        ( 1'b1   ),   // round trips
//    .CAP_EXCL      ( 1'b0   ),   // exclusive accesses
//    .CAP_ATOP      ( 1'b0   ),   // atomic transactions
//    // Counter widths for all interface monitors
//    .CW_CLK        ( 32'd42 ),
//    .CW_HS_CMD     ( 32'd42 ),
//    .CW_HS_DAT     ( 32'd42 ),
//    .CW_FL_TXN_ACC ( 32'd42 ),
//    .CW_FL_TXN_MAX ( 32'd42 ),
//    .CW_FL_DAT_ACC ( 32'd42 ),
//    .CW_FL_DAT_MAX ( 32'd42 ),
//    .CW_TX_DAT     ( 32'd42 ),
//    .CW_STALL_CMD  ( 32'd42 ),
//    .CW_STALL_DAT  ( 32'd42 ),
//    .CW_STALL_MAX  ( 32'd42 ),
//    .CW_RT_ACC     ( 32'd42 ),
//    .CW_RT_MAX     ( 32'd42 ),
//    .CW_EXCL       ( 32'd42 ),
//    .CW_ATOP       ( 32'd42 ),
//    // Protocol compliance assertions
//    .ASSERTIONS    ( 1'b1 )
//  ) i_llc_axi_perf_mon (
//    // APB Readout and Control Interface
//    .pclk_i    ( clk                 ),
//    .preset_ni ( ndmreset_n          ),
//    .paddr_i   ( llc_mon_apb_paddr   ), // 32 bit
//    .pprot_i   ( '0                  ), //  3 bit
//    .psel_i    ( llc_mon_apb_psel    ),
//    .penable_i ( llc_mon_apb_penable ),
//    .pwrite_i  ( llc_mon_apb_pwrite  ),
//    .pwdata_i  ( llc_mon_apb_pwdata  ), // 32 bit
//    .pstrb_i   ( '1                  ), //  4 bit
//    .pready_o  ( llc_mon_apb_pready  ),
//    .prdata_o  ( llc_mon_apb_prdata  ), // 32 bit
//    .pslverr_o ( llc_mon_apb_pslverr ),
//    // Monitored AXI Interfaces
//    .clk_axi_i  ({       clk_i,             clk_i              }),
//    .rst_axi_ni ({       rst_ni,            rst_ni             }),
//    .ar_id_i    ({{1'b0, llc_req.ar.id},    dram_req.ar.id     }),
//    .ar_len_i   ({       llc_req.ar.len,    dram_req.ar.len    }),
//    .ar_size_i  ({       llc_req.ar.size,   dram_req.ar.size   }),
//    .ar_lock_i  ({       llc_req.ar.lock,   dram_req.ar.lock   }),
//    .ar_valid_i ({       llc_req.ar_valid,  dram_req.ar_valid  }),
//    .ar_ready_i ({       llc_resp.ar_ready, dram_resp.ar_ready }),
//    .aw_id_i    ({{1'b0, llc_req.aw.id},    dram_req.aw.id     }),
//    .aw_len_i   ({       llc_req.aw.len,    dram_req.aw.len    }),
//    .aw_size_i  ({       llc_req.aw.size,   dram_req.aw.size   }),
//    .aw_lock_i  ({       llc_req.aw.lock,   dram_req.aw.lock   }),
//    .aw_atop_i  ( '0                                            ),
//    .aw_valid_i ({       llc_req.aw_valid,  dram_req.aw_valid  }),
//    .aw_ready_i ({       llc_resp.aw_ready, dram_resp.aw_ready }),
//    .r_id_i     ({{1'b0, llc_resp.r.id},    dram_resp.r.id     }),
//    .r_last_i   ({       llc_resp.r.last,   dram_resp.r.last   }),
//    .r_valid_i  ({       llc_resp.r_valid,  dram_resp.r_valid  }),
//    .r_ready_i  ({       llc_req.r_ready,   dram_req.r_ready   }),
//    .w_last_i   ({       llc_req.w.last,    dram_req.w.last    }),
//    .w_valid_i  ({       llc_req.w_valid,   dram_req.w_valid   }),
//    .w_ready_i  ({       llc_resp.w_ready,  dram_resp.w_ready  }),
//    .b_id_i     ({{1'b0, llc_resp.b.id},    dram_resp.b.id     }),
//    .b_resp_i   ({       llc_resp.b.resp,   dram_resp.b.resp   }),
//    .b_valid_i  ({       llc_resp.b_valid,  dram_resp.b_valid  }),
//    .b_ready_i  ({       llc_req.b_ready,   dram_req.b_ready   })
//  );

`ifdef PROTOCOL_CHECKER
logic pc_status;

xlnx_protocol_checker i_xlnx_protocol_checker (
  .pc_status(),
  .pc_asserted(pc_status),
  .aclk(clk),
  .aresetn(ndmreset_n),
  .pc_axi_awid     (dram_req.aw.id),
  .pc_axi_awaddr   (dram_req.aw.addr),
  .pc_axi_awlen    (dram_req.aw.len),
  .pc_axi_awsize   (dram_req.aw.size),
  .pc_axi_awburst  (dram_req.aw.burst),
  .pc_axi_awlock   (dram_req.aw.lock),
  .pc_axi_awcache  (dram_req.aw.cache),
  .pc_axi_awprot   (dram_req.aw.prot),
  .pc_axi_awqos    (dram_req.aw.qos),
  .pc_axi_awregion (dram_req.aw.region),
  .pc_axi_awuser   ( '0), //dram_req.aw.user),
  .pc_axi_awvalid  (dram_req.aw_valid),
  .pc_axi_awready  (dram_resp.aw_ready),
  .pc_axi_wlast    (dram_req.w.last),
  .pc_axi_wdata    (dram_req.w.data),
  .pc_axi_wstrb    (dram_req.w.strb),
  .pc_axi_wuser    ( '0), //dram_req.w.user),
  .pc_axi_wvalid   (dram_req.w_valid),
  .pc_axi_wready   (dram_resp.w_ready),
  .pc_axi_bid      (dram_resp.b.id),
  .pc_axi_bresp    (dram_resp.b.resp),
  .pc_axi_buser    ('0), //dram.b_user),
  .pc_axi_bvalid   (dram_resp.b_valid),
  .pc_axi_bready   (dram_req.b_ready),
  .pc_axi_arid     (dram_req.ar.id),
  .pc_axi_araddr   (dram_req.ar.addr),
  .pc_axi_arlen    (dram_req.ar.len),
  .pc_axi_arsize   (dram_req.ar.size),
  .pc_axi_arburst  (dram_req.ar.burst),
  .pc_axi_arlock   (dram_req.ar.lock),
  .pc_axi_arcache  (dram_req.ar.cache),
  .pc_axi_arprot   (dram_req.ar.prot),
  .pc_axi_arqos    (dram_req.ar.qos),
  .pc_axi_arregion (dram_req.ar.region),
  .pc_axi_aruser   ('0),//dram_req.ar.user),
  .pc_axi_arvalid  (dram_req.ar_valid),
  .pc_axi_arready  (dram_resp.ar_ready),
  .pc_axi_rid      (dram_resp.r.id),
  .pc_axi_rlast    (dram_resp.r.last),
  .pc_axi_rdata    (dram_resp.r.data),
  .pc_axi_rresp    (dram_resp.r.resp),
  .pc_axi_ruser    ('0), //dram_resp.r.user),
  .pc_axi_rvalid   (dram_resp.r_valid),
  .pc_axi_rready   (dram_req.r_ready)
);
`endif

// assign dram.r_user = '0;
// assign dram.b_user = '0;

xlnx_axi_clock_converter i_xlnx_axi_clock_converter_ddr (
  .s_axi_aclk     ( clk              ),
  .s_axi_aresetn  ( ndmreset_n       ),
  .s_axi_awid     ( dram_req.aw.id       ),
  .s_axi_awaddr   ( dram_req.aw.addr     ),
  .s_axi_awlen    ( dram_req.aw.len      ),
  .s_axi_awsize   ( dram_req.aw.size     ),
  .s_axi_awburst  ( dram_req.aw.burst    ),
  .s_axi_awlock   ( dram_req.aw.lock     ),
  .s_axi_awcache  ( dram_req.aw.cache    ),
  .s_axi_awprot   ( dram_req.aw.prot     ),
  .s_axi_awregion ( dram_req.aw.region   ),
  .s_axi_awqos    ( dram_req.aw.qos      ),
  .s_axi_awvalid  ( dram_req.aw_valid    ),
  .s_axi_awready  ( dram_resp.aw_ready    ),
  .s_axi_wdata    ( dram_req.w.data      ),
  .s_axi_wstrb    ( dram_req.w.strb      ),
  .s_axi_wlast    ( dram_req.w.last      ),
  .s_axi_wvalid   ( dram_req.w_valid     ),
  .s_axi_wready   ( dram_resp.w_ready     ),
  .s_axi_bid      ( dram_resp.b.id        ),
  .s_axi_bresp    ( dram_resp.b.resp      ),
  .s_axi_bvalid   ( dram_resp.b_valid     ),
  .s_axi_bready   ( dram_req.b_ready     ),
  .s_axi_arid     ( dram_req.ar.id       ),
  .s_axi_araddr   ( dram_req.ar.addr     ),
  .s_axi_arlen    ( dram_req.ar.len      ),
  .s_axi_arsize   ( dram_req.ar.size     ),
  .s_axi_arburst  ( dram_req.ar.burst    ),
  .s_axi_arlock   ( dram_req.ar.lock     ),
  .s_axi_arcache  ( dram_req.ar.cache    ),
  .s_axi_arprot   ( dram_req.ar.prot     ),
  .s_axi_arregion ( dram_req.ar.region   ),
  .s_axi_arqos    ( dram_req.ar.qos      ),
  .s_axi_arvalid  ( dram_req.ar_valid    ),
  .s_axi_arready  ( dram_resp.ar_ready    ),
  .s_axi_rid      ( dram_resp.r.id        ),
  .s_axi_rdata    ( dram_resp.r.data      ),
  .s_axi_rresp    ( dram_resp.r.resp      ),
  .s_axi_rlast    ( dram_resp.r.last      ),
  .s_axi_rvalid   ( dram_resp.r_valid     ),
  .s_axi_rready   ( dram_req.r_ready     ),
  // to size converter
  .m_axi_aclk     ( ddr_clock_out    ),
  .m_axi_aresetn  ( ndmreset_n       ),
  .m_axi_awid     ( s_axi_awid       ),
  .m_axi_awaddr   ( s_axi_awaddr     ),
  .m_axi_awlen    ( s_axi_awlen      ),
  .m_axi_awsize   ( s_axi_awsize     ),
  .m_axi_awburst  ( s_axi_awburst    ),
  .m_axi_awlock   ( s_axi_awlock     ),
  .m_axi_awcache  ( s_axi_awcache    ),
  .m_axi_awprot   ( s_axi_awprot     ),
  .m_axi_awregion ( s_axi_awregion   ),
  .m_axi_awqos    ( s_axi_awqos      ),
  .m_axi_awvalid  ( s_axi_awvalid    ),
  .m_axi_awready  ( s_axi_awready    ),
  .m_axi_wdata    ( s_axi_wdata      ),
  .m_axi_wstrb    ( s_axi_wstrb      ),
  .m_axi_wlast    ( s_axi_wlast      ),
  .m_axi_wvalid   ( s_axi_wvalid     ),
  .m_axi_wready   ( s_axi_wready     ),
  .m_axi_bid      ( s_axi_bid        ),
  .m_axi_bresp    ( s_axi_bresp      ),
  .m_axi_bvalid   ( s_axi_bvalid     ),
  .m_axi_bready   ( s_axi_bready     ),
  .m_axi_arid     ( s_axi_arid       ),
  .m_axi_araddr   ( s_axi_araddr     ),
  .m_axi_arlen    ( s_axi_arlen      ),
  .m_axi_arsize   ( s_axi_arsize     ),
  .m_axi_arburst  ( s_axi_arburst    ),
  .m_axi_arlock   ( s_axi_arlock     ),
  .m_axi_arcache  ( s_axi_arcache    ),
  .m_axi_arprot   ( s_axi_arprot     ),
  .m_axi_arregion ( s_axi_arregion   ),
  .m_axi_arqos    ( s_axi_arqos      ),
  .m_axi_arvalid  ( s_axi_arvalid    ),
  .m_axi_arready  ( s_axi_arready    ),
  .m_axi_rid      ( s_axi_rid        ),
  .m_axi_rdata    ( s_axi_rdata      ),
  .m_axi_rresp    ( s_axi_rresp      ),
  .m_axi_rlast    ( s_axi_rlast      ),
  .m_axi_rvalid   ( s_axi_rvalid     ),
  .m_axi_rready   ( s_axi_rready     )
);

xlnx_clk_gen i_xlnx_clk_gen (
  .clk_out1 ( clk           ), // 50 MHz
  .clk_out2 ( phy_tx_clk    ), // 125 MHz (for RGMII PHY)
  .clk_out3 ( eth_clk       ), // 125 MHz quadrature (90 deg phase shift)
  .clk_out4 ( sd_clk_sys    ), // 50 MHz clock
  .reset    ( cpu_reset     ),
  .locked   ( pll_locked    ),
  .clk_in1  ( ddr_clock_out )
);

// `ifdef GENESYSII
fan_ctrl i_fan_ctrl (
    .clk_i         ( clk        ),
    .rst_ni        ( ndmreset_n ),
    .pwm_setting_i ( '1         ),
    .fan_pwm_o     ( fan_pwm    )
);

xlnx_mig_7_ddr3 i_ddr (
    .sys_clk_p,
    .sys_clk_n,
    .ddr3_dq,
    .ddr3_dqs_n,
    .ddr3_dqs_p,
    .ddr3_addr,
    .ddr3_ba,
    .ddr3_ras_n,
    .ddr3_cas_n,
    .ddr3_we_n,
    .ddr3_reset_n,
    .ddr3_ck_p,
    .ddr3_ck_n,
    .ddr3_cke,
    .ddr3_cs_n,
    .ddr3_dm,
    .ddr3_odt,
    .mmcm_locked     (                ), // keep open
    .app_sr_req      ( '0             ),
    .app_ref_req     ( '0             ),
    .app_zq_req      ( '0             ),
    .app_sr_active   (                ), // keep open
    .app_ref_ack     (                ), // keep open
    .app_zq_ack      (                ), // keep open
    .ui_clk          ( ddr_clock_out  ),
    .ui_clk_sync_rst ( ddr_sync_reset ),
    .aresetn         ( ndmreset_n     ),
    .s_axi_awid,
    .s_axi_awaddr    ( s_axi_awaddr[29:0] ),
    .s_axi_awlen,
    .s_axi_awsize,
    .s_axi_awburst,
    .s_axi_awlock,
    .s_axi_awcache,
    .s_axi_awprot,
    .s_axi_awqos,
    .s_axi_awvalid,
    .s_axi_awready,
    .s_axi_wdata,
    .s_axi_wstrb,
    .s_axi_wlast,
    .s_axi_wvalid,
    .s_axi_wready,
    .s_axi_bready,
    .s_axi_bid,
    .s_axi_bresp,
    .s_axi_bvalid,
    .s_axi_arid,
    .s_axi_araddr     ( s_axi_araddr[29:0] ),
    .s_axi_arlen,
    .s_axi_arsize,
    .s_axi_arburst,
    .s_axi_arlock,
    .s_axi_arcache,
    .s_axi_arprot,
    .s_axi_arqos,
    .s_axi_arvalid,
    .s_axi_arready,
    .s_axi_rready,
    .s_axi_rid,
    .s_axi_rdata,
    .s_axi_rresp,
    .s_axi_rlast,
    .s_axi_rvalid,
    .init_calib_complete (            ), // keep open
    .device_temp         (            ), // keep open
    .sys_rst             ( cpu_resetn )
);
//`elsif VCU118
//
//  logic [63:0]  dram_dwidth_axi_awaddr;
//  logic [7:0]   dram_dwidth_axi_awlen;
//  logic [2:0]   dram_dwidth_axi_awsize;
//  logic [1:0]   dram_dwidth_axi_awburst;
//  logic [0:0]   dram_dwidth_axi_awlock;
//  logic [3:0]   dram_dwidth_axi_awcache;
//  logic [2:0]   dram_dwidth_axi_awprot;
//  logic [3:0]   dram_dwidth_axi_awqos;
//  logic         dram_dwidth_axi_awvalid;
//  logic         dram_dwidth_axi_awready;
//  logic [511:0] dram_dwidth_axi_wdata;
//  logic [63:0]  dram_dwidth_axi_wstrb;
//  logic         dram_dwidth_axi_wlast;
//  logic         dram_dwidth_axi_wvalid;
//  logic         dram_dwidth_axi_wready;
//  logic         dram_dwidth_axi_bready;
//  logic [1:0]   dram_dwidth_axi_bresp;
//  logic         dram_dwidth_axi_bvalid;
//  logic [63:0]  dram_dwidth_axi_araddr;
//  logic [7:0]   dram_dwidth_axi_arlen;
//  logic [2:0]   dram_dwidth_axi_arsize;
//  logic [1:0]   dram_dwidth_axi_arburst;
//  logic [0:0]   dram_dwidth_axi_arlock;
//  logic [3:0]   dram_dwidth_axi_arcache;
//  logic [2:0]   dram_dwidth_axi_arprot;
//  logic [3:0]   dram_dwidth_axi_arqos;
//  logic         dram_dwidth_axi_arvalid;
//  logic         dram_dwidth_axi_arready;
//  logic         dram_dwidth_axi_rready;
//  logic         dram_dwidth_axi_rlast;
//  logic         dram_dwidth_axi_rvalid;
//  logic [1:0]   dram_dwidth_axi_rresp;
//  logic [511:0] dram_dwidth_axi_rdata;
//
//axi_dwidth_converter_512_64 i_axi_dwidth_converter_512_64 (
//  .s_axi_aclk     ( ddr_clock_out            ),
//  .s_axi_aresetn  ( ndmreset_n               ),
//
//  .s_axi_awid     ( s_axi_awid               ),
//  .s_axi_awaddr   ( s_axi_awaddr             ),
//  .s_axi_awlen    ( s_axi_awlen              ),
//  .s_axi_awsize   ( s_axi_awsize             ),
//  .s_axi_awburst  ( s_axi_awburst            ),
//  .s_axi_awlock   ( s_axi_awlock             ),
//  .s_axi_awcache  ( s_axi_awcache            ),
//  .s_axi_awprot   ( s_axi_awprot             ),
//  .s_axi_awregion ( '0                       ),
//  .s_axi_awqos    ( s_axi_awqos              ),
//  .s_axi_awvalid  ( s_axi_awvalid            ),
//  .s_axi_awready  ( s_axi_awready            ),
//  .s_axi_wdata    ( s_axi_wdata              ),
//  .s_axi_wstrb    ( s_axi_wstrb              ),
//  .s_axi_wlast    ( s_axi_wlast              ),
//  .s_axi_wvalid   ( s_axi_wvalid             ),
//  .s_axi_wready   ( s_axi_wready             ),
//  .s_axi_bid      ( s_axi_bid                ),
//  .s_axi_bresp    ( s_axi_bresp              ),
//  .s_axi_bvalid   ( s_axi_bvalid             ),
//  .s_axi_bready   ( s_axi_bready             ),
//  .s_axi_arid     ( s_axi_arid               ),
//  .s_axi_araddr   ( s_axi_araddr             ),
//  .s_axi_arlen    ( s_axi_arlen              ),
//  .s_axi_arsize   ( s_axi_arsize             ),
//  .s_axi_arburst  ( s_axi_arburst            ),
//  .s_axi_arlock   ( s_axi_arlock             ),
//  .s_axi_arcache  ( s_axi_arcache            ),
//  .s_axi_arprot   ( s_axi_arprot             ),
//  .s_axi_arregion ( '0                       ),
//  .s_axi_arqos    ( s_axi_arqos              ),
//  .s_axi_arvalid  ( s_axi_arvalid            ),
//  .s_axi_arready  ( s_axi_arready            ),
//  .s_axi_rid      ( s_axi_rid                ),
//  .s_axi_rdata    ( s_axi_rdata              ),
//  .s_axi_rresp    ( s_axi_rresp              ),
//  .s_axi_rlast    ( s_axi_rlast              ),
//  .s_axi_rvalid   ( s_axi_rvalid             ),
//  .s_axi_rready   ( s_axi_rready             ),
//
//  .m_axi_awaddr   ( dram_dwidth_axi_awaddr   ),
//  .m_axi_awlen    ( dram_dwidth_axi_awlen    ),
//  .m_axi_awsize   ( dram_dwidth_axi_awsize   ),
//  .m_axi_awburst  ( dram_dwidth_axi_awburst  ),
//  .m_axi_awlock   ( dram_dwidth_axi_awlock   ),
//  .m_axi_awcache  ( dram_dwidth_axi_awcache  ),
//  .m_axi_awprot   ( dram_dwidth_axi_awprot   ),
//  .m_axi_awregion (                          ), // left open
//  .m_axi_awqos    ( dram_dwidth_axi_awqos    ),
//  .m_axi_awvalid  ( dram_dwidth_axi_awvalid  ),
//  .m_axi_awready  ( dram_dwidth_axi_awready  ),
//  .m_axi_wdata    ( dram_dwidth_axi_wdata    ),
//  .m_axi_wstrb    ( dram_dwidth_axi_wstrb    ),
//  .m_axi_wlast    ( dram_dwidth_axi_wlast    ),
//  .m_axi_wvalid   ( dram_dwidth_axi_wvalid   ),
//  .m_axi_wready   ( dram_dwidth_axi_wready   ),
//  .m_axi_bresp    ( dram_dwidth_axi_bresp    ),
//  .m_axi_bvalid   ( dram_dwidth_axi_bvalid   ),
//  .m_axi_bready   ( dram_dwidth_axi_bready   ),
//  .m_axi_araddr   ( dram_dwidth_axi_araddr   ),
//  .m_axi_arlen    ( dram_dwidth_axi_arlen    ),
//  .m_axi_arsize   ( dram_dwidth_axi_arsize   ),
//  .m_axi_arburst  ( dram_dwidth_axi_arburst  ),
//  .m_axi_arlock   ( dram_dwidth_axi_arlock   ),
//  .m_axi_arcache  ( dram_dwidth_axi_arcache  ),
//  .m_axi_arprot   ( dram_dwidth_axi_arprot   ),
//  .m_axi_arregion (                          ),
//  .m_axi_arqos    ( dram_dwidth_axi_arqos    ),
//  .m_axi_arvalid  ( dram_dwidth_axi_arvalid  ),
//  .m_axi_arready  ( dram_dwidth_axi_arready  ),
//  .m_axi_rdata    ( dram_dwidth_axi_rdata    ),
//  .m_axi_rresp    ( dram_dwidth_axi_rresp    ),
//  .m_axi_rlast    ( dram_dwidth_axi_rlast    ),
//  .m_axi_rvalid   ( dram_dwidth_axi_rvalid   ),
//  .m_axi_rready   ( dram_dwidth_axi_rready   )
//);
//
//  ddr4_0 i_ddr (
//    .c0_init_calib_complete (                              ),
//    .dbg_clk                (                              ),
//    .c0_sys_clk_p           ( c0_sys_clk_p                 ),
//    .c0_sys_clk_n           ( c0_sys_clk_n                 ),
//    .dbg_bus                (                              ),
//    .c0_ddr4_adr            ( c0_ddr4_adr                  ),
//    .c0_ddr4_ba             ( c0_ddr4_ba                   ),
//    .c0_ddr4_cke            ( c0_ddr4_cke                  ),
//    .c0_ddr4_cs_n           ( c0_ddr4_cs_n                 ),
//    .c0_ddr4_dm_dbi_n       ( c0_ddr4_dm_dbi_n             ),
//    .c0_ddr4_dq             ( c0_ddr4_dq                   ),
//    .c0_ddr4_dqs_c          ( c0_ddr4_dqs_c                ),
//    .c0_ddr4_dqs_t          ( c0_ddr4_dqs_t                ),
//    .c0_ddr4_odt            ( c0_ddr4_odt                  ),
//    .c0_ddr4_bg             ( c0_ddr4_bg                   ),
//    .c0_ddr4_reset_n        ( c0_ddr4_reset_n              ),
//    .c0_ddr4_act_n          ( c0_ddr4_act_n                ),
//    .c0_ddr4_ck_c           ( c0_ddr4_ck_c                 ),
//    .c0_ddr4_ck_t           ( c0_ddr4_ck_t                 ),
//    .c0_ddr4_ui_clk         ( ddr_clock_out                ),
//    .c0_ddr4_ui_clk_sync_rst( ddr_sync_reset               ),
//    .c0_ddr4_aresetn        ( ndmreset_n                   ),
//    .c0_ddr4_s_axi_awid     ( '0                           ),
//    .c0_ddr4_s_axi_awaddr   ( dram_dwidth_axi_awaddr[30:0] ),
//    .c0_ddr4_s_axi_awlen    ( dram_dwidth_axi_awlen        ),
//    .c0_ddr4_s_axi_awsize   ( dram_dwidth_axi_awsize       ),
//    .c0_ddr4_s_axi_awburst  ( dram_dwidth_axi_awburst      ),
//    .c0_ddr4_s_axi_awlock   ( dram_dwidth_axi_awlock       ),
//    .c0_ddr4_s_axi_awcache  ( dram_dwidth_axi_awcache      ),
//    .c0_ddr4_s_axi_awprot   ( dram_dwidth_axi_awprot       ),
//    .c0_ddr4_s_axi_awqos    ( dram_dwidth_axi_awqos        ),
//    .c0_ddr4_s_axi_awvalid  ( dram_dwidth_axi_awvalid      ),
//    .c0_ddr4_s_axi_awready  ( dram_dwidth_axi_awready      ),
//    .c0_ddr4_s_axi_wdata    ( dram_dwidth_axi_wdata        ),
//    .c0_ddr4_s_axi_wstrb    ( dram_dwidth_axi_wstrb        ),
//    .c0_ddr4_s_axi_wlast    ( dram_dwidth_axi_wlast        ),
//    .c0_ddr4_s_axi_wvalid   ( dram_dwidth_axi_wvalid       ),
//    .c0_ddr4_s_axi_wready   ( dram_dwidth_axi_wready       ),
//    .c0_ddr4_s_axi_bready   ( dram_dwidth_axi_bready       ),
//    .c0_ddr4_s_axi_bid      (                              ),
//    .c0_ddr4_s_axi_bresp    ( dram_dwidth_axi_bresp        ),
//    .c0_ddr4_s_axi_bvalid   ( dram_dwidth_axi_bvalid       ),
//    .c0_ddr4_s_axi_arid     ( '0                           ),
//    .c0_ddr4_s_axi_araddr   ( dram_dwidth_axi_araddr[30:0] ),
//    .c0_ddr4_s_axi_arlen    ( dram_dwidth_axi_arlen        ),
//    .c0_ddr4_s_axi_arsize   ( dram_dwidth_axi_arsize       ),
//    .c0_ddr4_s_axi_arburst  ( dram_dwidth_axi_arburst      ),
//    .c0_ddr4_s_axi_arlock   ( dram_dwidth_axi_arlock       ),
//    .c0_ddr4_s_axi_arcache  ( dram_dwidth_axi_arcache      ),
//    .c0_ddr4_s_axi_arprot   ( dram_dwidth_axi_arprot       ),
//    .c0_ddr4_s_axi_arqos    ( dram_dwidth_axi_arqos        ),
//    .c0_ddr4_s_axi_arvalid  ( dram_dwidth_axi_arvalid      ),
//    .c0_ddr4_s_axi_arready  ( dram_dwidth_axi_arready      ),
//    .c0_ddr4_s_axi_rready   ( dram_dwidth_axi_rready       ),
//    .c0_ddr4_s_axi_rlast    ( dram_dwidth_axi_rlast        ),
//    .c0_ddr4_s_axi_rvalid   ( dram_dwidth_axi_rvalid       ),
//    .c0_ddr4_s_axi_rresp    ( dram_dwidth_axi_rresp        ),
//    .c0_ddr4_s_axi_rid      (                              ),
//    .c0_ddr4_s_axi_rdata    ( dram_dwidth_axi_rdata        ),
//    .sys_rst                ( cpu_reset                    )
//  );
//
//
//  logic pcie_ref_clk;
//  logic pcie_ref_clk_gt;
//
//  logic pcie_axi_clk;
//  logic pcie_axi_rstn;
//
//  logic         pcie_axi_awready;
//  logic         pcie_axi_wready;
//  logic [3:0]   pcie_axi_bid;
//  logic [1:0]   pcie_axi_bresp;
//  logic         pcie_axi_bvalid;
//  logic         pcie_axi_arready;
//  logic [3:0]   pcie_axi_rid;
//  logic [255:0] pcie_axi_rdata;
//  logic [1:0]   pcie_axi_rresp;
//  logic         pcie_axi_rlast;
//  logic         pcie_axi_rvalid;
//  logic [3:0]   pcie_axi_awid;
//  logic [63:0]  pcie_axi_awaddr;
//  logic [7:0]   pcie_axi_awlen;
//  logic [2:0]   pcie_axi_awsize;
//  logic [1:0]   pcie_axi_awburst;
//  logic [2:0]   pcie_axi_awprot;
//  logic         pcie_axi_awvalid;
//  logic         pcie_axi_awlock;
//  logic [3:0]   pcie_axi_awcache;
//  logic [255:0] pcie_axi_wdata;
//  logic [31:0]  pcie_axi_wstrb;
//  logic         pcie_axi_wlast;
//  logic         pcie_axi_wvalid;
//  logic         pcie_axi_bready;
//  logic [3:0]   pcie_axi_arid;
//  logic [63:0]  pcie_axi_araddr;
//  logic [7:0]   pcie_axi_arlen;
//  logic [2:0]   pcie_axi_arsize;
//  logic [1:0]   pcie_axi_arburst;
//  logic [2:0]   pcie_axi_arprot;
//  logic         pcie_axi_arvalid;
//  logic         pcie_axi_arlock;
//  logic [3:0]   pcie_axi_arcache;
//  logic         pcie_axi_rready;
//
//  logic [63:0]  pcie_dwidth_axi_awaddr;
//  logic [7:0]   pcie_dwidth_axi_awlen;
//  logic [2:0]   pcie_dwidth_axi_awsize;
//  logic [1:0]   pcie_dwidth_axi_awburst;
//  logic [0:0]   pcie_dwidth_axi_awlock;
//  logic [3:0]   pcie_dwidth_axi_awcache;
//  logic [2:0]   pcie_dwidth_axi_awprot;
//  logic [3:0]   pcie_dwidth_axi_awregion;
//  logic [3:0]   pcie_dwidth_axi_awqos;
//  logic         pcie_dwidth_axi_awvalid;
//  logic         pcie_dwidth_axi_awready;
//  logic [63:0]  pcie_dwidth_axi_wdata;
//  logic [7:0]   pcie_dwidth_axi_wstrb;
//  logic         pcie_dwidth_axi_wlast;
//  logic         pcie_dwidth_axi_wvalid;
//  logic         pcie_dwidth_axi_wready;
//  logic [1:0]   pcie_dwidth_axi_bresp;
//  logic         pcie_dwidth_axi_bvalid;
//  logic         pcie_dwidth_axi_bready;
//  logic [63:0]  pcie_dwidth_axi_araddr;
//  logic [7:0]   pcie_dwidth_axi_arlen;
//  logic [2:0]   pcie_dwidth_axi_arsize;
//  logic [1:0]   pcie_dwidth_axi_arburst;
//  logic [0:0]   pcie_dwidth_axi_arlock;
//  logic [3:0]   pcie_dwidth_axi_arcache;
//  logic [2:0]   pcie_dwidth_axi_arprot;
//  logic [3:0]   pcie_dwidth_axi_arregion;
//  logic [3:0]   pcie_dwidth_axi_arqos;
//  logic         pcie_dwidth_axi_arvalid;
//  logic         pcie_dwidth_axi_arready;
//  logic [63:0]  pcie_dwidth_axi_rdata;
//  logic [1:0]   pcie_dwidth_axi_rresp;
//  logic         pcie_dwidth_axi_rlast;
//  logic         pcie_dwidth_axi_rvalid;
//  logic         pcie_dwidth_axi_rready;
//
//  // PCIe Reset
//  logic sys_rst_n_c;
//  IBUF sys_reset_n_ibuf (.O(sys_rst_n_c), .I(sys_rst_n));
//
//  IBUFDS_GTE4 #(
//    .REFCLK_HROW_CK_SEL ( 2'b00 )
//  ) IBUFDS_GTE4_inst (
//    .O     ( pcie_ref_clk_gt ),
//    .ODIV2 ( pcie_ref_clk    ),
//    .CEB   ( 1'b0            ),
//    .I     ( sys_clk_p       ),
//    .IB    ( sys_clk_n       )
//  );
//
//  // 250 MHz AXI
//  xdma_0 i_xdma (
//    .sys_clk                  ( pcie_ref_clk     ),
//    .sys_clk_gt               ( pcie_ref_clk_gt  ),
//    .sys_rst_n                ( sys_rst_n_c      ),
//    .user_lnk_up              (                  ),
//
//    // Tx
//    .pci_exp_txp              ( pci_exp_txp      ),
//    .pci_exp_txn              ( pci_exp_txn      ),
//    // Rx
//    .pci_exp_rxp              ( pci_exp_rxp      ),
//    .pci_exp_rxn              ( pci_exp_rxn      ),
//    .usr_irq_req              ( 1'b0             ),
//    .usr_irq_ack              (                  ),
//    .msi_enable               (                  ),
//    .msi_vector_width         (                  ),
//    .axi_aclk                 ( pcie_axi_clk     ),
//    .axi_aresetn              ( pcie_axi_rstn    ),
//    .m_axi_awready            ( pcie_axi_awready ),
//    .m_axi_wready             ( pcie_axi_wready  ),
//    .m_axi_bid                ( pcie_axi_bid     ),
//    .m_axi_bresp              ( pcie_axi_bresp   ),
//    .m_axi_bvalid             ( pcie_axi_bvalid  ),
//    .m_axi_arready            ( pcie_axi_arready ),
//    .m_axi_rid                ( pcie_axi_rid     ),
//    .m_axi_rdata              ( pcie_axi_rdata   ),
//    .m_axi_rresp              ( pcie_axi_rresp   ),
//    .m_axi_rlast              ( pcie_axi_rlast   ),
//    .m_axi_rvalid             ( pcie_axi_rvalid  ),
//    .m_axi_awid               ( pcie_axi_awid    ),
//    .m_axi_awaddr             ( pcie_axi_awaddr  ),
//    .m_axi_awlen              ( pcie_axi_awlen   ),
//    .m_axi_awsize             ( pcie_axi_awsize  ),
//    .m_axi_awburst            ( pcie_axi_awburst ),
//    .m_axi_awprot             ( pcie_axi_awprot  ),
//    .m_axi_awvalid            ( pcie_axi_awvalid ),
//    .m_axi_awlock             ( pcie_axi_awlock  ),
//    .m_axi_awcache            ( pcie_axi_awcache ),
//    .m_axi_wdata              ( pcie_axi_wdata   ),
//    .m_axi_wstrb              ( pcie_axi_wstrb   ),
//    .m_axi_wlast              ( pcie_axi_wlast   ),
//    .m_axi_wvalid             ( pcie_axi_wvalid  ),
//    .m_axi_bready             ( pcie_axi_bready  ),
//    .m_axi_arid               ( pcie_axi_arid    ),
//    .m_axi_araddr             ( pcie_axi_araddr  ),
//    .m_axi_arlen              ( pcie_axi_arlen   ),
//    .m_axi_arsize             ( pcie_axi_arsize  ),
//    .m_axi_arburst            ( pcie_axi_arburst ),
//    .m_axi_arprot             ( pcie_axi_arprot  ),
//    .m_axi_arvalid            ( pcie_axi_arvalid ),
//    .m_axi_arlock             ( pcie_axi_arlock  ),
//    .m_axi_arcache            ( pcie_axi_arcache ),
//    .m_axi_rready             ( pcie_axi_rready  ),
//
//    .cfg_mgmt_addr            ( '0               ),
//    .cfg_mgmt_write           ( '0               ),
//    .cfg_mgmt_write_data      ( '0               ),
//    .cfg_mgmt_byte_enable     ( '0               ),
//    .cfg_mgmt_read            ( '0               ),
//    .cfg_mgmt_read_data       (                  ),
//    .cfg_mgmt_read_write_done (                  )
//  );
//
//  axi_dwidth_converter_256_64 i_axi_dwidth_converter_256_64 (
//    .s_axi_aclk     ( pcie_axi_clk             ),
//    .s_axi_aresetn  ( pcie_axi_rstn            ),
//    .s_axi_awid     ( pcie_axi_awid            ),
//    .s_axi_awaddr   ( pcie_axi_awaddr          ),
//    .s_axi_awlen    ( pcie_axi_awlen           ),
//    .s_axi_awsize   ( pcie_axi_awsize          ),
//    .s_axi_awburst  ( pcie_axi_awburst         ),
//    .s_axi_awlock   ( pcie_axi_awlock          ),
//    .s_axi_awcache  ( pcie_axi_awcache         ),
//    .s_axi_awprot   ( pcie_axi_awprot          ),
//    .s_axi_awregion ( '0                       ),
//    .s_axi_awqos    ( '0                       ),
//    .s_axi_awvalid  ( pcie_axi_awvalid         ),
//    .s_axi_awready  ( pcie_axi_awready         ),
//    .s_axi_wdata    ( pcie_axi_wdata           ),
//    .s_axi_wstrb    ( pcie_axi_wstrb           ),
//    .s_axi_wlast    ( pcie_axi_wlast           ),
//    .s_axi_wvalid   ( pcie_axi_wvalid          ),
//    .s_axi_wready   ( pcie_axi_wready          ),
//    .s_axi_bid      ( pcie_axi_bid             ),
//    .s_axi_bresp    ( pcie_axi_rresp           ),
//    .s_axi_bvalid   ( pcie_axi_bvalid          ),
//    .s_axi_bready   ( pcie_axi_bready          ),
//    .s_axi_arid     ( pcie_axi_arid            ),
//    .s_axi_araddr   ( pcie_axi_araddr          ),
//    .s_axi_arlen    ( pcie_axi_arlen           ),
//    .s_axi_arsize   ( pcie_axi_arsize          ),
//    .s_axi_arburst  ( pcie_axi_arburst         ),
//    .s_axi_arlock   ( pcie_axi_arlock          ),
//    .s_axi_arcache  ( pcie_axi_arcache         ),
//    .s_axi_arprot   ( pcie_axi_arprot          ),
//    .s_axi_arregion ( '0                       ),
//    .s_axi_arqos    ( '0                       ),
//    .s_axi_arvalid  ( pcie_axi_arvalid         ),
//    .s_axi_arready  ( pcie_axi_arready         ),
//    .s_axi_rid      ( pcie_axi_rid             ),
//    .s_axi_rdata    ( pcie_axi_rdata           ),
//    .s_axi_rresp    ( pcie_axi_bresp           ),
//    .s_axi_rlast    ( pcie_axi_rlast           ),
//    .s_axi_rvalid   ( pcie_axi_rvalid          ),
//    .s_axi_rready   ( pcie_axi_rready          ),
//
//    .m_axi_awaddr   ( pcie_dwidth_axi_awaddr   ),
//    .m_axi_awlen    ( pcie_dwidth_axi_awlen    ),
//    .m_axi_awsize   ( pcie_dwidth_axi_awsize   ),
//    .m_axi_awburst  ( pcie_dwidth_axi_awburst  ),
//    .m_axi_awlock   ( pcie_dwidth_axi_awlock   ),
//    .m_axi_awcache  ( pcie_dwidth_axi_awcache  ),
//    .m_axi_awprot   ( pcie_dwidth_axi_awprot   ),
//    .m_axi_awregion ( pcie_dwidth_axi_awregion ),
//    .m_axi_awqos    ( pcie_dwidth_axi_awqos    ),
//    .m_axi_awvalid  ( pcie_dwidth_axi_awvalid  ),
//    .m_axi_awready  ( pcie_dwidth_axi_awready  ),
//    .m_axi_wdata    ( pcie_dwidth_axi_wdata    ),
//    .m_axi_wstrb    ( pcie_dwidth_axi_wstrb    ),
//    .m_axi_wlast    ( pcie_dwidth_axi_wlast    ),
//    .m_axi_wvalid   ( pcie_dwidth_axi_wvalid   ),
//    .m_axi_wready   ( pcie_dwidth_axi_wready   ),
//    .m_axi_bresp    ( pcie_dwidth_axi_bresp    ),
//    .m_axi_bvalid   ( pcie_dwidth_axi_bvalid   ),
//    .m_axi_bready   ( pcie_dwidth_axi_bready   ),
//    .m_axi_araddr   ( pcie_dwidth_axi_araddr   ),
//    .m_axi_arlen    ( pcie_dwidth_axi_arlen    ),
//    .m_axi_arsize   ( pcie_dwidth_axi_arsize   ),
//    .m_axi_arburst  ( pcie_dwidth_axi_arburst  ),
//    .m_axi_arlock   ( pcie_dwidth_axi_arlock   ),
//    .m_axi_arcache  ( pcie_dwidth_axi_arcache  ),
//    .m_axi_arprot   ( pcie_dwidth_axi_arprot   ),
//    .m_axi_arregion ( pcie_dwidth_axi_arregion ),
//    .m_axi_arqos    ( pcie_dwidth_axi_arqos    ),
//    .m_axi_arvalid  ( pcie_dwidth_axi_arvalid  ),
//    .m_axi_arready  ( pcie_dwidth_axi_arready  ),
//    .m_axi_rdata    ( pcie_dwidth_axi_rdata    ),
//    .m_axi_rresp    ( pcie_dwidth_axi_rresp    ),
//    .m_axi_rlast    ( pcie_dwidth_axi_rlast    ),
//    .m_axi_rvalid   ( pcie_dwidth_axi_rvalid   ),
//    .m_axi_rready   ( pcie_dwidth_axi_rready   )
//  );
//
//
//assign slave[1].aw_user = '0;
//assign slave[1].ar_user = '0;
//assign slave[1].w_user = '0;
//
//logic [3:0] slave_b_id;
//logic [3:0] slave_r_id;
//
//assign slave[1].b_id = slave_b_id[1:0];
//assign slave[1].r_id = slave_r_id[1:0];
//
//// PCIe Clock Converter
//axi_clock_converter_0 pcie_axi_clock_converter (
//  .m_axi_aclk     ( clk                      ),
//  .m_axi_aresetn  ( ndmreset_n               ),
//  .m_axi_awid     ( {2'b0, slave[1].aw_id} ),
//  .m_axi_awaddr   ( slave[1].aw_addr   ),
//  .m_axi_awlen    ( slave[1].aw_len    ),
//  .m_axi_awsize   ( slave[1].aw_size   ),
//  .m_axi_awburst  ( slave[1].aw_burst  ),
//  .m_axi_awlock   ( slave[1].aw_lock   ),
//  .m_axi_awcache  ( slave[1].aw_cache  ),
//  .m_axi_awprot   ( slave[1].aw_prot   ),
//  .m_axi_awregion ( slave[1].aw_region ),
//  .m_axi_awqos    ( slave[1].aw_qos    ),
//  .m_axi_awvalid  ( slave[1].aw_valid  ),
//  .m_axi_awready  ( slave[1].aw_ready  ),
//  .m_axi_wdata    ( slave[1].w_data    ),
//  .m_axi_wstrb    ( slave[1].w_strb    ),
//  .m_axi_wlast    ( slave[1].w_last    ),
//  .m_axi_wvalid   ( slave[1].w_valid   ),
//  .m_axi_wready   ( slave[1].w_ready   ),
//  .m_axi_bid      ( slave_b_id         ),
//  .m_axi_bresp    ( slave[1].b_resp    ),
//  .m_axi_bvalid   ( slave[1].b_valid   ),
//  .m_axi_bready   ( slave[1].b_ready   ),
//  .m_axi_arid     ( {2'b0, slave[1].ar_id} ),
//  .m_axi_araddr   ( slave[1].ar_addr   ),
//  .m_axi_arlen    ( slave[1].ar_len    ),
//  .m_axi_arsize   ( slave[1].ar_size   ),
//  .m_axi_arburst  ( slave[1].ar_burst  ),
//  .m_axi_arlock   ( slave[1].ar_lock   ),
//  .m_axi_arcache  ( slave[1].ar_cache  ),
//  .m_axi_arprot   ( slave[1].ar_prot   ),
//  .m_axi_arregion ( slave[1].ar_region ),
//  .m_axi_arqos    ( slave[1].ar_qos    ),
//  .m_axi_arvalid  ( slave[1].ar_valid  ),
//  .m_axi_arready  ( slave[1].ar_ready  ),
//  .m_axi_rid      ( slave_r_id         ),
//  .m_axi_rdata    ( slave[1].r_data    ),
//  .m_axi_rresp    ( slave[1].r_resp    ),
//  .m_axi_rlast    ( slave[1].r_last    ),
//  .m_axi_rvalid   ( slave[1].r_valid   ),
//  .m_axi_rready   ( slave[1].r_ready   ),
//  // from size converter
//  .s_axi_aclk     ( pcie_axi_clk             ),
//  .s_axi_aresetn  ( ndmreset_n               ),
//  .s_axi_awid     ( '0                       ),
//  .s_axi_awaddr   ( pcie_dwidth_axi_awaddr   ),
//  .s_axi_awlen    ( pcie_dwidth_axi_awlen    ),
//  .s_axi_awsize   ( pcie_dwidth_axi_awsize   ),
//  .s_axi_awburst  ( pcie_dwidth_axi_awburst  ),
//  .s_axi_awlock   ( pcie_dwidth_axi_awlock   ),
//  .s_axi_awcache  ( pcie_dwidth_axi_awcache  ),
//  .s_axi_awprot   ( pcie_dwidth_axi_awprot   ),
//  .s_axi_awregion ( pcie_dwidth_axi_awregion ),
//  .s_axi_awqos    ( pcie_dwidth_axi_awqos    ),
//  .s_axi_awvalid  ( pcie_dwidth_axi_awvalid  ),
//  .s_axi_awready  ( pcie_dwidth_axi_awready  ),
//  .s_axi_wdata    ( pcie_dwidth_axi_wdata    ),
//  .s_axi_wstrb    ( pcie_dwidth_axi_wstrb    ),
//  .s_axi_wlast    ( pcie_dwidth_axi_wlast    ),
//  .s_axi_wvalid   ( pcie_dwidth_axi_wvalid   ),
//  .s_axi_wready   ( pcie_dwidth_axi_wready   ),
//  .s_axi_bid      (                          ),
//  .s_axi_bresp    ( pcie_dwidth_axi_bresp    ),
//  .s_axi_bvalid   ( pcie_dwidth_axi_bvalid   ),
//  .s_axi_bready   ( pcie_dwidth_axi_bready   ),
//  .s_axi_arid     ( '0                       ),
//  .s_axi_araddr   ( pcie_dwidth_axi_araddr   ),
//  .s_axi_arlen    ( pcie_dwidth_axi_arlen    ),
//  .s_axi_arsize   ( pcie_dwidth_axi_arsize   ),
//  .s_axi_arburst  ( pcie_dwidth_axi_arburst  ),
//  .s_axi_arlock   ( pcie_dwidth_axi_arlock   ),
//  .s_axi_arcache  ( pcie_dwidth_axi_arcache  ),
//  .s_axi_arprot   ( pcie_dwidth_axi_arprot   ),
//  .s_axi_arregion ( pcie_dwidth_axi_arregion ),
//  .s_axi_arqos    ( pcie_dwidth_axi_arqos    ),
//  .s_axi_arvalid  ( pcie_dwidth_axi_arvalid  ),
//  .s_axi_arready  ( pcie_dwidth_axi_arready  ),
//  .s_axi_rid      (                          ),
//  .s_axi_rdata    ( pcie_dwidth_axi_rdata    ),
//  .s_axi_rresp    ( pcie_dwidth_axi_rresp    ),
//  .s_axi_rlast    ( pcie_dwidth_axi_rlast    ),
//  .s_axi_rvalid   ( pcie_dwidth_axi_rvalid   ),
//  .s_axi_rready   ( pcie_dwidth_axi_rready   )
//);
//`endif

endmodule
