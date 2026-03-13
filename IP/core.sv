// =============================================================================
// Module      : core
// Project     : AXI-to-APB Bridge IP
// Description : Bridge core module. Instantiates the AXI4 interface, three
//               decoupling FIFOs (request, write-data, read-data), the bridge
//               FSM, and the APB4 interface. Connects all sub-blocks via
//               internal handshake buses.
// -----------------------------------------------------------------------------
// Parameters:
//   DATA_WIDTH - AXI/APB data bus width        (default:       32)
//   ADDR_WIDTH - AXI/APB address bus width     (default:       32)
//   FIFO_DEPTH - Depth of all three FIFOs      (default:       16)
//   ID_WIDTH   - AXI transaction ID width      (default:        4)
//   ADDR_BASE  - Base address of mapped region (default: 0x1000_0000)
//   ADDR_SIZE  - Size of mapped region (bytes) (default: 0x0010_0000 = 1 MB)
// =============================================================================

module core #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter FIFO_DEPTH = 16,
    parameter ID_WIDTH   = 4,
    parameter ADDR_BASE  = 32'h1000_0000,
    parameter ADDR_SIZE  = 32'h0010_0000
)(
    input  logic clk,
    input  logic reset,

    // -------------------------------------------------------------------------
    // AXI4 Slave Interface - Write Address Channel (AW)
    // -------------------------------------------------------------------------
    input  logic                    awvalid,
    output logic                    awready,
    input  logic [ADDR_WIDTH-1:0]   awaddr,
    input  logic [2:0]              awsize,
    input  logic [7:0]              awlen,
    input  logic [1:0]              awburst,
    input  logic [ID_WIDTH-1:0]     awid,
    input  logic [2:0]              awprot,
    input  logic                    awlock,
    input  logic [3:0]              awcache,
    input  logic [3:0]              awqos,
    input  logic [3:0]              awregion,

    // -------------------------------------------------------------------------
    // AXI4 Slave Interface - Read Address Channel (AR)
    // -------------------------------------------------------------------------
    input  logic                    arvalid,
    output logic                    arready,
    input  logic [ADDR_WIDTH-1:0]   araddr,
    input  logic [2:0]              arsize,
    input  logic [7:0]              arlen,
    input  logic [1:0]              arburst,
    input  logic [ID_WIDTH-1:0]     arid,
    input  logic [2:0]              arprot,
    input  logic                    arlock,
    input  logic [3:0]              arcache,
    input  logic [3:0]              arqos,
    input  logic [3:0]              arregion,

    // -------------------------------------------------------------------------
    // AXI4 Slave Interface - Write Data Channel (W)
    // -------------------------------------------------------------------------
    input  logic                    wvalid,
    output logic                    wready,
    input  logic [DATA_WIDTH-1:0]   wdata,
    input  logic [DATA_WIDTH/8-1:0] wstrb,
    input  logic                    wlast,
    input  logic [ID_WIDTH-1:0]     wid,

    // -------------------------------------------------------------------------
    // AXI4 Slave Interface - Read Data Channel (R)
    // -------------------------------------------------------------------------
    output logic                    rvalid,
    input  logic                    rready,
    output logic                    rlast,
    output logic [1:0]              rresp,
    output logic [ID_WIDTH-1:0]     rid,
    output logic [DATA_WIDTH-1:0]   rdata,

    // -------------------------------------------------------------------------
    // AXI4 Slave Interface - Write Response Channel (B)
    // -------------------------------------------------------------------------
    output logic                    bvalid,
    input  logic                    bready,
    output logic [1:0]              bresp,
    output logic [ID_WIDTH-1:0]     bid,

    // -------------------------------------------------------------------------
    // APB4 Master Interface
    // -------------------------------------------------------------------------
    output logic [ADDR_WIDTH-1:0]   paddr,
    output logic                    pwrite,
    output logic                    psel,
    output logic                    penable,
    output logic [DATA_WIDTH-1:0]   pwdata,
    output logic [DATA_WIDTH/8-1:0] pstrb,
    output logic [2:0]              pprot,
    input  logic [DATA_WIDTH-1:0]   prdata,
    input  logic                    pready,
    input  logic                    pslverr,

    // -------------------------------------------------------------------------
    // Control & Status
    // -------------------------------------------------------------------------
    output logic irq,       // Interrupt request (reserved)
    input  logic debug_req  // Debug request (reserved)
);

// =============================================================================
// Internal Signals - AXI Interface <-> FIFO Pack/Unpack
// =============================================================================

// Command bus (AXI interface -> request FIFO)
logic                  axi_cmd_valid, axi_cmd_ready;
logic [ADDR_WIDTH-1:0] axi_cmd_addr;
logic                  axi_cmd_write;
logic [ID_WIDTH-1:0]   axi_cmd_id;
logic [7:0]            axi_cmd_len;
logic [1:0]            axi_cmd_burst;
logic [2:0]            axi_cmd_size;
logic [2:0]            axi_cmd_prot;

// Write data bus (AXI interface -> write-data FIFO)
logic                    axi_wdata_valid, axi_wdata_ready;
logic [DATA_WIDTH-1:0]   axi_wdata_data;
logic [DATA_WIDTH/8-1:0] axi_wdata_strb;
logic                    axi_wdata_last;
logic [ID_WIDTH-1:0]     axi_wdata_id;

// Read data bus (read-data FIFO -> AXI interface)
logic                  axi_rdata_valid, axi_rdata_ready;
logic [DATA_WIDTH-1:0] axi_rdata_data;
logic [ID_WIDTH-1:0]   axi_rdata_id;
logic                  axi_rdata_last;
logic [1:0]            axi_rdata_resp;

// Write response bus (FSM -> AXI interface)
logic              axi_bresp_valid, axi_bresp_ready;
logic [ID_WIDTH-1:0] axi_bresp_id;
logic [1:0]          axi_bresp_resp;

// =============================================================================
// FIFO Width Parameters
// =============================================================================
// Request FIFO  : addr(ADDR_WIDTH) | write(1) | id(ID_WIDTH) | len(8) | burst(2) | size(3) | prot(3)
// WData  FIFO  : data(DATA_WIDTH) | strb(DATA_WIDTH/8) | last(1) | id(ID_WIDTH)
// RData  FIFO  : data(DATA_WIDTH) | id(ID_WIDTH) | last(1) | resp(2)
localparam REQ_FIFO_WIDTH   = ADDR_WIDTH + 1 + ID_WIDTH + 8 + 2 + 3 + 3;
localparam WDATA_FIFO_WIDTH = DATA_WIDTH + DATA_WIDTH/8 + 1 + ID_WIDTH;
localparam RDATA_FIFO_WIDTH = DATA_WIDTH + ID_WIDTH + 1 + 2;

// =============================================================================
// Internal Signals - FIFO Control
// =============================================================================

// Request FIFO
logic                        req_fifo_wr_en,    req_fifo_rd_en;
logic                        req_fifo_full,     req_fifo_empty;
logic [REQ_FIFO_WIDTH-1:0]   req_fifo_wr_data,  req_fifo_rd_data;

// Write Data FIFO
logic                        wdata_fifo_wr_en,  wdata_fifo_rd_en;
logic                        wdata_fifo_full,   wdata_fifo_empty;
logic [WDATA_FIFO_WIDTH-1:0] wdata_fifo_wr_data, wdata_fifo_rd_data;

// Read Data FIFO
logic                        rdata_fifo_wr_en,  rdata_fifo_rd_en;
logic                        rdata_fifo_full,   rdata_fifo_empty;
logic [RDATA_FIFO_WIDTH-1:0] rdata_fifo_wr_data, rdata_fifo_rd_data;

// =============================================================================
// Internal Signals - FSM <-> APB Interface
// =============================================================================
logic                    fsm_psel, fsm_penable, fsm_pwrite;
logic [ADDR_WIDTH-1:0]   fsm_paddr;
logic [DATA_WIDTH-1:0]   fsm_pwdata;
logic [DATA_WIDTH/8-1:0] fsm_pstrb;
logic [2:0]              fsm_pprot;

// =============================================================================
// AXI Interface Instantiation
// =============================================================================
axi_interface #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .ID_WIDTH  (ID_WIDTH)
) u_axi_interface (
    .clk   (clk),
    .reset (reset),

    // AXI4 Write Address
    .awvalid  (awvalid),  .awready  (awready),
    .awaddr   (awaddr),   .awsize   (awsize),
    .awlen    (awlen),    .awburst  (awburst),
    .awid     (awid),     .awprot   (awprot),
    .awlock   (awlock),   .awcache  (awcache),
    .awqos    (awqos),    .awregion (awregion),

    // AXI4 Read Address
    .arvalid  (arvalid),  .arready  (arready),
    .araddr   (araddr),   .arsize   (arsize),
    .arlen    (arlen),    .arburst  (arburst),
    .arid     (arid),     .arprot   (arprot),
    .arlock   (arlock),   .arcache  (arcache),
    .arqos    (arqos),    .arregion (arregion),

    // AXI4 Write Data
    .wvalid   (wvalid),   .wready   (wready),
    .wdata    (wdata),    .wstrb    (wstrb),
    .wlast    (wlast),    .wid      (wid),

    // AXI4 Read Data
    .rvalid   (rvalid),   .rready   (rready),
    .rdata    (rdata),    .rlast    (rlast),
    .rresp    (rresp),    .rid      (rid),

    // AXI4 Write Response
    .bvalid   (bvalid),   .bready   (bready),
    .bresp    (bresp),    .bid      (bid),

    // Internal command bus
    .cmd_valid  (axi_cmd_valid),  .cmd_ready  (axi_cmd_ready),
    .cmd_addr   (axi_cmd_addr),   .cmd_write  (axi_cmd_write),
    .cmd_id     (axi_cmd_id),     .cmd_len    (axi_cmd_len),
    .cmd_burst  (axi_cmd_burst),  .cmd_size   (axi_cmd_size),
    .cmd_prot   (axi_cmd_prot),

    // Internal write data bus
    .wdata_valid (axi_wdata_valid), .wdata_ready (axi_wdata_ready),
    .wdata_data  (axi_wdata_data),  .wdata_strb  (axi_wdata_strb),
    .wdata_last  (axi_wdata_last),  .wdata_id    (axi_wdata_id),

    // Internal read data bus
    .rdata_valid (axi_rdata_valid), .rdata_ready (axi_rdata_ready),
    .rdata_data  (axi_rdata_data),  .rdata_id    (axi_rdata_id),
    .rdata_last  (axi_rdata_last),  .rdata_resp  (axi_rdata_resp),

    // Internal write response bus
    .bresp_valid (axi_bresp_valid), .bresp_ready (axi_bresp_ready),
    .bresp_id    (axi_bresp_id),    .bresp_resp  (axi_bresp_resp)
);

// =============================================================================
// FIFO Pack / Control Logic
// =============================================================================

// Pack request fields into FIFO word (LSB -> MSB: prot, size, burst, len, id, write, addr)
assign req_fifo_wr_data   = {axi_cmd_addr,   axi_cmd_write, axi_cmd_id,
                              axi_cmd_len,    axi_cmd_burst, axi_cmd_size, axi_cmd_prot};
// Pack write data fields into FIFO word (LSB -> MSB: id, last, strb, data)
assign wdata_fifo_wr_data = {axi_wdata_data, axi_wdata_strb, axi_wdata_last, axi_wdata_id};

// Handshake: push to FIFO on valid+ready; back-pressure when full
assign req_fifo_wr_en   = axi_cmd_valid   && axi_cmd_ready;
assign wdata_fifo_wr_en = axi_wdata_valid && axi_wdata_ready;
assign axi_cmd_ready    = !req_fifo_full;
assign axi_wdata_ready  = !wdata_fifo_full;

// =============================================================================
// Request FIFO Instantiation
// =============================================================================
fifo #(
    .DATA_WIDTH(REQ_FIFO_WIDTH),
    .DEPTH     (FIFO_DEPTH)
) u_req_fifo (
    .clk     (clk),
    .reset   (reset),
    .wr_en   (req_fifo_wr_en),
    .wr_data (req_fifo_wr_data),
    .rd_en   (req_fifo_rd_en),
    .rd_data (req_fifo_rd_data),
    .full    (req_fifo_full),
    .empty   (req_fifo_empty)
);

// =============================================================================
// Write Data FIFO Instantiation
// =============================================================================
fifo #(
    .DATA_WIDTH(WDATA_FIFO_WIDTH),
    .DEPTH     (FIFO_DEPTH)
) u_wdata_fifo (
    .clk     (clk),
    .reset   (reset),
    .wr_en   (wdata_fifo_wr_en),
    .wr_data (wdata_fifo_wr_data),
    .rd_en   (wdata_fifo_rd_en),
    .rd_data (wdata_fifo_rd_data),
    .full    (wdata_fifo_full),
    .empty   (wdata_fifo_empty)
);

// =============================================================================
// Read Data FIFO Instantiation
// =============================================================================
fifo #(
    .DATA_WIDTH(RDATA_FIFO_WIDTH),
    .DEPTH     (FIFO_DEPTH)
) u_rdata_fifo (
    .clk     (clk),
    .reset   (reset),
    .wr_en   (rdata_fifo_wr_en),
    .wr_data (rdata_fifo_wr_data),
    .rd_en   (rdata_fifo_rd_en),
    .rd_data (rdata_fifo_rd_data),
    .full    (rdata_fifo_full),
    .empty   (rdata_fifo_empty)
);

// Unpack read data FIFO word -> AXI read data bus
assign {axi_rdata_data, axi_rdata_id, axi_rdata_last, axi_rdata_resp} = rdata_fifo_rd_data;
assign axi_rdata_valid  = !rdata_fifo_empty;
assign rdata_fifo_rd_en = axi_rdata_valid && axi_rdata_ready;

// =============================================================================
// FSM Instantiation
// =============================================================================
fsm #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .ID_WIDTH  (ID_WIDTH),
    .ADDR_BASE (ADDR_BASE),
    .ADDR_SIZE (ADDR_SIZE)
) u_fsm (
    .clk   (clk),
    .reset (reset),

    // Request FIFO interface
    .req_fifo_empty    (req_fifo_empty),
    .req_fifo_rd_en    (req_fifo_rd_en),
    .req_fifo_rd_data  (req_fifo_rd_data),

    // Write Data FIFO interface
    .wdata_fifo_empty  (wdata_fifo_empty),
    .wdata_fifo_rd_en  (wdata_fifo_rd_en),
    .wdata_fifo_rd_data(wdata_fifo_rd_data),

    // Read Data FIFO interface
    .rdata_fifo_full   (rdata_fifo_full),
    .rdata_fifo_wr_en  (rdata_fifo_wr_en),
    .rdata_fifo_wr_data(rdata_fifo_wr_data),

    // Write Response bus
    .bresp_valid  (axi_bresp_valid),
    .bresp_ready  (axi_bresp_ready),
    .bresp_id     (axi_bresp_id),
    .bresp_resp   (axi_bresp_resp),

    // APB Master bus (to apb_interface layer)
    .apb_psel    (fsm_psel),    .apb_penable (fsm_penable),
    .apb_pwrite  (fsm_pwrite),  .apb_paddr   (fsm_paddr),
    .apb_pwdata  (fsm_pwdata),  .apb_pstrb   (fsm_pstrb),
    .apb_pprot   (fsm_pprot),   .apb_prdata  (prdata),
    .apb_pready  (pready),      .apb_pslverr (pslverr)
);

// =============================================================================
// APB Interface Instantiation
// =============================================================================
apb_interface #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) u_apb_interface (
    .clk        (clk),
    .reset      (reset),

    // FSM-side inputs
    .fsm_psel    (fsm_psel),    .fsm_penable (fsm_penable),
    .fsm_pwrite  (fsm_pwrite),  .fsm_paddr   (fsm_paddr),
    .fsm_pwdata  (fsm_pwdata),  .fsm_pstrb   (fsm_pstrb),
    .fsm_pprot   (fsm_pprot),

    // Physical APB4 bus outputs
    .psel    (psel),    .penable (penable),
    .pwrite  (pwrite),  .paddr   (paddr),
    .pwdata  (pwdata),  .pstrb   (pstrb),
    .pprot   (pprot)
);

// =============================================================================
// IRQ (Reserved)
// =============================================================================
assign irq = 1'b0; // Not yet implemented

endmodule