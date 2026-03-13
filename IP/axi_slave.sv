// =============================================================================
// Module      : axi_slave
// Project     : AXI-to-APB Bridge IP
// Description : AXI4 slave top-level wrapper. Performs address range checking,
//               transaction counting, and error tracking before forwarding
//               traffic to the bridge core module.
// -----------------------------------------------------------------------------
// Parameters:
//   DATA_WIDTH - AXI/APB data bus width        (default:       32)
//   ADDR_WIDTH - AXI/APB address bus width     (default:       32)
//   ID_WIDTH   - AXI transaction ID width      (default:        4)
//   FIFO_DEPTH - Internal FIFO depth           (default:       16)
//   ADDR_BASE  - Base address of mapped region (default: 0x1000_0000)
//   ADDR_SIZE  - Size of mapped region (bytes) (default: 0x0010_0000 = 1 MB)
// =============================================================================

module axi_slave #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH   = 4,
    parameter FIFO_DEPTH = 16,
    parameter ADDR_BASE  = 32'h1000_0000,
    parameter ADDR_SIZE  = 32'h0010_0000   // 1 MB address space
)(
    input  logic clk,
    input  logic reset,

    // -------------------------------------------------------------------------
    // AXI4 Slave Interface - Write Address Channel (AW)
    // -------------------------------------------------------------------------
    input  logic                    awvalid,
    output logic                    awready,
    input  logic [ADDR_WIDTH-1:0]   awaddr,
    input  logic [ID_WIDTH-1:0]     awid,
    input  logic [7:0]              awlen,
    input  logic [1:0]              awburst,
    input  logic [2:0]              awprot,
    input  logic [2:0]              awsize,
    input  logic                    awlock,
    input  logic [3:0]              awcache,
    input  logic [3:0]              awqos,
    input  logic [3:0]              awregion,

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
    // AXI4 Slave Interface - Write Response Channel (B)
    // -------------------------------------------------------------------------
    output logic                    bvalid,
    input  logic                    bready,
    output logic [ID_WIDTH-1:0]     bid,
    output logic [1:0]              bresp,

    // -------------------------------------------------------------------------
    // AXI4 Slave Interface - Read Address Channel (AR)
    // -------------------------------------------------------------------------
    input  logic                    arvalid,
    output logic                    arready,
    input  logic [ADDR_WIDTH-1:0]   araddr,
    input  logic [ID_WIDTH-1:0]     arid,
    input  logic [7:0]              arlen,
    input  logic [1:0]              arburst,
    input  logic [2:0]              arprot,
    input  logic [2:0]              arsize,
    input  logic                    arlock,
    input  logic [3:0]              arcache,
    input  logic [3:0]              arqos,
    input  logic [3:0]              arregion,

    // -------------------------------------------------------------------------
    // AXI4 Slave Interface - Read Data Channel (R)
    // -------------------------------------------------------------------------
    output logic                    rvalid,
    input  logic                    rready,
    output logic [ID_WIDTH-1:0]     rid,
    output logic [DATA_WIDTH-1:0]   rdata,
    output logic                    rlast,
    output logic [1:0]              rresp,

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
    output logic        irq,
    input  logic        debug_req,
    output logic        bridge_active,
    output logic [31:0] error_count,
    output logic [31:0] transaction_count
);

// =============================================================================
// Internal Signals - Core Connection
// =============================================================================

// Write address channel
logic                  core_awvalid, core_awready;
logic [ADDR_WIDTH-1:0] core_awaddr;
logic [ID_WIDTH-1:0]   core_awid;
logic [7:0]            core_awlen;
logic [1:0]            core_awburst;
logic [2:0]            core_awprot, core_awsize;
logic                  core_awlock;
logic [3:0]            core_awcache, core_awqos, core_awregion;

// Read address channel
logic                  core_arvalid, core_arready;
logic [ADDR_WIDTH-1:0] core_araddr;
logic [ID_WIDTH-1:0]   core_arid;
logic [7:0]            core_arlen;
logic [1:0]            core_arburst;
logic [2:0]            core_arprot, core_arsize;
logic                  core_arlock;
logic [3:0]            core_arcache, core_arqos, core_arregion;

// Write data channel
logic                    core_wvalid, core_wready;
logic [DATA_WIDTH-1:0]   core_wdata;
logic [DATA_WIDTH/8-1:0] core_wstrb;
logic                    core_wlast;
logic [ID_WIDTH-1:0]     core_wid;

// Read data channel
logic                  core_rvalid, core_rready;
logic [ID_WIDTH-1:0]   core_rid;
logic [DATA_WIDTH-1:0] core_rdata;
logic                  core_rlast;
logic [1:0]            core_rresp;

// Write response channel
logic              core_bvalid, core_bready;
logic [ID_WIDTH-1:0] core_bid;
logic [1:0]          core_bresp;

// =============================================================================
// Address Range Checking
// =============================================================================
logic aw_addr_valid, ar_addr_valid;

assign aw_addr_valid = (awaddr >= ADDR_BASE) && (awaddr < (ADDR_BASE + ADDR_SIZE));
assign ar_addr_valid = (araddr >= ADDR_BASE) && (araddr < (ADDR_BASE + ADDR_SIZE));

// =============================================================================
// Transaction & Error Counters
// =============================================================================
logic [31:0] error_count_reg;
logic [31:0] transaction_count_reg;
logic [31:0] write_count_reg;
logic [31:0] read_count_reg;

// Handshake strobes
wire aw_handshake = awvalid && awready && aw_addr_valid;
wire ar_handshake = arvalid && arready && ar_addr_valid;
wire w_handshake  = wvalid  && wready  && wlast;
wire r_handshake  = rvalid  && rready  && rlast;
wire b_handshake  = bvalid  && bready;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        error_count_reg       <= '0;
        transaction_count_reg <= '0;
        write_count_reg       <= '0;
        read_count_reg        <= '0;
    end else begin
        // Increment on any non-OKAY response
        if ((b_handshake && bresp != 2'b00) ||
            (r_handshake && rresp != 2'b00)) begin
            error_count_reg <= error_count_reg + 1;
        end

        // Count accepted address transactions
        if (aw_handshake || ar_handshake)
            transaction_count_reg <= transaction_count_reg + 1;

        // Count successful completions
        if (b_handshake && bresp == 2'b00) write_count_reg <= write_count_reg + 1;
        if (r_handshake && rresp == 2'b00) read_count_reg  <= read_count_reg  + 1;
    end
end

// =============================================================================
// AXI Channel Wiring - Write Address (AW)
// =============================================================================
assign core_awvalid  = awvalid;
assign awready       = core_awready;
assign core_awaddr   = awaddr;
assign core_awid     = awid;
assign core_awlen    = awlen;
assign core_awburst  = awburst;
assign core_awprot   = awprot;
assign core_awsize   = awsize;
assign core_awlock   = awlock;
assign core_awcache  = awcache;
assign core_awqos    = awqos;
assign core_awregion = awregion;

// =============================================================================
// AXI Channel Wiring - Read Address (AR)
// =============================================================================
assign core_arvalid  = arvalid;
assign arready       = core_arready;
assign core_araddr   = araddr;
assign core_arid     = arid;
assign core_arlen    = arlen;
assign core_arburst  = arburst;
assign core_arprot   = arprot;
assign core_arsize   = arsize;
assign core_arlock   = arlock;
assign core_arcache  = arcache;
assign core_arqos    = arqos;
assign core_arregion = arregion;

// =============================================================================
// AXI Channel Wiring - Write Data (W)
// =============================================================================
assign core_wvalid = wvalid;
assign wready      = core_wready;
assign core_wdata  = wdata;
assign core_wstrb  = wstrb;
assign core_wlast  = wlast;
assign core_wid    = wid;

// =============================================================================
// AXI Channel Wiring - Read Data (R)
// =============================================================================
assign rvalid      = core_rvalid;
assign core_rready = rready;
assign rid         = core_rid;
assign rdata       = core_rdata;
assign rlast       = core_rlast;
assign rresp       = core_rresp;

// =============================================================================
// AXI Channel Wiring - Write Response (B)
// =============================================================================
assign bvalid      = core_bvalid;
assign core_bready = bready;
assign bid         = core_bid;
assign bresp       = core_bresp;

// =============================================================================
// Status Outputs
// =============================================================================
assign bridge_active     = core_awvalid || core_arvalid || core_wvalid ||
                           core_rvalid  || core_bvalid  ||
                           (transaction_count_reg > 0);
assign error_count       = error_count_reg;
assign transaction_count = transaction_count_reg;

// =============================================================================
// Core Bridge Instantiation
// =============================================================================
core #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .FIFO_DEPTH(FIFO_DEPTH),
    .ID_WIDTH  (ID_WIDTH),
    .ADDR_BASE (ADDR_BASE),
    .ADDR_SIZE (ADDR_SIZE)
) u_core (
    .clk   (clk),
    .reset (reset),

    // AXI4 Write Address
    .awvalid  (core_awvalid),  .awready  (core_awready),
    .awaddr   (core_awaddr),   .awsize   (core_awsize),
    .awlen    (core_awlen),    .awburst  (core_awburst),
    .awid     (core_awid),     .awprot   (core_awprot),
    .awlock   (core_awlock),   .awcache  (core_awcache),
    .awqos    (core_awqos),    .awregion (core_awregion),

    // AXI4 Read Address
    .arvalid  (core_arvalid),  .arready  (core_arready),
    .araddr   (core_araddr),   .arsize   (core_arsize),
    .arlen    (core_arlen),    .arburst  (core_arburst),
    .arid     (core_arid),     .arprot   (core_arprot),
    .arlock   (core_arlock),   .arcache  (core_arcache),
    .arqos    (core_arqos),    .arregion (core_arregion),

    // AXI4 Write Data
    .wvalid   (core_wvalid),   .wready   (core_wready),
    .wdata    (core_wdata),    .wstrb    (core_wstrb),
    .wlast    (core_wlast),    .wid      (core_wid),

    // AXI4 Read Data
    .rvalid   (core_rvalid),   .rready   (core_rready),
    .rdata    (core_rdata),    .rlast    (core_rlast),
    .rresp    (core_rresp),    .rid      (core_rid),

    // AXI4 Write Response
    .bvalid   (core_bvalid),   .bready   (core_bready),
    .bresp    (core_bresp),    .bid      (core_bid),

    // APB4 Master
    .paddr    (paddr),    .pwrite   (pwrite),   .psel     (psel),
    .penable  (penable),  .pwdata   (pwdata),   .pstrb    (pstrb),
    .pprot    (pprot),    .prdata   (prdata),   .pready   (pready),
    .pslverr  (pslverr),

    // Control
    .irq      (irq),      .debug_req(debug_req)
);

endmodule