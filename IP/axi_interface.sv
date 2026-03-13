// =============================================================================
// Module      : axi_interface
// Project     : AXI-to-APB Bridge IP
// Description : AXI4 interface layer. Captures AW/AR handshakes into internal
//               registers and exposes a unified command bus to the bridge core.
//               Write channel is pass-through; prioritizes write over read.
// -----------------------------------------------------------------------------
// Parameters:
//   DATA_WIDTH - AXI data bus width         (default: 32)
//   ADDR_WIDTH - AXI address bus width      (default: 32)
//   ID_WIDTH   - AXI transaction ID width   (default:  4)
// =============================================================================

module axi_interface #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH   = 4
)(
    input  logic clk,
    input  logic reset,

    // -------------------------------------------------------------------------
    // AXI4 Write Address Channel (AW)
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
    // AXI4 Write Data Channel (W)
    // -------------------------------------------------------------------------
    input  logic                    wvalid,
    output logic                    wready,
    input  logic [DATA_WIDTH-1:0]   wdata,
    input  logic [DATA_WIDTH/8-1:0] wstrb,
    input  logic                    wlast,
    input  logic [ID_WIDTH-1:0]     wid,

    // -------------------------------------------------------------------------
    // AXI4 Write Response Channel (B)
    // -------------------------------------------------------------------------
    output logic                    bvalid,
    input  logic                    bready,
    output logic [ID_WIDTH-1:0]     bid,
    output logic [1:0]              bresp,

    // -------------------------------------------------------------------------
    // AXI4 Read Address Channel (AR)
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
    // AXI4 Read Data Channel (R)
    // -------------------------------------------------------------------------
    output logic                    rvalid,
    input  logic                    rready,
    output logic [ID_WIDTH-1:0]     rid,
    output logic [DATA_WIDTH-1:0]   rdata,
    output logic                    rlast,
    output logic [1:0]              rresp,

    // -------------------------------------------------------------------------
    // Internal Command Interface (to bridge core)
    // -------------------------------------------------------------------------
    output logic                    cmd_valid,
    input  logic                    cmd_ready,
    output logic [ADDR_WIDTH-1:0]   cmd_addr,
    output logic                    cmd_write,
    output logic [ID_WIDTH-1:0]     cmd_id,
    output logic [7:0]              cmd_len,
    output logic [1:0]              cmd_burst,
    output logic [2:0]              cmd_size,
    output logic [2:0]              cmd_prot,

    // -------------------------------------------------------------------------
    // Internal Write Data Interface (to bridge core)
    // -------------------------------------------------------------------------
    output logic                    wdata_valid,
    input  logic                    wdata_ready,
    output logic [DATA_WIDTH-1:0]   wdata_data,
    output logic [DATA_WIDTH/8-1:0] wdata_strb,
    output logic                    wdata_last,
    output logic [ID_WIDTH-1:0]     wdata_id,

    // -------------------------------------------------------------------------
    // Internal Read Data Interface (from bridge core)
    // -------------------------------------------------------------------------
    input  logic                    rdata_valid,
    output logic                    rdata_ready,
    input  logic [DATA_WIDTH-1:0]   rdata_data,
    input  logic [ID_WIDTH-1:0]     rdata_id,
    input  logic                    rdata_last,
    input  logic [1:0]              rdata_resp,

    // -------------------------------------------------------------------------
    // Internal Write Response Interface (from bridge core)
    // -------------------------------------------------------------------------
    input  logic                    bresp_valid,
    output logic                    bresp_ready,
    input  logic [ID_WIDTH-1:0]     bresp_id,
    input  logic [1:0]              bresp_resp
);

// =============================================================================
// Internal Registers
// =============================================================================
logic                  aw_pending, ar_pending;
logic [ADDR_WIDTH-1:0] aw_addr_reg, ar_addr_reg;
logic [ID_WIDTH-1:0]   aw_id_reg,   ar_id_reg;
logic [7:0]            aw_len_reg,  ar_len_reg;
logic [1:0]            aw_burst_reg, ar_burst_reg;
logic [2:0]            aw_size_reg,  ar_size_reg;
logic [2:0]            aw_prot_reg,  ar_prot_reg;

// =============================================================================
// Write Address Channel Handshake
// =============================================================================
wire aw_handshake = awvalid && awready;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        awready      <= 1'b1; // Accept immediately after reset
        aw_pending   <= 1'b0;
        aw_addr_reg  <= '0;
        aw_id_reg    <= '0;
        aw_len_reg   <= '0;
        aw_burst_reg <= '0;
        aw_size_reg  <= '0;
        aw_prot_reg  <= '0;
    end else begin
        if (aw_handshake) begin
            // Latch AW fields; de-assert ready until command is consumed
            awready      <= 1'b0;
            aw_pending   <= 1'b1;
            aw_addr_reg  <= awaddr;
            aw_id_reg    <= awid;
            aw_len_reg   <= awlen;
            aw_burst_reg <= awburst;
            aw_size_reg  <= awsize;
            aw_prot_reg  <= awprot;
        end else if (cmd_valid && cmd_ready && cmd_write) begin
            // Command accepted by core; ready for next AW transaction
            aw_pending <= 1'b0;
            awready    <= 1'b1;
        end
    end
end

// =============================================================================
// Write Data Channel - Pass-Through
// =============================================================================
assign wready      = wdata_ready;
assign wdata_valid = wvalid;
assign wdata_data  = wdata;
assign wdata_strb  = wstrb;
assign wdata_last  = wlast;
assign wdata_id    = wid;

// =============================================================================
// Read Address Channel Handshake
// =============================================================================
wire ar_handshake = arvalid && arready;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        arready      <= 1'b1;
        ar_pending   <= 1'b0;
        ar_addr_reg  <= '0;
        ar_id_reg    <= '0;
        ar_len_reg   <= '0;
        ar_burst_reg <= '0;
        ar_size_reg  <= '0;
        ar_prot_reg  <= '0;
    end else begin
        if (ar_handshake) begin
            arready      <= 1'b0;
            ar_pending   <= 1'b1;
            ar_addr_reg  <= araddr;
            ar_id_reg    <= arid;
            ar_len_reg   <= arlen;
            ar_burst_reg <= arburst;
            ar_size_reg  <= arsize;
            ar_prot_reg  <= arprot;
        end else if (cmd_valid && cmd_ready && !cmd_write) begin
            ar_pending <= 1'b0;
            arready    <= 1'b1;
        end
    end
end

// =============================================================================
// Command Interface - Write takes priority over Read
// =============================================================================
assign cmd_valid = aw_pending || ar_pending;
assign cmd_write = aw_pending;
assign cmd_addr  = aw_pending ? aw_addr_reg  : ar_addr_reg;
assign cmd_id    = aw_pending ? aw_id_reg    : ar_id_reg;
assign cmd_len   = aw_pending ? aw_len_reg   : ar_len_reg;
assign cmd_burst = aw_pending ? aw_burst_reg : ar_burst_reg;
assign cmd_size  = aw_pending ? aw_size_reg  : ar_size_reg;
assign cmd_prot  = aw_pending ? aw_prot_reg  : ar_prot_reg;

// =============================================================================
// Read Data Channel - Pass-Through
// =============================================================================
assign rvalid      = rdata_valid;
assign rdata       = rdata_data;
assign rid         = rdata_id;
assign rlast       = rdata_last;
assign rresp       = rdata_resp;
assign rdata_ready = rready;

// =============================================================================
// Write Response Channel - Pass-Through
// =============================================================================
assign bvalid      = bresp_valid;
assign bid         = bresp_id;
assign bresp       = bresp_resp;
assign bresp_ready = bready;

endmodule