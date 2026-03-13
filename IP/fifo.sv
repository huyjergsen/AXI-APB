// =============================================================================
// Module      : fifo
// Project     : AXI-to-APB Bridge IP
// Description : Generic synchronous FIFO with parametric depth and data width.
//               Uses an extra pointer bit for full/empty detection (no BRAM
//               inference needed). Read data is combinational (zero-latency).
// -----------------------------------------------------------------------------
// Parameters:
//   DATA_WIDTH - Width of each FIFO entry  (default: 32)
//   DEPTH      - Number of entries in FIFO (default: 16)
// =============================================================================

module fifo #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 16
)(
    input  logic                    clk,
    input  logic                    reset,
    input  logic                    wr_en,
    input  logic [DATA_WIDTH-1:0]   wr_data,
    input  logic                    rd_en,
    output logic [DATA_WIDTH-1:0]   rd_data,
    output logic                    full,
    output logic                    empty
);

// =============================================================================
// Internal Storage
// =============================================================================
localparam ADDR_WIDTH = $clog2(DEPTH);

logic [DATA_WIDTH-1:0] memory [DEPTH];
logic [ADDR_WIDTH:0]   wr_ptr, rd_ptr; // MSB used for wrap-around detection
logic [ADDR_WIDTH:0]   count;

// =============================================================================
// Write Operation
// =============================================================================
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        wr_ptr <= '0;
    end else if (wr_en && !full) begin
        memory[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;
        wr_ptr <= wr_ptr + 1;
    end
end

// =============================================================================
// Read Operation
// =============================================================================
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        rd_ptr <= '0;
    end else if (rd_en && !empty) begin
        rd_ptr <= rd_ptr + 1;
    end
end

// Combinational read: data appears immediately at rd_data
assign rd_data = memory[rd_ptr[ADDR_WIDTH-1:0]];

// =============================================================================
// Occupancy Counter
// =============================================================================
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        count <= '0;
    end else begin
        case ({wr_en && !full, rd_en && !empty})
            2'b10:   count <= count + 1; // Write only
            2'b01:   count <= count - 1; // Read only
            default: count <= count;     // Idle or simultaneous RW
        endcase
    end
end

// =============================================================================
// Status Flags
// =============================================================================
assign full  = (count == DEPTH);
assign empty = (count == 0);

endmodule
