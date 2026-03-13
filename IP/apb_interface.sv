// =============================================================================
// Module      : apb_interface
// Project     : AXI-to-APB Bridge IP
// Description : Combinational pass-through layer between the FSM and the
//               physical APB4 master bus. Maintains zero-latency handshake;
//               all signals are wired directly without registering.
// -----------------------------------------------------------------------------
// Parameters:
//   DATA_WIDTH - APB data bus width    (default: 32)
//   ADDR_WIDTH - APB address bus width (default: 32)
// =============================================================================

module apb_interface #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
)(
    input  logic clk,   // Unused; kept for structural consistency
    input  logic reset, // Unused; kept for structural consistency

    // -------------------------------------------------------------------------
    // FSM-Side Inputs
    // -------------------------------------------------------------------------
    input  logic                    fsm_psel,
    input  logic                    fsm_penable,
    input  logic                    fsm_pwrite,
    input  logic [ADDR_WIDTH-1:0]   fsm_paddr,
    input  logic [DATA_WIDTH-1:0]   fsm_pwdata,
    input  logic [DATA_WIDTH/8-1:0] fsm_pstrb,
    input  logic [2:0]              fsm_pprot,

    // -------------------------------------------------------------------------
    // APB4 Master Bus Outputs
    // -------------------------------------------------------------------------
    output logic                    psel,
    output logic                    penable,
    output logic                    pwrite,
    output logic [ADDR_WIDTH-1:0]   paddr,
    output logic [DATA_WIDTH-1:0]   pwdata,
    output logic [DATA_WIDTH/8-1:0] pstrb,
    output logic [2:0]              pprot
);

// =============================================================================
// Pass-Through Logic (Combinational)
// =============================================================================
always_comb begin
    psel    = fsm_psel;
    penable = fsm_penable;
    pwrite  = fsm_pwrite;
    paddr   = fsm_paddr;
    pwdata  = fsm_pwdata;
    pstrb   = fsm_pstrb;
    pprot   = fsm_pprot;
end

endmodule