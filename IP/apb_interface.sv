// MODULE DRIVER
module apb_interface#(
    parameter  DATA_WIDTH = 32,
    parameter  ADDR_WIDTH = 32
)(
    input logic clk,
    input logic reset,

    // Input from FSM
    input logic fsm_psel,
    input logic fsm_penable,
    input logic fsm_pwrite,
    input logic [ADDR_WIDTH-1:0] fsm_paddr,
    input logic [DATA_WIDTH-1:0] fsm_pwdata,
    input logic [3:0] fsm_pstrb,
    input logic [2:0] fsm_pprot,
    
    // Output to APB Bus
    output logic psel,
    output logic penable,
    output logic pwrite,
    output logic [ADDR_WIDTH-1:0] paddr,
    output logic [DATA_WIDTH-1:0] pwdata,
    output logic [DATA_WIDTH/8-1:0] pstrb,
    output logic [2:0] pprot
);

// Pipeline buffer - adds 1 clock delay for better timing
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        psel <= 1'b0;
        penable <= 1'b0;
        pwrite <= 1'b0;
        paddr <= '0;
        pwdata <= '0;
        pstrb <= '0;
        pprot <= '0;
    end else begin
        psel <= fsm_psel;
        penable <= fsm_penable;
        pwrite <= fsm_pwrite;
        paddr <= fsm_paddr;
        pwdata <= fsm_pwdata;
        pstrb <= fsm_pstrb;
        pprot <= fsm_pprot;
    end
end

endmodule



