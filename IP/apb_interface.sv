// lap trinh 2 interface truoc
module apb_interface#(
    parameter  DATA_WIDTH = 32,
    parameter  ADDR_WIDTH = 32,
    parameter  ID_WIDTH = 4,
)(
    input logic clk,
    input logic reset,
    //APB signals
    input logic psel,
    input logic penable,
    input logic [DATA_WIDTH-1:0] pwdata,
    input logic [DATA_WIDTH-1:0] pwrite,
    input logic [ADDR_WIDTH-1:0] paddr,
    input logic [ID_WIDTH-1:0] pid,
    input logic [DATA_WIDTH-1:0] prdata,
    output logic pready,

)