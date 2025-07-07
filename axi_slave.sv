module axi_slave#(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH = 4,
)(
    input logic clk,
    input logic reset,
    //AXI WRITE ADDRESS signals
    input logic awvalid, //data valid
    output logic awready, //data ready
    input logic [ADDR_WIDTH-1:0] awaddr,    //data address
    input logic [ID_WIDTH-1:0] awid,        //data id
    input logic [7:0] awlen,                //data length in 1 burst
    input logic [1:0] awburst,              //burst type: 2'b00: FIXED, 2'b01: INCR, 2'b10: WRAP
    input logic [2:0] awprot,               //protection type: 3'b000: UNPRIV, 3'b001: PRIV, 3'b010: SECURE, 3'b011: SECURE_UNPRIV
    input logic [2:0] awsize,               //data size: 3'b000: 8bit, 3'b001: 16bit, 3'b010: 32bit, 3'b011: 64bit
    input logic [1:0] awlock,               //lock type: 2'b00: NORMAL, 2'b01: EXCLUSIVE
    input logic [3:0] awcache,              //cache type: 4'b0000: CACHEABLE, 4'b0001: UNCACHED, 4'b0010: BUFFERABLE, 4'b0011: UNCACHED_BUFFERABLE
    input logic [3:0] awqos,                //quality of service, 4'b0000: LOW_LATENCY, 4'b0001: MEDIUM_LATENCY, 4'b0010: HIGH_LATENCY, 4'b0011: HIGHEST_LATENCY
    input logic [3:0] awregion,             //region, 4'b0000: REGION_0, 4'b0001: REGION_1, 4'b0010: REGION_2, 4'b0011: REGION_3
    input logic [3:0] awid,                 //id, 4'b0000: ID_0, 4'b0001: ID_1, 4'b0010: ID_2, 4'b0011: ID_3
)
    //AXI WRITE DATA signals
    input logic wvalid,