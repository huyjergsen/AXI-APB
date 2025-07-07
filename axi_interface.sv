//lap trinh 2 interface truoc
module axi_interface#(
    parameter  DATA_WIDTH = 32,
    parameter  ADDR_WIDTH = 32,
    parameter  ID_WIDTH = 4,
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

    //AXI WRITE DATA signals
    input logic wvalid,
    output logic wready,
    input logic [DATA_WIDTH-1:0] wdata,     //data
    input logic [DATA_WIDTH/8-1:0] wstrb,   //write strobe, indicate which bytes of the data are valid
    input logic wlast,                      //last transaction in a burst
    input logic [ID_WIDTH-1:0] wid,         //id

    //AXI WRITE RESPONSE signals
    output logic bvalid,
    input logic bready,
    input logic [ID_WIDTH-1:0] bid,         //id
    input logic [1:0] bresp,                //response: 2'b00: OKAY, 2'b01: EXOKAY, 2'b10: SLVERR, 2'b11: DECERR

    //AXI READ ADDRESS signals
    input logic arvalid,
    output logic arready,
    input logic [ADDR_WIDTH-1:0] araddr,
    input logic [ID_WIDTH-1:0] arid,
    input logic [7:0] arlen,
    input logic [1:0] arburst,
    input logic [2:0] arprot,
    input logic [2:0] arsize,
    input logic [1:0] arlock,
    input logic [3:0] arcache,
    input logic [3:0] arqos,
    input logic [3:0] arregion,
    input logic [3:0] arid,
    
    //AXI READ DATA signals
    output logic rvalid,
    input logic rready,
    input logic [ID_WIDTH-1:0] rid,
    input logic [DATA_WIDTH-1:0] rdata,
    input logic rlast,
    input logic [1:0] rresp,
)

//HANDSHAKE WRITE ADDRESS
wire aw_handshake = awvalid && awready;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        awready <= 1'b0;
    end else begin
        if (!awready && awvalid) begin
            awready <= 1'b1;
        end else if (aw_handshake) begin
            awready <= 1'b0;
        end
    end
end

//HANDSHAKE WRITE ADDRESS PROCESSING
always_ff @(posedge clk) begin
    if (aw_handshake) begin
        //
    end
end

//HANDSHAKE WRITE DATA
wire w_handshake = wvalid && wready;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        wready <= 1'b0;
    end else begin
        if (!wready && wvalid) begin
            wready <= 1'b1;
        end else if (w_handshake) begin
            wready <= 1'b0;
        end
    end

end

//HANDSHAKE WRITE DATA PROCESSING
always_ff @(posedge clk) begin
    if (w_handshake) begin
        //
    end
end

//HANDSHAKE WRITE RESPONSE
wire b_handshake = bvalid && bready;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        bvalid <= 1'b0;
    end else begin
        if (!bvalid && bready) begin
            bvalid <= 1'b1;
        end else if (b_handshake) begin
            bvalid <= 1'b0;
        end
    end
end

always_ff @(posedge clk) begin
    if (b_handshake) begin
        //
    end
end

//HANDSHAKE READ ADDRESS
wire a_readadd_handshake = arvalid && arready;
    always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        arready <= 1'b0;
    end else begin
        if (!arready && arvalid) begin
            arready <= 1'b1;
        end else if (a_readadd_handshake) begin
            arready <= 1'b0;
        end
    end
end

always_ff @(posedge clk) begin
    if (a_readadd_handshake) begin
        //
    end
end

//HANDSHAKE READ DATA
wire r_handshake = rvalid && rready;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        rvalid <= 1'b0;
    end else begin
        if (!rvalid && rready) begin
            rvalid <= 1'b1;
        end else if (r_handshake) begin
            rvalid <= 1'b0;
        end
    end
end

always_ff @(posedge clk) begin
    //
    end
end
