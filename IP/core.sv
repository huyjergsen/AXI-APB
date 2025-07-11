module core(
    // Clock and reset
    input  logic        clk,
    input  logic        reset,
    
    // AXI4 Slave Interface - Write Address Channel
    input  logic        awvalid,
    output logic        awready,
    input  logic [31:0] awaddr,
    input  logic [2:0]  awsize,
    input  logic [7:0]  awlen,
    input  logic [1:0]  awburst,
    input  logic [3:0]  awid,
    input  logic [2:0]  awprot,
    input  logic        awlock,
    input  logic [3:0]  awcache,
    input  logic [3:0]  awqos,
    input  logic [3:0]  awregion,
    
    // AXI4 Slave Interface - Read Address Channel
    input  logic        arvalid,
    output logic        arready,
    input  logic [31:0] araddr,
    input  logic [2:0]  arsize,
    input  logic [7:0]  arlen,
    input  logic [1:0]  arburst,
    input  logic [3:0]  arid,
    input  logic [2:0]  arprot,
    input  logic        arlock,
    input  logic [3:0]  arcache,
    input  logic [3:0]  arqos,
    input  logic [3:0]  arregion,
    
    // AXI4 Slave Interface - Write Data Channel
    input  logic        wvalid,
    output logic        wready,
    input  logic [31:0] wdata,
    input  logic [3:0]  wstrb,
    input  logic        wlast,
    input  logic [3:0]  wid,
    
    // AXI4 Slave Interface - Read Data Channel
    output logic        rvalid,
    input  logic        rready,
    output logic        rlast,
    output logic [1:0]  rresp,
    output logic [3:0]  rid,
    output logic [31:0] rdata,
    
    // AXI4 Slave Interface - Write Response Channel
    output logic        bvalid,
    input  logic        bready,
    output logic [1:0]  bresp,
    output logic [3:0]  bid,
    
    // APB4 Master Interface
    output logic [31:0] paddr,
    output logic        pwrite,
    output logic        psel,
    output logic        penable,
    output logic [31:0] pwdata,
    output logic [3:0]  pstrb,
    output logic [2:0]  pprot,
    input  logic [31:0] prdata,
    input  logic        pready,
    input  logic        pslverr,
    
    // Optional signals
    output logic        irq,
    input  logic        debug_req
);

    // Parameters
    parameter DATA_WIDTH = 32;
    parameter ADDR_WIDTH = 32;
    parameter FIFO_DEPTH = 16;
    parameter ID_WIDTH = 4;

    // Internal signals - AXI to FIFO
    logic axi_cmd_valid, axi_cmd_ready;
    logic [31:0] axi_cmd_addr;
    logic axi_cmd_write;
    logic [3:0] axi_cmd_id;
    logic [7:0] axi_cmd_len;
    logic [1:0] axi_cmd_burst;
    logic [2:0] axi_cmd_size;
    logic [2:0] axi_cmd_prot;
    
    logic axi_wdata_valid, axi_wdata_ready;
    logic [31:0] axi_wdata_data;
    logic [3:0] axi_wdata_strb;
    logic axi_wdata_last;
    logic [3:0] axi_wdata_id;
    
    logic axi_rdata_valid, axi_rdata_ready;
    logic [31:0] axi_rdata_data;
    logic [3:0] axi_rdata_id;
    logic axi_rdata_last;
    logic [1:0] axi_rdata_resp;
    
    logic axi_bresp_valid, axi_bresp_ready;
    logic [3:0] axi_bresp_id;
    logic [1:0] axi_bresp_resp;

    // FIFO signals - Request FIFO
    logic req_fifo_wr_en, req_fifo_rd_en;
    logic req_fifo_full, req_fifo_empty;
    logic [31+1+4+8+2+3+3-1:0] req_fifo_wr_data, req_fifo_rd_data; // addr+write+id+len+burst+size+prot
    
    // FIFO signals - Write Data FIFO
    logic wdata_fifo_wr_en, wdata_fifo_rd_en;
    logic wdata_fifo_full, wdata_fifo_empty;
    logic [31+4+1+4-1:0] wdata_fifo_wr_data, wdata_fifo_rd_data; // data+strb+last+id
    
    // FIFO signals - Read Data FIFO
    logic rdata_fifo_wr_en, rdata_fifo_rd_en;
    logic rdata_fifo_full, rdata_fifo_empty;
    logic [31+4+1+2-1:0] rdata_fifo_wr_data, rdata_fifo_rd_data; // data+id+last+resp
    
    // FSM to APB signals
    logic fsm_psel, fsm_penable, fsm_pwrite;
    logic [31:0] fsm_paddr, fsm_pwdata;
    logic [3:0] fsm_pstrb;
    logic [2:0] fsm_pprot;

    // Instantiate AXI Interface
    axi_interface #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH(ID_WIDTH)
    ) u_axi_interface (
        .clk(clk),
        .reset(reset),
        
        // AXI Slave ports
        .awvalid(awvalid), .awready(awready), .awaddr(awaddr), .awsize(awsize),
        .awlen(awlen), .awburst(awburst), .awid(awid), .awprot(awprot),
        .awlock(awlock), .awcache(awcache), .awqos(awqos), .awregion(awregion),
        
        .arvalid(arvalid), .arready(arready), .araddr(araddr), .arsize(arsize),
        .arlen(arlen), .arburst(arburst), .arid(arid), .arprot(arprot),
        .arlock(arlock), .arcache(arcache), .arqos(arqos), .arregion(arregion),
        
        .wvalid(wvalid), .wready(wready), .wdata(wdata), .wstrb(wstrb),
        .wlast(wlast), .wid(wid),
        
        .rvalid(rvalid), .rready(rready), .rdata(rdata), .rlast(rlast),
        .rresp(rresp), .rid(rid),
        
        .bvalid(bvalid), .bready(bready), .bresp(bresp), .bid(bid),
        
        // Internal command interface
        .cmd_valid(axi_cmd_valid), .cmd_ready(axi_cmd_ready),
        .cmd_addr(axi_cmd_addr), .cmd_write(axi_cmd_write), .cmd_id(axi_cmd_id),
        .cmd_len(axi_cmd_len), .cmd_burst(axi_cmd_burst), .cmd_size(axi_cmd_size),
        .cmd_prot(axi_cmd_prot),
        
        // Internal write data interface
        .wdata_valid(axi_wdata_valid), .wdata_ready(axi_wdata_ready),
        .wdata_data(axi_wdata_data), .wdata_strb(axi_wdata_strb),
        .wdata_last(axi_wdata_last), .wdata_id(axi_wdata_id),
        
        // Internal read data interface  
        .rdata_valid(axi_rdata_valid), .rdata_ready(axi_rdata_ready),
        .rdata_data(axi_rdata_data), .rdata_id(axi_rdata_id),
        .rdata_last(axi_rdata_last), .rdata_resp(axi_rdata_resp),
        
        // Internal write response interface
        .bresp_valid(axi_bresp_valid), .bresp_ready(axi_bresp_ready),
        .bresp_id(axi_bresp_id), .bresp_resp(axi_bresp_resp)
    );

    // Pack data for FIFOs
    assign req_fifo_wr_data = {axi_cmd_addr, axi_cmd_write, axi_cmd_id, axi_cmd_len, 
                               axi_cmd_burst, axi_cmd_size, axi_cmd_prot};
    assign wdata_fifo_wr_data = {axi_wdata_data, axi_wdata_strb, axi_wdata_last, axi_wdata_id};
    
    // FIFO control signals
    assign req_fifo_wr_en = axi_cmd_valid && axi_cmd_ready;
    assign wdata_fifo_wr_en = axi_wdata_valid && axi_wdata_ready;
    assign axi_cmd_ready = !req_fifo_full;
    assign axi_wdata_ready = !wdata_fifo_full;

    // Instantiate Request FIFO
    fifo_request #(
        .DATA_WIDTH($bits(req_fifo_wr_data)),
        .DEPTH(FIFO_DEPTH)
    ) u_req_fifo (
        .clk(clk),
        .reset(reset),
        .wr_en(req_fifo_wr_en),
        .wr_data(req_fifo_wr_data),
        .rd_en(req_fifo_rd_en),
        .rd_data(req_fifo_rd_data),
        .full(req_fifo_full),
        .empty(req_fifo_empty)
    );

    // Instantiate Write Data FIFO
    fifo_write #(
        .DATA_WIDTH($bits(wdata_fifo_wr_data)),
        .DEPTH(FIFO_DEPTH)
    ) u_wdata_fifo (
        .clk(clk),
        .reset(reset),
        .wr_en(wdata_fifo_wr_en),
        .wr_data(wdata_fifo_wr_data),
        .rd_en(wdata_fifo_rd_en),
        .rd_data(wdata_fifo_rd_data),
        .full(wdata_fifo_full),
        .empty(wdata_fifo_empty)
    );

    // Instantiate Read Data FIFO
    fifo_read #(
        .DATA_WIDTH($bits(rdata_fifo_wr_data)),
        .DEPTH(FIFO_DEPTH)
    ) u_rdata_fifo (
        .clk(clk),
        .reset(reset),
        .wr_en(rdata_fifo_wr_en),
        .wr_data(rdata_fifo_wr_data),
        .rd_en(rdata_fifo_rd_en),
        .rd_data(rdata_fifo_rd_data),
        .full(rdata_fifo_full),
        .empty(rdata_fifo_empty)
    );

    // Unpack read data from FIFO to AXI
    assign {axi_rdata_data, axi_rdata_id, axi_rdata_last, axi_rdata_resp} = rdata_fifo_rd_data;
    assign axi_rdata_valid = !rdata_fifo_empty;
    assign rdata_fifo_rd_en = axi_rdata_valid && axi_rdata_ready;

    // Instantiate FSM
    fsm #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH(ID_WIDTH)
    ) u_fsm (
        .clk(clk),
        .reset(reset),
        
        // Request FIFO interface
        .req_fifo_empty(req_fifo_empty),
        .req_fifo_rd_en(req_fifo_rd_en),
        .req_fifo_rd_data(req_fifo_rd_data),
        
        // Write Data FIFO interface
        .wdata_fifo_empty(wdata_fifo_empty),
        .wdata_fifo_rd_en(wdata_fifo_rd_en),
        .wdata_fifo_rd_data(wdata_fifo_rd_data),
        
        // Read Data FIFO interface
        .rdata_fifo_full(rdata_fifo_full),
        .rdata_fifo_wr_en(rdata_fifo_wr_en),
        .rdata_fifo_wr_data(rdata_fifo_wr_data),
        
        // Write Response interface
        .bresp_valid(axi_bresp_valid),
        .bresp_ready(axi_bresp_ready),
        .bresp_id(axi_bresp_id),
        .bresp_resp(axi_bresp_resp),
        
        // APB Master interface
        .apb_psel(fsm_psel),
        .apb_penable(fsm_penable),
        .apb_pwrite(fsm_pwrite),
        .apb_paddr(fsm_paddr),
        .apb_pwdata(fsm_pwdata),
        .apb_pstrb(fsm_pstrb),
        .apb_pprot(fsm_pprot),
        .apb_prdata(prdata),
        .apb_pready(pready),
        .apb_pslverr(pslverr)
    );

    // Instantiate APB Interface
    apb_interface u_apb_interface (
        .clk(clk),
        .reset(reset),
        
        // Input from FSM
        .fsm_psel(fsm_psel),
        .fsm_penable(fsm_penable),
        .fsm_pwrite(fsm_pwrite),
        .fsm_paddr(fsm_paddr),
        .fsm_pwdata(fsm_pwdata),
        .fsm_pstrb(fsm_pstrb),
        .fsm_pprot(fsm_pprot),
        
        // Output to APB Bus
        .psel(psel),
        .penable(penable),
        .pwrite(pwrite),
        .paddr(paddr),
        .pwdata(pwdata),
        .pstrb(pstrb),
        .pprot(pprot)
    );

    // IRQ generation 
    assign irq = 1'b0; // Can be enhanced later

endmodule
        

