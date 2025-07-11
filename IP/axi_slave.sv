module axi_slave#(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH = 4,
    parameter FIFO_DEPTH = 16,
    // Address range parameters
    parameter ADDR_BASE = 32'h1000_0000,
    parameter ADDR_SIZE = 32'h0010_0000  // 1MB address space
)(
    input logic clk,
    input logic reset,
    
    // AXI4 Slave Interface - Write Address Channel
    input  logic awvalid,
    output logic awready,
    input  logic [ADDR_WIDTH-1:0] awaddr,
    input  logic [ID_WIDTH-1:0] awid,
    input  logic [7:0] awlen,
    input  logic [1:0] awburst,
    input  logic [2:0] awprot,
    input  logic [2:0] awsize,
    input  logic awlock,
    input  logic [3:0] awcache,
    input  logic [3:0] awqos,
    input  logic [3:0] awregion,
    
    // AXI4 Slave Interface - Write Data Channel
    input  logic wvalid,
    output logic wready,
    input  logic [DATA_WIDTH-1:0] wdata,
    input  logic [DATA_WIDTH/8-1:0] wstrb,
    input  logic wlast,
    input  logic [ID_WIDTH-1:0] wid,
    
    // AXI4 Slave Interface - Write Response Channel
    output logic bvalid,
    input  logic bready,
    output logic [ID_WIDTH-1:0] bid,
    output logic [1:0] bresp,
    
    // AXI4 Slave Interface - Read Address Channel
    input  logic arvalid,
    output logic arready,
    input  logic [ADDR_WIDTH-1:0] araddr,
    input  logic [ID_WIDTH-1:0] arid,
    input  logic [7:0] arlen,
    input  logic [1:0] arburst,
    input  logic [2:0] arprot,
    input  logic [2:0] arsize,
    input  logic arlock,
    input  logic [3:0] arcache,
    input  logic [3:0] arqos,
    input  logic [3:0] arregion,
    
    // AXI4 Slave Interface - Read Data Channel
    output logic rvalid,
    input  logic rready,
    output logic [ID_WIDTH-1:0] rid,
    output logic [DATA_WIDTH-1:0] rdata,
    output logic rlast,
    output logic [1:0] rresp,
    
    // APB4 Master Interface
    output logic [ADDR_WIDTH-1:0] paddr,
    output logic pwrite,
    output logic psel,
    output logic penable,
    output logic [DATA_WIDTH-1:0] pwdata,
    output logic [3:0] pstrb,
    output logic [2:0] pprot,
    input  logic [DATA_WIDTH-1:0] prdata,
    input  logic pready,
    input  logic pslverr,
    
    // Additional control and status
    output logic irq,
    input  logic debug_req,
    output logic bridge_active,
    output logic [31:0] error_count,
    output logic [31:0] transaction_count
);

    // Internal signals for core connection
    logic core_awvalid, core_awready;
    logic [ADDR_WIDTH-1:0] core_awaddr;
    logic [ID_WIDTH-1:0] core_awid;
    logic [7:0] core_awlen;
    logic [1:0] core_awburst;
    logic [2:0] core_awprot, core_awsize;
    logic core_awlock;
    logic [3:0] core_awcache, core_awqos, core_awregion;
    
    logic core_arvalid, core_arready;
    logic [ADDR_WIDTH-1:0] core_araddr;
    logic [ID_WIDTH-1:0] core_arid;
    logic [7:0] core_arlen;
    logic [1:0] core_arburst;
    logic [2:0] core_arprot, core_arsize;
    logic core_arlock;
    logic [3:0] core_arcache, core_arqos, core_arregion;
    
    logic core_wvalid, core_wready;
    logic [DATA_WIDTH-1:0] core_wdata;
    logic [DATA_WIDTH/8-1:0] core_wstrb;
    logic core_wlast;
    logic [ID_WIDTH-1:0] core_wid;
    
    logic core_rvalid, core_rready;
    logic [ID_WIDTH-1:0] core_rid;
    logic [DATA_WIDTH-1:0] core_rdata;
    logic core_rlast;
    logic [1:0] core_rresp;
    
    logic core_bvalid, core_bready;
    logic [ID_WIDTH-1:0] core_bid;
    logic [1:0] core_bresp;

    // Address range checking
    logic aw_addr_valid, ar_addr_valid;
    logic aw_addr_error, ar_addr_error;
    
    assign aw_addr_valid = (awaddr >= ADDR_BASE) && (awaddr < (ADDR_BASE + ADDR_SIZE));
    assign ar_addr_valid = (araddr >= ADDR_BASE) && (araddr < (ADDR_BASE + ADDR_SIZE));
    assign aw_addr_error = awvalid && !aw_addr_valid;
    assign ar_addr_error = arvalid && !ar_addr_valid;
    
    // Counters
    logic [31:0] error_count_reg;
    logic [31:0] transaction_count_reg;
    logic [31:0] write_count_reg;
    logic [31:0] read_count_reg;
    
    // Transaction counting
    wire aw_handshake = awvalid && awready;
    wire ar_handshake = arvalid && arready;
    wire w_handshake = wvalid && wready && wlast;
    wire r_handshake = rvalid && rready && rlast;
    wire b_handshake = bvalid && bready;
    
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            error_count_reg <= '0;
            transaction_count_reg <= '0;
            write_count_reg <= '0;
            read_count_reg <= '0;
        end else begin
            // Count errors
            if (aw_addr_error || ar_addr_error || (core_bvalid && core_bresp != 2'b00) || 
                (core_rvalid && core_rresp != 2'b00)) begin
                error_count_reg <= error_count_reg + 1;
            end
            
            // Count transactions
            if (aw_handshake || ar_handshake) begin
                transaction_count_reg <= transaction_count_reg + 1;
            end
            
            // Count writes and reads
            if (b_handshake) write_count_reg <= write_count_reg + 1;
            if (r_handshake) read_count_reg <= read_count_reg + 1;
        end
    end

    
    // Write Address Channel
    assign core_awvalid = awvalid && aw_addr_valid;
    assign awready = aw_addr_valid ? core_awready : 1'b1;
    assign core_awaddr = awaddr;
    assign core_awid = awid;
    assign core_awlen = awlen;
    assign core_awburst = awburst;
    assign core_awprot = awprot;
    assign core_awsize = awsize;
    assign core_awlock = awlock;
    assign core_awcache = awcache;
    assign core_awqos = awqos;
    assign core_awregion = awregion;
    
    // Read Address Channel
    assign core_arvalid = arvalid && ar_addr_valid;
    assign arready = ar_addr_valid ? core_arready : 1'b1; 
    assign core_araddr = araddr;
    assign core_arid = arid;
    assign core_arlen = arlen;
    assign core_arburst = arburst;
    assign core_arprot = arprot;
    assign core_arsize = arsize;
    assign core_arlock = arlock;
    assign core_arcache = arcache;
    assign core_arqos = arqos;
    assign core_arregion = arregion;
    
    // Write Data Channel - pass if address was valid
    assign core_wvalid = wvalid && aw_addr_valid;
    assign wready = aw_addr_valid ? core_wready : 1'b1;
    assign core_wdata = wdata;
    assign core_wstrb = wstrb;
    assign core_wlast = wlast;
    assign core_wid = wid;
    
    // Read Data Channel
    assign rvalid = core_rvalid;
    assign core_rready = rready;
    assign rid = core_rid;
    assign rdata = core_rdata;
    assign rlast = core_rlast;
    assign rresp = core_rresp;
    
    // Write Response Channel  
    assign bvalid = core_bvalid;
    assign core_bready = bready;
    assign bid = core_bid;
    assign bresp = core_bresp;

    // Error response for invalid addresses
    logic error_aw_pending, error_ar_pending;
    logic [ID_WIDTH-1:0] error_aw_id, error_ar_id;
    
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            error_aw_pending <= 1'b0;
            error_ar_pending <= 1'b0;
            error_aw_id <= '0;
            error_ar_id <= '0;
        end else begin
            // Track invalid write addresses
            if (aw_addr_error && awready) begin
                error_aw_pending <= 1'b1;
                error_aw_id <= awid;
            end else if (error_aw_pending && bready && !core_bvalid) begin
                error_aw_pending <= 1'b0;
            end
            
            // Track invalid read addresses  
            if (ar_addr_error && arready) begin
                error_ar_pending <= 1'b1;
                error_ar_id <= arid;
            end else if (error_ar_pending && rready && !core_rvalid) begin
                error_ar_pending <= 1'b0;
            end
        end
    end
    
    // Bridge status
    assign bridge_active = core_awvalid || core_arvalid || core_wvalid || 
                          core_rvalid || core_bvalid || 
                          (transaction_count_reg > 0);
    
    // Output statistics
    assign error_count = error_count_reg;
    assign transaction_count = transaction_count_reg;

    // Instantiate the core bridge module
    core #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH),
        .ID_WIDTH(ID_WIDTH)
    ) u_core (
        .clk(clk),
        .reset(reset),
        
        // AXI4 Slave Interface
        .awvalid(core_awvalid), .awready(core_awready), 
        .awaddr(core_awaddr), .awsize(core_awsize),
        .awlen(core_awlen), .awburst(core_awburst), 
        .awid(core_awid), .awprot(core_awprot),
        .awlock(core_awlock), .awcache(core_awcache), 
        .awqos(core_awqos), .awregion(core_awregion),
        
        .arvalid(core_arvalid), .arready(core_arready), 
        .araddr(core_araddr), .arsize(core_arsize),
        .arlen(core_arlen), .arburst(core_arburst), 
        .arid(core_arid), .arprot(core_arprot),
        .arlock(core_arlock), .arcache(core_arcache), 
        .arqos(core_arqos), .arregion(core_arregion),
        
        .wvalid(core_wvalid), .wready(core_wready), 
        .wdata(core_wdata), .wstrb(core_wstrb),
        .wlast(core_wlast), .wid(core_wid),
        
        .rvalid(core_rvalid), .rready(core_rready), 
        .rdata(core_rdata), .rlast(core_rlast),
        .rresp(core_rresp), .rid(core_rid),
        
        .bvalid(core_bvalid), .bready(core_bready), 
        .bresp(core_bresp), .bid(core_bid),
        
        // APB4 Master Interface
        .paddr(paddr), .pwrite(pwrite), .psel(psel), 
        .penable(penable), .pwdata(pwdata), .pstrb(pstrb), 
        .pprot(pprot), .prdata(prdata), .pready(pready), 
        .pslverr(pslverr),
        
        // Control signals
        .irq(irq), .debug_req(debug_req)
    );

endmodule
