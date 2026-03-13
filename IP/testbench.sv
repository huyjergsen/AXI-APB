`timescale 1ns / 1ps

// =============================================================================
// Module      : testbench
// Project     : AXI-to-APB Bridge IP
// Description : Self-checking testbench for the AXI4-to-APB4 bridge.
//               Covers single write/read, burst write/read, and error-response
//               tests for out-of-range addresses.
// =============================================================================

module testbench;

// =============================================================================
// Parameters
// =============================================================================
parameter DATA_WIDTH = 32;
parameter ADDR_WIDTH = 32;
parameter ID_WIDTH   = 4;
parameter CLK_PERIOD = 10;                // 100 MHz clock
parameter ADDR_BASE  = 32'h1000_0000;

// =============================================================================
// Clock & Reset
// =============================================================================
logic clk;
logic reset;

// =============================================================================
// AXI4 Slave Interface Signals
// =============================================================================

// Write Address Channel (AW)
logic                    awvalid, awready;
logic [ADDR_WIDTH-1:0]   awaddr;
logic [2:0]              awsize;
logic [7:0]              awlen;
logic [1:0]              awburst;
logic [ID_WIDTH-1:0]     awid;
logic [2:0]              awprot;
logic                    awlock;
logic [3:0]              awcache, awqos, awregion;

// Read Address Channel (AR)
logic                    arvalid, arready;
logic [ADDR_WIDTH-1:0]   araddr;
logic [2:0]              arsize;
logic [7:0]              arlen;
logic [1:0]              arburst;
logic [ID_WIDTH-1:0]     arid;
logic [2:0]              arprot;
logic                    arlock;
logic [3:0]              arcache, arqos, arregion;

// Write Data Channel (W)
logic                    wvalid, wready;
logic [DATA_WIDTH-1:0]   wdata;
logic [DATA_WIDTH/8-1:0] wstrb;
logic                    wlast;
logic [ID_WIDTH-1:0]     wid;

// Read Data Channel (R)
logic                    rvalid, rready;
logic [DATA_WIDTH-1:0]   rdata;
logic                    rlast;
logic [1:0]              rresp;
logic [ID_WIDTH-1:0]     rid;

// Write Response Channel (B)
logic        bvalid, bready;
logic [1:0]  bresp;
logic [ID_WIDTH-1:0] bid;

// =============================================================================
// APB4 Interface Signals
// =============================================================================

// Bridge master outputs
logic [ADDR_WIDTH-1:0]   paddr;
logic                    pwrite;
logic                    psel;
logic                    penable;
logic [DATA_WIDTH-1:0]   pwdata;
logic [DATA_WIDTH/8-1:0] pstrb;
logic [2:0]              pprot;

// Dummy slave inputs
logic [DATA_WIDTH-1:0]   prdata;
logic                    pready;
logic                    pslverr;

// =============================================================================
// Miscellaneous Signals
// =============================================================================
logic        irq;
logic        debug_req;
logic        bridge_active;
logic [31:0] error_count_out;
logic [31:0] transaction_count_out;

// =============================================================================
// Test Bookkeeping
// =============================================================================
integer test_count  = 0;
integer error_count = 0;
logic [DATA_WIDTH-1:0] read_data;

// =============================================================================
// Clock Generation
// =============================================================================
initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// =============================================================================
// DUT Instantiation
// =============================================================================
axi_slave #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .ID_WIDTH  (ID_WIDTH),
    .ADDR_BASE (ADDR_BASE)
) dut (
    .clk   (clk),
    .reset (reset),

    // Write Address
    .awvalid(awvalid), .awready(awready), .awaddr(awaddr),   .awsize(awsize),
    .awlen  (awlen),   .awburst(awburst), .awid  (awid),     .awprot(awprot),
    .awlock (awlock),  .awcache(awcache), .awqos (awqos),    .awregion(awregion),

    // Read Address
    .arvalid(arvalid), .arready(arready), .araddr(araddr),   .arsize(arsize),
    .arlen  (arlen),   .arburst(arburst), .arid  (arid),     .arprot(arprot),
    .arlock (arlock),  .arcache(arcache), .arqos (arqos),    .arregion(arregion),

    // Write Data
    .wvalid(wvalid), .wready(wready), .wdata(wdata), .wstrb(wstrb),
    .wlast (wlast),  .wid  (wid),

    // Read Data
    .rvalid(rvalid), .rready(rready), .rdata(rdata), .rlast(rlast),
    .rresp (rresp),  .rid  (rid),

    // Write Response
    .bvalid(bvalid), .bready(bready), .bresp(bresp), .bid(bid),

    // APB Master
    .paddr  (paddr),   .pwrite  (pwrite),   .psel    (psel),    .penable(penable),
    .pwdata (pwdata),  .pstrb   (pstrb),    .pprot   (pprot),
    .prdata (prdata),  .pready  (pready),   .pslverr (pslverr),

    // Control / Status
    .irq              (irq),
    .debug_req        (debug_req),
    .bridge_active    (bridge_active),
    .error_count      (error_count_out),
    .transaction_count(transaction_count_out)
);

// =============================================================================
// APB Slave Simulation Model  (4 KB word-addressed memory)
// =============================================================================
logic [DATA_WIDTH-1:0] apb_memory [1024];

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        pready  <= 1'b0;
        pslverr <= 1'b0;
        prdata  <= '0;
    end else begin
        pready  <= 1'b0;
        pslverr <= 1'b0;

        if (psel && penable) begin
            pready <= 1'b1; // Zero-wait-state response

            if (pwrite) begin
                apb_memory[paddr[11:2]] <= pwdata;
                $display("[APB SLAVE] WR Addr=0x%08x  Data=0x%08x", paddr, pwdata);
            end else begin
                prdata <= apb_memory[paddr[11:2]];
                $display("[APB SLAVE] RD Addr=0x%08x  Data=0x%08x", paddr, apb_memory[paddr[11:2]]);
            end
        end
    end
end

// =============================================================================
// Task: init_signals - De-assert all AXI master signals
// =============================================================================
task init_signals();
    awvalid = 0; awaddr = 0; awsize = 0; awlen = 0; awburst = 0;
    awid    = 0; awprot = 0; awlock = 0; awcache = 0; awqos = 0; awregion = 0;

    arvalid = 0; araddr = 0; arsize = 0; arlen = 0; arburst = 0;
    arid    = 0; arprot = 0; arlock = 0; arcache = 0; arqos = 0; arregion = 0;

    wvalid = 0; wdata = 0; wstrb = 0; wlast = 0; wid = 0;

    rready    = 1; // Always ready to receive read data
    bready    = 1; // Always ready to receive write response
    debug_req = 0;
endtask

// =============================================================================
// Task: reset_dut - Apply synchronous reset
// =============================================================================
task reset_dut();
    reset = 1;
    init_signals();
    repeat(20) @(posedge clk); // 200 ns > GSR period for post-synthesis sim
    reset = 0;
    repeat(5)  @(posedge clk); // Settling time
    $display("--- Reset completed ---");
endtask

// =============================================================================
// Task: axi_write - Single-beat AXI write transaction
// =============================================================================
task axi_write(
    input [ADDR_WIDTH-1:0] addr,
    input [DATA_WIDTH-1:0] data,
    input [ID_WIDTH-1:0]   id
);
    logic [1:0]        cap_bresp;
    logic [ID_WIDTH-1:0] cap_bid;
    begin
        test_count++;
        $display("Test %0d: AXI Single Write  Addr=0x%08x  Data=0x%08x", test_count, addr, data);

        // Fork: send AW and W channels concurrently
        fork
            begin // Write Address
                @(negedge clk);
                awvalid = 1; awaddr = addr;
                awsize = 3'b010; awlen = 8'h00; awburst = 2'b01; awid = id;
                while(1) begin @(posedge clk); if(awready) break; end
                @(negedge clk);
                awvalid = 0;
            end

            begin // Write Data
                @(negedge clk);
                wvalid = 1; wdata = data; wstrb = 4'hF; wlast = 1; wid = id;
                while(1) begin @(posedge clk); if(wready) break; end
                @(negedge clk);
                wvalid = 0; wlast = 0;
            end
        join

        // Wait for write response
        while(1) begin
            @(posedge clk);
            if(bvalid && bready) begin
                cap_bresp = bresp;
                cap_bid   = bid;
                break;
            end
        end

        // Wait for bvalid de-assertion before next test
        while(bvalid) @(posedge clk);

        if (cap_bresp == 2'b00 && cap_bid == id)
            $display("--> Write OK");
        else begin
            $display("--> Write FAILED  bresp=%b", cap_bresp);
            error_count++;
        end
    end
endtask

// =============================================================================
// Task: axi_write_burst - Multi-beat AXI INCR burst write
// =============================================================================
task axi_write_burst(
    input [ADDR_WIDTH-1:0] addr,
    input [DATA_WIDTH-1:0] start_data,
    input [7:0]            len,  // Beats - 1 (AXI encoding)
    input [ID_WIDTH-1:0]   id
);
    integer i;
    logic [1:0] cap_bresp;
    begin
        test_count++;
        $display("Test %0d: AXI Burst Write  Beats=%0d  StartAddr=0x%08x", test_count, len+1, addr);

        fork
            begin // Write Address
                @(negedge clk);
                awvalid = 1; awaddr = addr;
                awsize = 3'b010; awlen = len; awburst = 2'b01; awid = id;
                while(1) begin @(posedge clk); if(awready) break; end
                @(negedge clk);
                awvalid = 0;
            end

            begin // Write Data beats
                for (i = 0; i <= len; i++) begin
                    @(negedge clk);
                    wvalid = 1;
                    wdata  = start_data + i; // Incrementing data pattern
                    wstrb  = 4'hF;
                    wlast  = (i == len);
                    wid    = id;
                    while(1) begin @(posedge clk); if(wready) break; end
                    if (i == len) begin
                        @(negedge clk);
                        wvalid = 0; wlast = 0;
                    end
                end
            end
        join

        // Wait for write response
        while(1) begin
            @(posedge clk);
            if(bvalid && bready) begin
                cap_bresp = bresp;
                break;
            end
        end

        @(negedge clk);
        if (cap_bresp == 2'b00)
            $display("--> Burst Write OK");
        else begin
            $display("--> Burst Write FAILED  bresp=%b", cap_bresp);
            error_count++;
        end
    end
endtask

// =============================================================================
// Task: axi_read - Single-beat AXI read transaction
// =============================================================================
task axi_read(
    input  [ADDR_WIDTH-1:0] addr,
    input  [ID_WIDTH-1:0]   id,
    output [DATA_WIDTH-1:0] read_data_out
);
    logic [1:0] cap_rresp;
    logic       cap_rlast;
    begin
        test_count++;
        $display("Test %0d: AXI Single Read  Addr=0x%08x", test_count, addr);

        // Send Read Address
        @(negedge clk);
        arvalid = 1; araddr = addr;
        arsize = 3'b010; arlen = 8'h00; arburst = 2'b01; arid = id;
        while(1) begin @(posedge clk); if(arready) break; end
        @(negedge clk);
        arvalid = 0;

        // Receive Read Data
        while(1) begin
            @(posedge clk);
            if(rvalid && rready) begin
                read_data_out = rdata;
                cap_rresp     = rresp;
                cap_rlast     = rlast;
                break;
            end
        end

        // Wait for rvalid de-assertion before next test
        while(rvalid) @(posedge clk);

        if (cap_rresp == 2'b00 && cap_rlast)
            $display("--> Read OK  Data=0x%08x", read_data_out);
        else begin
            $display("--> Read FAILED  rresp=%b", cap_rresp);
            error_count++;
        end
    end
endtask

// =============================================================================
// Main Test Process
// =============================================================================
initial begin
    $display("\n===========================================");
    $display(" STARTING AXI TO APB BRIDGE VERIFICATION ");
    $display("===========================================\n");

    reset_dut();

    // -----------------------------------------------------------------
    // TEST 1: Single Write & Read Verification
    // -----------------------------------------------------------------
    axi_write(ADDR_BASE + 32'h10, 32'hAAAA_5555, 4'h1);
    repeat(5) @(posedge clk);

    axi_read(ADDR_BASE + 32'h10, 4'h1, read_data);
    if (read_data !== 32'hAAAA_5555) begin
        $display("ERROR: Data Mismatch! Exp=AAAA_5555  Got=0x%08x", read_data);
        error_count++;
    end

    repeat(10) @(posedge clk);

    // -----------------------------------------------------------------
    // TEST 2: Burst Write (4 beats, starting at offset 0x20)
    //         Data pattern: 0x10, 0x11, 0x12, 0x13
    // -----------------------------------------------------------------
    axi_write_burst(ADDR_BASE + 32'h20, 32'h0000_0010, 8'h03, 4'h2);
    repeat(20) @(posedge clk);

    // -----------------------------------------------------------------
    // TEST 3: Verify Burst Data with Single Reads
    // -----------------------------------------------------------------
    axi_read(ADDR_BASE + 32'h20, 4'h3, read_data);
    if (read_data !== 32'h10) begin $display("ERROR Burst[0]"); error_count++; end

    axi_read(ADDR_BASE + 32'h24, 4'h3, read_data);
    if (read_data !== 32'h11) begin $display("ERROR Burst[1]"); error_count++; end

    axi_read(ADDR_BASE + 32'h2C, 4'h3, read_data);
    if (read_data !== 32'h13) begin $display("ERROR Burst[3]"); error_count++; end

    repeat(10) @(posedge clk);

    // -----------------------------------------------------------------
    // TEST 4: Error Handling - Out-of-Range Write (expect SLVERR)
    // -----------------------------------------------------------------
    $display("Test: Out-of-range Write  Addr=0xE000_0000  (Expect SLVERR)");
    axi_write_burst(32'hE000_0000, 32'hBAD_0000, 8'h03, 4'h4);
    repeat(10) @(posedge clk);

    // -----------------------------------------------------------------
    // TEST 5: Error Handling - Out-of-Range Burst Read (expect SLVERR)
    //         Bridge must return 0 data + SLVERR for all beats
    // -----------------------------------------------------------------
    $display("Test: Out-of-range Burst Read  Addr=0xE000_0000  (Expect SLVERR x3)");
    test_count++;
    fork
        begin // Send Read Address
            @(negedge clk);
            arvalid = 1; araddr = 32'hE000_0000;
            arsize = 3'b010; arlen = 8'h02; arburst = 2'b01; arid = 4'h5;
            while(1) begin @(posedge clk); if(arready) break; end
            @(negedge clk);
            arvalid = 0;
        end

        begin // Receive and check all 3 error beats
            integer rx_count;
            logic [1:0]        cap_rresp;
            logic [DATA_WIDTH-1:0] cap_rdata;
            rx_count = 0;
            while (rx_count < 3) begin
                while(1) begin
                    @(posedge clk);
                    if(rvalid && rready) begin
                        cap_rresp = rresp;
                        cap_rdata = rdata;
                        break;
                    end
                end

                if (cap_rresp !== 2'b10) begin
                    $display("ERROR: Expected SLVERR, got rresp=%b", cap_rresp);
                    error_count++;
                end else
                    $display("INFO: Beat[%0d] returned SLVERR as expected", rx_count);

                if (cap_rdata !== 0) begin
                    $display("ERROR: Expected rdata=0, got 0x%08x", cap_rdata);
                    error_count++;
                end

                rx_count++;
            end
            $display("--> Burst Error Read OK");
        end
    join

    repeat(10) @(posedge clk);

    // -----------------------------------------------------------------
    // SUMMARY
    // -----------------------------------------------------------------
    $display("\n===========================================");
    $display(" TEST SUMMARY");
    $display("===========================================");
    $display("Total Tests Run  : %0d", test_count);
    $display("TB Errors        : %0d", error_count);
    $display("DUT Error Count  : %0d  (Expected: 2)", error_count_out);

    // Pass criterion: 1 TB error from invalid write + 2 DUT-tracked errors
    if (error_count == 1 && error_count_out == 2)
        $display("\n[PASSED] CONGRATULATIONS! SYSTEM VERIFIED.\n");
    else
        $display("\n[FAILED] PLEASE CHECK WAVEFORMS.\n");

    $finish;
end

// =============================================================================
// Watchdog Timer
// =============================================================================
initial begin
    #50000; // 50 us timeout
    $display("TIMEOUT: Simulation exceeded 50 us limit!");
    $finish;
end

endmodule